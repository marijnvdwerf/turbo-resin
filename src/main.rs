// SPDX-License-Identifier: GPL-3.0-or-later

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(int_abs_diff)]
#![allow(unused_imports, dead_code, unused_variables, unused_macros, unreachable_code)]
#![feature(type_alias_impl_trait)]

#![feature(core_intrinsics)]

mod drivers;
mod consts;
mod ui;
mod util;

use alloc::format;
use lvgl::style::State;
use stm32f1xx_hal::pac::Interrupt;
use consts::system::*;
use consts::display::*;
use drivers::{
    machine::Machine,
    //machine::{Systick, Machine, prelude::*},
    touch_screen::{TouchEvent, TouchScreen, ADS7846},
    display::Display as RawDisplay,
    zaxis,
};

use embedded_graphics::pixelcolor::Rgb565;

use lvgl::core::{
    Lvgl, TouchPad, Display, InputDevice, ObjExt
};


pub(crate) use runtime::debug;
extern crate alloc;

use core::cell::RefCell;
use core::mem::MaybeUninit;
use lvgl::core::Screen;

mod runtime {
    use super::*;

    #[alloc_error_handler]
    fn oom(l: core::alloc::Layout) -> ! {
        panic!("Out of memory. Failed to allocate {} bytes", l.size());
    }

    #[inline(never)]
    #[panic_handler]
    fn panic(info: &core::panic::PanicInfo) -> ! {
        debug!("{}", info);
        loop {}
    }

    macro_rules! debug {
        ($($tt:tt)*) => {
            rtt_target::rprintln!($($tt)*)
        }
    }
    pub(crate) use debug;
}


use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_stm32::Peripherals;
use embassy::util::Forever;
use embassy_stm32::interrupt;
use embassy::executor::{Executor, InterruptExecutor};
use embassy::interrupt::InterruptExt;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy::channel::signal::Signal;
use embassy_stm32::time::U32Ext;

use embassy::blocking_mutex::CriticalSectionMutex as Mutex;

use util::SharedWithInterrupt;

static LAST_TOUCH_EVENT: Mutex<RefCell<Option<TouchEvent>>> = Mutex::new(RefCell::new(None));
static Z_AXIS: Forever<SharedWithInterrupt<zaxis::MotionControl>> = Forever::new();

static USER_ACTION: Signal<crate::ui::UserAction> = Signal::new();

/// Maximum priority Tasks
mod maximum_priority_tasks {
    use super::*;

    #[interrupt]
    fn TIM7() {
        unsafe { Z_AXIS.steal().lock_from_interrupt(|z| z.on_interrupt()) }
    }
}

mod high_priority_tasks {
    use super::*;

    #[embassy::task]
    pub async fn touch_screen_task(mut touch_screen: TouchScreen) {
        loop {
            // What should happen if lvgl is not pumping click events fast enought?
            // We have different solutions. But here we go with last event wins.
            // If we had a keyboard, we would queue up events to avoid loosing key
            // presses.
            let touch_event = touch_screen.get_next_touch_event().await;
            LAST_TOUCH_EVENT.lock(|e| *e.borrow_mut() = touch_event);
        }
    }

    #[embassy::task]
    pub async fn lvgl_tick_task(mut lvgl_ticks: lvgl::core::Ticks) {
        loop {
            lvgl_ticks.inc(1);
            Timer::after(Duration::from_millis(1)).await
        }
    }

    #[embassy::task]
    pub async fn main_task(
        zaxis: &'static SharedWithInterrupt<zaxis::MotionControl>,
    ) {
        // Here is the control center, coordinating the printer hardware.
        // We react to user input, and do something with it.
        loop {
            let user_action = USER_ACTION.wait().await;
            debug!("Executing user action: {:?}", user_action);
            user_action.do_user_action(zaxis).await;
            debug!("Done with user action");
        }
    }
}

mod low_priority_tasks {
    use super::*;

    pub fn idle_task(
        mut lvgl: Lvgl,
        mut lvgl_input_device: InputDevice<TouchPad>,
        mut display: Display<RawDisplay>,
        mut ui: Screen<ui::MoveZ>,
    ) -> ! {
        loop {
            let z_axis = unsafe { Z_AXIS.steal() };
            let ui_state = z_axis.lock(|z_axis|
                ui::UiState {
                    zaxis_idle: z_axis.is_idle(),
                    zaxis_current_position: z_axis.current_position,
                    zaxis_max_speed: z_axis.get_max_speed(),
                }
            );
            ui.context().as_mut().unwrap().update_ui(ui_state);

            LAST_TOUCH_EVENT.lock(|e| {
                *lvgl_input_device.state() = if let Some(e) = e.borrow().as_ref() {
                    TouchPad::Pressed { x: e.x as i16, y: e.y as i16 }
                } else {
                    TouchPad::Released
                };
            });

            lvgl.run_tasks();

            display.backlight.set_high();
        }
    }
}

fn lvgl_init(display: RawDisplay) -> (Lvgl, Display<RawDisplay>) {
    let mut lvgl = Lvgl::new();
    lvgl.register_logger(|s| rtt_target::rprint!(s));
    // Display init with its draw buffer
    static mut DRAW_BUFFER: [MaybeUninit<Rgb565>; LVGL_BUFFER_LEN] =
        [MaybeUninit::<Rgb565>::uninit(); LVGL_BUFFER_LEN];
    let display = Display::new(&lvgl, display, unsafe { &mut DRAW_BUFFER });
    (lvgl, display)
}

fn main() -> ! {
    rtt_target::rtt_init_print!();

    let p = {
        // We are doing the clock init here because of the gigadevice differences.
        crate::drivers::clock::setup_clock_120m_hxtal();
        let clk = embassy_stm32::rcc::Clocks {
            sys: 120.mhz().into(),
            apb1: 120.mhz().into(),
            apb2: 60.mhz().into(),
            apb1_tim: 120.mhz().into(),
            apb2_tim: 60.mhz().into(),
            ahb1: 120.mhz().into(),
            adc: 30.mhz().into(),
        };
        unsafe { embassy_stm32::rcc::set_freqs(clk) };

        // Note: TIM3 is taken for time accounting. It's configurable in Cargo.toml
        embassy_stm32::init(Config::default())
    };

    let cp = cortex_m::Peripherals::take().unwrap();
    let machine = Machine::new(cp, p);
    let touch_screen = machine.touch_screen;
    let zaxis: &'static _ = Z_AXIS.put(SharedWithInterrupt::new(machine.stepper));

    let (lvgl, mut display) = lvgl_init(machine.display);
    let lvgl_input_device = lvgl::core::InputDevice::<TouchPad>::new(&mut display);
    let lvgl_ticks = lvgl.ticks();

    let mut move_z_ui = ui::MoveZ::new(&display, &USER_ACTION);
    display.load_screen(&mut move_z_ui);

    // Maximum priority for the motion control of the stepper motor.
    // as we need to deliver precise pulses with micro-second accuracy.
    {
        let irq: interrupt::TIM7 = unsafe { ::core::mem::transmute(()) };
        irq.set_priority(interrupt::Priority::P5);
        irq.unpend();
        irq.enable();
    }

    // High priority executor. It interrupts the low priority tasks (display rendering)
    {
        let irq = interrupt::take!(CAN1_RX0);
        irq.set_priority(interrupt::Priority::P6);
        static EXECUTOR_HIGH: Forever<InterruptExecutor<interrupt::CAN1_RX0>> = Forever::new();
        let executor = EXECUTOR_HIGH.put(InterruptExecutor::new(irq));
        executor.start(|spawner| {
            spawner.spawn(high_priority_tasks::touch_screen_task(touch_screen)).unwrap();
            spawner.spawn(high_priority_tasks::lvgl_tick_task(lvgl_ticks)).unwrap();
            spawner.spawn(high_priority_tasks::main_task(zaxis)).unwrap();
        });
    }

    // The idle task does UI drawing continuously.
    low_priority_tasks::idle_task(lvgl, lvgl_input_device, display, move_z_ui)
}

// Wrap main(), otherwise auto-completion with rust-analyzer doesn't work.
#[cortex_m_rt::entry]
fn main_() -> ! { main() }
