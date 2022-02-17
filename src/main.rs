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

use embassy::blocking_mutex::CriticalSectionMutex as Mutex;

use util::SharedWithInterrupt;

static LAST_TOUCH_EVENT: Mutex<RefCell<Option<TouchEvent>>> = Mutex::new(RefCell::new(None));

static Z_AXIS: Forever<SharedWithInterrupt<zaxis::MotionControl>> = Forever::new();

#[embassy::task]
async fn touch_screen_task(mut touch_screen: TouchScreen) {
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
async fn lvgl_tick_task(mut lvgl_ticks: lvgl::core::Ticks) {
    loop {
        lvgl_ticks.inc(1);
        Timer::after(Duration::from_millis(1)).await
    }
}

fn run_lvgl_tasks(lvgl: &mut Lvgl, lvgl_input_device: &mut InputDevice<TouchPad>) {
    LAST_TOUCH_EVENT.lock(|e| {
        *lvgl_input_device.state() = if let Some(e) = e.borrow().as_ref() {
            TouchPad::Pressed { x: e.x as i16, y: e.y as i16 }
        } else {
            TouchPad::Released
        };
    });

    lvgl.run_tasks();
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


#[interrupt]
fn TIM7() {
    unsafe { Z_AXIS.steal().lock_from_interrupt(|z| z.on_interrupt()) }
}

fn main() -> ! {
    rtt_target::rtt_init_print!();

    // FIXME this is broken right now. We need a proper HAL.
    crate::drivers::clock::setup_clock_120m_hxtal();

    // TIM3 is taken for time accounting. It's configurable in Cargo.toml
    let mut config = Config::default();
    config.rcc.sys_ck = Some(Hertz(36_000_000));
    let p = embassy_stm32::init(config);

    let cp = cortex_m::Peripherals::take().unwrap();
    let machine = Machine::new(cp, p);
    let touch_screen = machine.touch_screen;

    let (mut lvgl, mut display) = lvgl_init(machine.display);
    let mut lvgl_input_device = lvgl::core::InputDevice::<TouchPad>::new(&mut display);
    let lvgl_ticks = lvgl.ticks();

    let mut move_z_ui = ui::MoveZ::new(&display);
    display.load_screen(&mut move_z_ui);

    // Maximum priority for the motion control of the stepper motor.
    // as we need to deliver precise pulses with micro-second accuracy.
    {
        let irq = interrupt::take!(TIM7);
        irq.set_priority(interrupt::Priority::P5);
        //irq.set_handler_context()
        //irq.set_handler()
    }

    // High priority executor. Good for managing the printer. The print
    // algorithm is executed there. It interrupts the display rendering.
    {
        let irq = interrupt::take!(CAN1_RX0);
        irq.set_priority(interrupt::Priority::P6);
        static EXECUTOR_HIGH: Forever<InterruptExecutor<interrupt::CAN1_RX0>> = Forever::new();
        let executor = EXECUTOR_HIGH.put(InterruptExecutor::new(irq));
        executor.start(|spawner| {
            //spawner.spawn(run_high());
        });
    }

    // Low priority executor. Good for processing UI related tasks.
    {
        let irq = interrupt::take!(CAN1_RX1);
        irq.set_priority(interrupt::Priority::P7);
        static EXECUTOR_LOW: Forever<InterruptExecutor<interrupt::CAN1_RX1>> = Forever::new();
        let executor = EXECUTOR_LOW.put(InterruptExecutor::new(irq));
        executor.start(|spawner| {
            spawner.spawn(touch_screen_task(touch_screen)).unwrap();
            spawner.spawn(lvgl_tick_task(lvgl_ticks)).unwrap();
        });
    }

    // This is the idle task.
    // The non-interrupt context is busy drawing things to the display
    // continuously. As we are not on a power-restricted device, we don't care
    // about sleeping.
    loop {
        run_lvgl_tasks(&mut lvgl, &mut lvgl_input_device);
        display.backlight.set_high();
    }
}

// Wrap main(), otherwise auto-completion with rust-analyzer doesn't work.
#[cortex_m_rt::entry]
fn main_() -> ! { main() }



/*
mod app {
    #[init]
    #[task(priority = 5, binds = TIM7, shared = [stepper])]
    fn stepper_interrupt(mut ctx: stepper_interrupt::Context) {
        ctx.shared.stepper.lock(|s| s.on_interrupt());
    }

    #[task(priority = 3, local = [lvgl_ticks], shared = [])]
    fn lvgl_tick_task(ctx: lvgl_tick_task::Context) {
        // Not very precise (by the time we get here, some time has passed
        // already), but good enough
        lvgl_tick_task::spawn_after(1.millis()).unwrap();
        ctx.local.lvgl_ticks.inc(1);
    }
}
*/
