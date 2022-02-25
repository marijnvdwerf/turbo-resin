// SPDX-License-Identifier: GPL-3.0-or-later

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(int_abs_diff)]
#![feature(type_alias_impl_trait)]
#![feature(maybe_uninit_as_bytes)]
#![feature(maybe_uninit_uninit_array)]
#![allow(unused_imports, dead_code, unused_variables, unused_macros, unreachable_code)]

#![feature(core_intrinsics)]

extern crate alloc;

mod drivers;
mod consts;
mod ui;
mod util;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use lvgl::core::{Lvgl, TouchPad, Display, ObjExt};

use embassy::{
    time::{Duration, Timer},
    util::Forever,
    executor::InterruptExecutor,
    interrupt::InterruptExt,
    blocking_mutex::CriticalSectionMutex as Mutex,
};
use embassy_stm32::{Config, interrupt};

use consts::display::*;
use drivers::{
    machine::Machine,
    touch_screen::{TouchEvent, TouchScreen},
    display::Display as RawDisplay,
    zaxis,
    usb::UsbHost,
};
use util::TaskRunner;
use util::SharedWithInterrupt;
pub(crate) use runtime::debug;


static LAST_TOUCH_EVENT: Mutex<RefCell<Option<TouchEvent>>> = Mutex::new(RefCell::new(None));
static Z_AXIS: Forever<zaxis::MotionControlAsync> = Forever::new();
static USB_HOST: Forever<UsbHost> = Forever::new();

static TASK_RUNNER: Forever<TaskRunner<ui::Task>> = Forever::new();

mod maximum_priority_tasks {
    use super::*;

    #[interrupt]
    fn TIM7() {
        unsafe { Z_AXIS.steal().on_interrupt() }
    }
}

mod high_priority_tasks {
    use super::*;

    #[interrupt]
    fn OTG_FS() {
        unsafe { USB_HOST.steal().on_interrupt() }
    }
}

mod medium_priority_tasks {
    use super::*;

    #[embassy::task]
    pub async fn usb_stack() {
        let usb_host = unsafe { USB_HOST.steal() };
        usb_host.main_loop().await;
    }

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
    pub async fn main_task() {
        let z_axis = unsafe { Z_AXIS.steal() };
        let task_runner = unsafe { TASK_RUNNER.steal() };
        task_runner.run_tasks(z_axis).await;
    }
}

mod low_priority_tasks {
    use crate::drivers::touch_screen;

    use super::*;

    pub fn idle_task(
        mut lvgl: Lvgl,
        mut display: Display<RawDisplay>,
    ) -> ! {
        let mut lvgl_input_device = lvgl::core::InputDevice::<TouchPad>::new(&mut display);

        let mut ui = ui::new_screen(&display, |screen| {
            let z_axis = unsafe { Z_AXIS.steal() };
            let task_runner = unsafe { TASK_RUNNER.steal() };
            ui::MoveZ::new(screen, task_runner, z_axis)
        });

        display.load_screen(&mut ui);

        loop {
            ui.context().as_mut().unwrap().refresh();

            LAST_TOUCH_EVENT.lock(|e| {
                *lvgl_input_device.state() = touch_screen::into_lvgl_event(&e.borrow());
            });

            lvgl.run_tasks();
            display.backlight.set_high();
        }
    }
}

fn lvgl_init(display: RawDisplay) -> (Lvgl, Display<RawDisplay>) {
    use embedded_graphics::pixelcolor::Rgb565;

    let mut lvgl = Lvgl::new();
    lvgl.register_logger(|s| rtt_target::rprint!(s));
    // Display init with its draw buffer
    static mut DRAW_BUFFER: [MaybeUninit<Rgb565>; LVGL_BUFFER_LEN] =
        [MaybeUninit::<Rgb565>::uninit(); LVGL_BUFFER_LEN];
    let display = Display::new(&lvgl, display, unsafe { &mut DRAW_BUFFER });
    (lvgl, display)
}

fn main() -> ! {
    rtt_target::rtt_init_print!(NoBlockSkip, 10240);

    let machine = {
        let p = {
            // We are doing the clock init here because of the gigadevice differences.
            let clk = crate::drivers::clock::setup_clock_120m_hxtal();
            let clk = crate::drivers::clock::embassy_stm32_clock_from(&clk);
            unsafe { embassy_stm32::rcc::set_freqs(clk) };

            // Note: TIM3 is taken for time accounting. It's configurable in Cargo.toml
            embassy_stm32::init(Config::default())
        };

        let cp = cortex_m::Peripherals::take().unwrap();
        Machine::new(cp, p)
    };

    Z_AXIS.put(zaxis::MotionControlAsync::new(
        SharedWithInterrupt::new(machine.stepper),
        machine.z_bottom_sensor,
    ));

    let (lvgl, display) = lvgl_init(machine.display);

    USB_HOST.put(machine.usb_host);

    //let mut lcd = machine.lcd;
    //lcd.draw_waves(16);

    // Maximum priority for the motion control of the stepper motor.
    // as we need to deliver precise pulses with micro-second accuracy.
    {
        let irq: interrupt::TIM7 = unsafe { ::core::mem::transmute(()) };
        irq.set_priority(interrupt::Priority::P4);
        irq.unpend();
        irq.enable();
    }

    // High priority for the USB port
    {
        let irq: interrupt::OTG_FS = unsafe { ::core::mem::transmute(()) };
        irq.set_priority(interrupt::Priority::P5);
        //irq.unpend();
        irq.enable();
    }

    // Medium priority executor. It interrupts the low priority tasks (UI rendering)
    {
        let lvgl_ticks = lvgl.ticks();
        let touch_screen = machine.touch_screen;
        let irq = interrupt::take!(CAN1_RX0);
        irq.set_priority(interrupt::Priority::P6);
        static EXECUTOR_MEDIUM: Forever<InterruptExecutor<interrupt::CAN1_RX0>> = Forever::new();
        let executor = EXECUTOR_MEDIUM.put(InterruptExecutor::new(irq));
        executor.start(|spawner| {
            spawner.spawn(medium_priority_tasks::touch_screen_task(touch_screen)).unwrap();
            spawner.spawn(medium_priority_tasks::lvgl_tick_task(lvgl_ticks)).unwrap();
            spawner.spawn(medium_priority_tasks::main_task()).unwrap();
            spawner.spawn(medium_priority_tasks::usb_stack()).unwrap();
        });
    }

    // TODO release the stack

    // The idle task does UI drawing continuously.
    low_priority_tasks::idle_task(lvgl, display)
}

// Wrap main(), otherwise auto-completion with rust-analyzer doesn't work.
#[cortex_m_rt::entry]
fn main_() -> ! { main() }

pub mod runtime {
    use super::*;

    #[alloc_error_handler]
    fn oom(l: core::alloc::Layout) -> ! {
        panic!("Out of memory. Failed to allocate {} bytes", l.size());
    }

    pub fn print_stack_size() {
        debug!("stack size: {:x?}",  0x20000000 + 96*1024 - ((&mut [0u8;1]).as_ptr() as u32));
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
