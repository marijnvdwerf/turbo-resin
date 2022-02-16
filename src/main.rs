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

use alloc::format;
use lvgl::style::State;
use stm32f1xx_hal::pac::Interrupt;
use consts::system::*;
use consts::display::*;
use drivers::{
    machine::{Systick, Machine, prelude::*},
    touch_screen::{TouchEvent, TouchScreen, ADS7846},
    display::Display as RawDisplay,

    zaxis::{
        BottomSensor,
        MotionControl,
    }
};

use embedded_graphics::pixelcolor::Rgb565;

use lvgl::core::{
    Lvgl, TouchPad, Display, InputDevice, ObjExt
};


pub(crate) use runtime::debug;

extern crate alloc;

use core::mem::MaybeUninit;

use lvgl::core::Screen;

mod runtime {
    use super::*;

    /*
    #[global_allocator]
    static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

    pub fn init_heap() {
        // Using cortex_m_rt::heap_start() is bad. It doesn't tell us if our
        // HEAP_SIZE is too large and we will fault accessing non-existing RAM
        // Instead, we'll allocate a static buffer for our heap.
        unsafe {
            static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            ALLOCATOR.init((&mut HEAP).as_ptr() as usize, HEAP_SIZE);
        }
    }
    */

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

#[embassy::task]
async fn touch_screen_task(mut touch_screen: TouchScreen) {
    loop {
        let touch_event = touch_screen.get_next_touch_event().await;
        debug!("touch_event: {:?}", touch_event);
    }
}

#[embassy::task]
async fn main_task() {
    loop {

    }
}

//static EXECUTOR_HIGH: Forever<InterruptExecutor<interrupt::UART4>> = Forever::new();
static EXECUTOR_MED: Forever<InterruptExecutor<interrupt::UART5>> = Forever::new();
static EXECUTOR_LOW: Forever<Executor> = Forever::new();

fn main() -> ! {
    rtt_target::rtt_init_print!();

    let mut config = Config::default();
    config.rcc.sys_ck = Some(Hertz(36_000_000));

    // TIM3 is taken for time accounting. It's configurable in Cargo.toml
    let p = embassy_stm32::init(config);

    let touch_screen = TouchScreen::new(
        ADS7846::new(p.PC7, p.PC8, p.PC9, p.PA8, p.PA9, p.EXTI9));



    /*
    // High-priority executor: SWI1_EGU1, priority level 6
    let irq = interrupt::take!(UART4);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_HIGH.put(InterruptExecutor::new(irq));
    executor.start(|spawner| {
        spawner.spawn(run_high());
    });
    */

    // Medium-priority executor: SWI0_EGU0, priority level 7
    let irq = interrupt::take!(UART5);
    irq.set_priority(interrupt::Priority::P7);
    let executor = EXECUTOR_MED.put(InterruptExecutor::new(irq));
    executor.start(|spawner| {
        spawner.spawn(touch_screen_task(touch_screen)).unwrap();
    });

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.put(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main_task()).unwrap();
    });
}

// Wrap main(), otherwise auto-completion with rust-analyzer doesn't work.
#[cortex_m_rt::entry]
fn main_() -> ! { main() }


/*
async fn main(spawner: Spawner, p: Peripherals) -> ! {
    debug!("Running from embassy!");

    let mut backlight = Output::new(p.PA10, Level::Low, Speed::Low);

    loop {
        debug!("high");
        backlight.set_high();
        Timer::after(Duration::from_millis(300)).await;

        debug!("low");
        backlight.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
*/

/*
#[rtic::app(
    device = stm32f1xx_hal::stm32, peripherals = true,
    // Picked random interrupts that we'll never use. RTIC will use this to schedule tasks.
    dispatchers=[CAN_RX1, CAN_SCE, CAN2_RX0, CAN2_RX1]
)]
mod app {

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonotonicClock = Systick;

    /* resources shared across RTIC tasks */
    #[shared]
    struct Shared {
        stepper: MotionControl,
        #[lock_free]
        touch_screen: drivers::touch_screen::TouchScreen,
        last_touch_event: Option<TouchEvent>,
    }

    /* resources local to specific RTIC tasks */
    #[local]
    struct Local {
        lvgl: Lvgl,
        lvgl_ticks: lvgl::core::Ticks,
        lvgl_input_device: InputDevice::<TouchPad>,
        display: Display::<RawDisplay>,
        move_z_ui: Screen<ui::MoveZ>,
        lcd: drivers::lcd::Lcd,
        z_bottom_sensor: BottomSensor,
    }

    fn lvgl_init(display: RawDisplay) -> (Lvgl, Display<RawDisplay>, InputDevice<TouchPad>) {
        let mut lvgl = Lvgl::new();

        // Register logger
        lvgl.register_logger(|s| rtt_target::rprint!(s));

        static mut DRAW_BUFFER: [MaybeUninit<Rgb565>; LVGL_BUFFER_LEN] =
            [MaybeUninit::<Rgb565>::uninit(); LVGL_BUFFER_LEN];

        let mut display = Display::new(&lvgl, display, unsafe { &mut DRAW_BUFFER });

        let input_device = lvgl::core::InputDevice::<TouchPad>::new(&mut display);

        (lvgl, display, input_device)
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_target::rtt_init_print!();
        debug!("Init...");

        lvgl::core::Lvgl::new();

        let machine = Machine::new(ctx.core, ctx.device);

        let display = machine.display;
        let systick = machine.systick;
        let stepper = machine.stepper;
        let touch_screen = machine.touch_screen;
        let lcd = machine.lcd;
        let z_bottom_sensor = machine.z_bottom_sensor;

        let (mut lvgl, mut display, lvgl_input_device) = lvgl_init(display);

        let lvgl_ticks = lvgl.ticks();
        lvgl_tick_task::spawn().unwrap();

        let last_touch_event = None;

        let mut move_z_ui = ui::MoveZ::new(&display);
        // Fill the display with something before turning it on.
        display.load_screen(&mut move_z_ui);
        lvgl.run_tasks();
        display.backlight.set_high();

        /*
        let ext_flash = machine.ext_flash;
        let delay = machine.delay;
        */

        debug!("Init complete");

        (
            Shared { stepper, touch_screen, last_touch_event },
            Local { lvgl, lvgl_ticks, lvgl_input_device, display, move_z_ui, lcd, z_bottom_sensor },
            init::Monotonics(systick),
        )
    }

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

    #[task(priority = 2, binds = EXTI9_5, shared = [touch_screen])]
    fn touch_screen_pen_down_interrupt(ctx: touch_screen_pen_down_interrupt::Context) {
        use TouchScreenResult::*;
        match ctx.shared.touch_screen.on_pen_down_interrupt() {
            DelayMs(delay_ms) => {
                cortex_m::peripheral::NVIC::mask(Interrupt::EXTI9_5);
                touch_screen_sampling_task::spawn_after((delay_ms as u64).millis()).unwrap();
            }
            Done(None) => {},
            Done(Some(_)) => unreachable!(),
        }
    }

    #[task(priority = 2, local = [], shared = [touch_screen, last_touch_event])]
    fn touch_screen_sampling_task(mut ctx: touch_screen_sampling_task::Context) {
        use TouchScreenResult::*;
        match ctx.shared.touch_screen.on_delay_expired() {
            DelayMs(delay_ms) => {
                touch_screen_sampling_task::spawn_after((delay_ms as u64).millis()).unwrap();
            },
            Done(touch_event) => {
                ctx.shared.last_touch_event.lock(|t| *t = touch_event);
                unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI9_5); }
            },
        }
    }

    #[idle(local = [lvgl, lvgl_input_device, display, move_z_ui, lcd, z_bottom_sensor], shared = [last_touch_event, stepper])]
    fn idle(mut ctx: idle::Context) -> ! {
        let lvgl = ctx.local.lvgl;
        let lvgl_input_device = ctx.local.lvgl_input_device;
        let zsensor = ctx.local.z_bottom_sensor;
        let move_z_ui = ctx.local.move_z_ui.context().as_mut().unwrap();

        loop {
            ctx.shared.last_touch_event.lock(|e| {
                *lvgl_input_device.state() = if let Some(ref e) = e {
                    TouchPad::Pressed { x: e.x as i16, y: e.y as i16 }
                } else {
                    TouchPad::Released
                };
            });

            move_z_ui.update(&mut ctx.shared.stepper, zsensor);
            lvgl.run_tasks();
        }
    }
}
*/


    /*
    fn draw_touch_event(display: &mut Display, touch_event: Option<&TouchEvent>) {
        use embedded_graphics::{prelude::*, primitives::{Circle, PrimitiveStyle}, pixelcolor::Rgb565};

        if let Some(touch_event) = touch_event {
            Circle::new(Point::new(touch_event.x as i32, touch_event.y as i32), 3)
                .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
                .draw(display).unwrap();
        }
    }
*/
