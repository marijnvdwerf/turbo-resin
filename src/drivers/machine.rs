// SPDX-License-Identifier: GPL-3.0-or-later

/*
use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::Timer,
    delay::Delay,
};
*/

use crate::drivers::{
    ext_flash::ExtFlash,
    display::Display,
    touch_screen::TouchScreen,
    zaxis::{
        MotionControl,
        BottomSensor,
        Drv8424,
    },
    lcd::Lcd,
    clock,
    touch_screen::*,
};

use crate::consts::system::*;


/*
pub type Systick = systick_monotonic::Systick<{ SYSTICK_HZ }>;
pub mod prelude {
    pub use systick_monotonic::ExtU64;
}
*/

pub struct Machine {
    //pub ext_flash: ExtFlash,
    pub display: Display,
    pub touch_screen: TouchScreen,
    //pub stepper: MotionControl,
    //pub systick: Systick,
    //pub lcd: Lcd,
    //pub z_bottom_sensor: BottomSensor,
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

impl Machine {
    pub fn new(/*cp: cortex_m::Peripherals,*/ p: Peripherals) -> Self {
        // Note, we can't use separate functions, because we are consuming (as
        // in taking ownership of) the device peripherals struct, and so we
        // cannot pass it as arguments to a function, as it would only be
        // partially valid.

        //--------------------------
        //  Clock configuration
        //--------------------------

        // Can't use the HAL. The GD32 is too different.
        //let clocks = clock::setup_clock_120m_hxtal(dp.RCC);
        //clock::CycleCounter::new(cp.DWT).into_global();

        //--------------------------
        //  External flash
        //--------------------------

        /*
        let ext_flash = ExtFlash::new(
            gpiob.pb12, gpiob.pb13, gpiob.pb14, gpiob.pb15,
            dp.SPI2,
            &clocks, &mut gpiob.crh
        );
        */

        //--------------------------
        //  TFT display
        //--------------------------

        //let _notsure = gpioa.pa6.into_push_pull_output(&mut gpioa.crl);
        let mut display = Display::new(
            p.PC6, p.PA10,
            p.PD4, p.PD5, p.PD7, p.PD11,
            p.PD14, p.PD15, p.PD0, p.PD1, p.PE7, p.PE8,
            p.PE9, p.PE10, p.PE11, p.PE12, p.PE13,
            p.PE14, p.PE15, p.PD8, p.PD9, p.PD10,
            //p.FSMC
        );
        display.init();

        //--------------------------
        //  Touch screen
        //--------------------------
        let touch_screen = TouchScreen::new(
            ADS7846::new(p.PC7, p.PC8, p.PC9, p.PA8, p.PA9, p.EXTI9)
        );

        //--------------------------
        // LCD Panel
        //--------------------------
        /*
        let lcd = Lcd::new(
            gpiod.pd12,
            gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7,
            dp.SPI1,
            &clocks, &mut gpioa.crl, &mut gpiod.crh, &mut afio.mapr,
        );
        */

        //--------------------------
        //  Stepper motor (Z-axis)
        //--------------------------

        /*
        let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // pb4 is used for TopSensor (or on Anycubic, it's the door sensor)
        let z_bottom_sensor = BottomSensor::new(pb3, &mut gpiob.crl);

        let drv8424 = Drv8424::new(
            gpioe.pe4, gpioe.pe5, gpioe.pe6,
            gpioc.pc3, gpioc.pc0,
            gpioc.pc1, gpioc.pc2,
            gpioa.pa3,
            Timer::new(dp.TIM2, &clocks),
            &mut gpioa.crl, gpioc.crl, &mut gpioe.crl, &mut afio.mapr,
        );

        let stepper = MotionControl::new(drv8424, Timer::new(dp.TIM7, &clocks));

         */


        Self { display, touch_screen }
        //Self { ext_flash, /*display, touch_screen,*/ stepper, lcd, z_bottom_sensor, systick }
    }
}
