use stm32f1xx_hal::{
    pac::SPI2,
    gpio::*,
    gpio::gpiob::*,
    rcc::Clocks,
    spi::*,
    spi,
};

use spi_memory::{
    //prelude::*,
    series25::Flash,
};

pub struct ExtFlash(
    // Generated by the compiler when providing it an incorrect type.
    // It's pretty dense, but not really necessary.
    pub Flash<
        Spi<
            SPI2,
            Spi2NoRemap,
            (
                PB13<Alternate<PushPull>>,
                PB14<Input<Floating>>,
                PB15<Alternate<PushPull>>
            ),
            u8,
        >,
        PB12<Output<PushPull>,
    >>
);

// We are reading from the SPI flash at 1.16 MB/s
// We should be getting 3x the speed

impl ExtFlash {
    pub fn new(
        cs: PB12<Input<Floating>>,
        sck: PB13<Input<Floating>>,
        miso: PB14<Input<Floating>>,
        mosi: PB15<Input<Floating>>,
        spi2: SPI2,
        clocks: &Clocks,
        gpiob_crh: &mut Cr<CRH, 'B'>,
    ) -> Self {
        let cs = cs.into_push_pull_output_with_state(gpiob_crh, PinState::High);

        let spi = {
            let sck = sck.into_alternate_push_pull(gpiob_crh);
            let miso = miso.into_floating_input(gpiob_crh);
            let mosi = mosi.into_alternate_push_pull(gpiob_crh);

            Spi::spi2(
                spi2,
                (sck, miso, mosi),
                spi::Mode { polarity: spi::Polarity::IdleLow, phase: spi::Phase::CaptureOnFirstTransition },
                clocks.pclk1()/2, // Run as fast as we can (30Mhz). The flash chip can go up to 133Mhz.
                *clocks,
            )
        };

        // Initialize the spi-memory library
        ExtFlash(Flash::init(spi, cs).unwrap())
    }

    /*
    pub fn dump(&mut self) {
        use crate::consts::*;
        use crate::drivers::hio::{open, nr::open};

        const BUFFER_SIZE: usize = 32*1024; // 32KB
        let mut buf = [0; BUFFER_SIZE];

        let mut file = open("ext.bin\0", open::RW_TRUNC_BINARY).unwrap();

        for addr in (0..EXT_FLASH_SIZE).step_by(BUFFER_SIZE) {
            self.0.read(addr, &mut buf).unwrap();
            file.write_all(&buf).unwrap();
        }

        cortex_m_semihosting::hprintln!("DONE").unwrap();
    }
    */
}