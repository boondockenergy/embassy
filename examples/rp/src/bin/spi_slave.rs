//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2040 chip.
//! No specific hardware is specified in this example. If you connect pin 11 and 12 you should get the same data back.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::spi_slave::{Config,SpiSlave,InterruptHandler};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::SPI1;

bind_interrupts!(struct Irqs {
    SPI1_IRQ => InterruptHandler<SPI1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Hello World!");

    let mosi = p.PIN_12;
    let miso = p.PIN_11;
    let clk = p.PIN_10;
    let cs = p.PIN_13;

    let mut spi = SpiSlave::new(p.SPI1, clk, mosi, miso, cs, p.DMA_CH0, p.DMA_CH1, Config::default());

    loop {
        // Create a receive buffer
        let mut rx_buf = [0_u8; 64];

        // Receive data from the SPI master
        // This may return fewer bytes than are available in the buffer
        // and read() will return the number of bytes received from the master.
        // The transaction ends when the SPI hardware indicates a receive timeout
        let rxlen = spi.read(&mut rx_buf).await.unwrap();
        info!("{:X}", rx_buf[..rxlen]);
    }
}
