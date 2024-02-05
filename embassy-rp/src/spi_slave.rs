//! Serial Peripheral Interface
use core::marker::PhantomData;
use core::future::poll_fn;

use embassy_hal_internal::{into_ref, PeripheralRef};

use crate::dma::{AnyChannel, Channel};
use crate::gpio::sealed::Pin as _;
use crate::gpio::{AnyPin, Pin as GpioPin};
use crate::interrupt::typelevel::{Interrupt};
use crate::{interrupt, pac, peripherals, Peripheral};

use embassy_sync::waitqueue::AtomicWaker;
use atomic_polyfill::{AtomicU8, AtomicBool, Ordering};
use core::task::Poll;

/// SPI errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Invalid IRQ state
    InvalidIrqState
}

/// SPI Clock Polarity (CPOL)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockPolarity {
    /// Clock idles low (CPOL=0)
    IdleLow = 0,
    /// Clock idles high (CPOL=1)
    IdleHigh = 1,
}

/// SPI Clock Phase (CPHA)
/// Note that the PL022 IP used in the RP2040
/// will only assert CS for the whole transaction when CPHA=1
/// See ARM PrimeCell PL022 documentation for details
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockPhase {
    /// Sample on first edge (CPHA=0)
    FirstEdge = 0,
    /// Sample on second edge (CPHA=1)
    SecondEdge = 1,
}

/// SPI configuration.
#[non_exhaustive]
#[derive(Clone)]
pub struct Config {
    polarity: ClockPolarity,
    phase: ClockPhase,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            polarity: ClockPolarity::IdleLow,
            phase: ClockPhase::SecondEdge,
        }
    }
}

/// Internal DMA state of SPI RX.
pub struct DmaState {
    rx_waker: AtomicWaker,
    rx_irq_state: AtomicU8,
    rx_timeout: AtomicBool,
}

/// SPI slave driver.
pub struct SpiSlave<'d, T: Instance, M: Mode> {
    inner: PeripheralRef<'d, T>,
    tx_dma: Option<PeripheralRef<'d, AnyChannel>>,
    rx_dma: Option<PeripheralRef<'d, AnyChannel>>,
    phantom: PhantomData<(&'d mut T, M)>,
}

impl<'d, T: Instance, M: Mode> SpiSlave<'d, T, M> {
    fn new_inner(
        inner: impl Peripheral<P = T> + 'd,
        clk: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        cs: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<PeripheralRef<'d, AnyChannel>>,
        rx_dma: Option<PeripheralRef<'d, AnyChannel>>,
        config: Config,
    ) -> Self {
        into_ref!(inner);

        let p = T::regs();

        p.cr0().write(|w| {
            w.set_dss(0b0111); // 8bit
            w.set_spo(config.polarity == ClockPolarity::IdleHigh);
            w.set_sph(config.phase == ClockPhase::SecondEdge);
        });

        // Always enable DREQ signals -- harmless if DMA is not listening
        p.dmacr().write(|reg| {
            reg.set_rxdmae(true);
            reg.set_txdmae(true);
        });


        // Set slave mode, and enable.
        p.cr1().write(|w| {
            w.set_ms(true);
            w.set_sse(true);
        });

        if let Some(pin) = &clk {
            pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        }
        if let Some(pin) = &mosi {
            pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        }
        if let Some(pin) = &miso {
            pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        }
        if let Some(pin) = &cs {
            pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        }

        // Set interrupt masks
        p.imsc().write(|w| {
            w.set_rorim(true);
            w.set_rtim(true);

            // Interrupt when FIFO is half full (4 words)
            w.set_rxim(true);
        });

        // Enable interrupts
        unsafe { T::Interrupt::enable() }

        Self {
            inner,
            tx_dma,
            rx_dma,
            phantom: PhantomData,
        }
    }
}

impl<'d, T: Instance> SpiSlave<'d, T, Async> {
    /// Create an SPI slave driver in async mode supporting DMA operations.
    pub fn new(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T> + 'd> + 'd,
        cs: impl Peripheral<P = impl CsPin<T> + 'd> + 'd,
        tx_dma: impl Peripheral<P = impl Channel> + 'd,
        rx_dma: impl Peripheral<P = impl Channel> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(tx_dma, rx_dma, clk, mosi, miso, cs);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            Some(cs.map_into()),
            Some(tx_dma.map_into()),
            Some(rx_dma.map_into()),
            config,
        )
    }

    /// Create an SPI slave driver in async mode supporting DMA write operations only.
    pub fn new_txonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        tx_dma: impl Peripheral<P = impl Channel> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(tx_dma, clk, mosi);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            Some(mosi.map_into()),
            None,
            None,
            Some(tx_dma.map_into()),
            None,
            config,
        )
    }

    /// Create an SPI slave driver in async mode supporting DMA read operations only.
    pub fn new_rxonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        cs: impl Peripheral<P = impl CsPin<T> + 'd> + 'd,
        rx_dma: impl Peripheral<P = impl Channel> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(rx_dma, clk, mosi, cs);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            Some(mosi.map_into()),
            None,
            Some(cs.map_into()),
            None,
            Some(rx_dma.map_into()),
            config,
        )
    }

    /// Read data from SPI using DMA.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {

        let mut rx_pos = 0usize;
        let mut remain = buffer.len();

        while remain != 0 {

            // Wait for an interrupt
            let irq_state = poll_fn(|cx| {
                T::dma_state().rx_waker.register(cx.waker());
                match T::dma_state().rx_irq_state.swap(0, Ordering::Relaxed) {
                    0 => Poll::Pending,
                    x => Poll::Ready(x),
                }
            }).await;

            match irq_state.into() {
                InterruptState::RxTimeout => {
                    // This indicates we have less than half a full RX FIFO,
                    // however some data still remains. Read one byte at a time
                    // until the RX FIFO is empty, or we've filled the destination buffer

                    while T::regs().sr().read().rne() {
                        let rx_ch = self.rx_dma.as_mut().unwrap();
                        let rx_transfer = unsafe {
                            crate::dma::read(rx_ch, T::regs().dr().as_ptr() as *const _, &mut buffer[rx_pos..rx_pos+1], T::RX_DREQ)
                        };
                        rx_transfer.await;

                        rx_pos += 1;
                        remain -= 1;

                        // If we've filled the dest buffer, break out of the inner loop
                        if remain == 0 {
                            break;
                        }
                    }
                }

                InterruptState::RxFifoHalfFull => {
                    // There are guaranteed to be 4 bytes in the RX FIFO.
                    let rx_ch = self.rx_dma.as_mut().unwrap();

                    // If we have enough room in the dest buffer, DMA transfer 4 bytes at rx_pos
                    if remain >= 4 {
                        let rx_transfer = unsafe {
                            crate::dma::read(rx_ch, T::regs().dr().as_ptr() as *const _, &mut buffer[rx_pos..rx_pos+4], T::RX_DREQ)
                        };
                        rx_transfer.await;

                        rx_pos += 4;
                        remain -= 4;
                    } else {
                        // Otherwise, receive as many bytes as we have available in the dest buffer
                        let rx_transfer = unsafe {
                            crate::dma::read(rx_ch, T::regs().dr().as_ptr() as *const _, &mut buffer[rx_pos..rx_pos+remain], T::RX_DREQ)
                        };
                        rx_transfer.await;

                        rx_pos += remain;
                        remain = 0;
                    }

                    // Unmask interrupts
                    T::regs().imsc().modify(|w| {
                        w.set_rxim(true);
                    });
                }

                _ => {
                    return Err(Error::InvalidIrqState);
                }
            }

            // Check if there was a timeout interrupt raised, and return to the caller
            // with the number of bytes received into the dest buffer
            match T::dma_state().rx_timeout.swap(false, Ordering::Relaxed) {
                true => return Ok(rx_pos),
                false => {}
            }
        }

        Ok(rx_pos)
    }
}

/// Interrupt state set by the interrupt handler and matched for match in the RX task
pub enum InterruptState {
    /// Waiting for an interrupt to fire
    Waiting,
    /// Receive timeout after 32 clock cycles
    /// This is used to assume the end of a transaction
    RxTimeout,
    /// RX FIFO is half full. There are least 4 bytes in the RX FIFO
    RxFifoHalfFull,
    /// RX Overflow
    RxOverflow,
    /// Invalid
    Invalid,
}

impl From<u8> for InterruptState {
    fn from(val: u8) -> InterruptState {
        match val {
            val if val == InterruptState::RxTimeout as u8 => InterruptState::RxTimeout,
            val if val == InterruptState::RxFifoHalfFull as u8 => InterruptState::RxFifoHalfFull,
            val if val == InterruptState::RxOverflow as u8 => InterruptState::RxOverflow,
            _ => InterruptState::Invalid
        }
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let state = T::dma_state();

        let status = r.ris().read();

        // Check if the RX FIFO half full interrupt has been raised
        if status.rxris() == true {
            // Mask the interrupts, which will be re-enabled after we start draining the FIFO
            r.imsc().modify(|w| {
                w.set_rxim(false);
            });

            state.rx_irq_state.store(InterruptState::RxFifoHalfFull as u8, Ordering::Relaxed);
            state.rx_waker.wake();
        }

        if status.rtris() == true {
            // RX timeout after 32 clocks (? unclear in docs)

            state.rx_timeout.store(true, Ordering::Relaxed);

            state.rx_irq_state.store(InterruptState::RxTimeout as u8, Ordering::Relaxed);
            state.rx_waker.wake();

            // Clear the RX timeout interrupt
            T::regs().icr().write(|w| {
                w.set_rtic(true);
            });
        }

        if status.rorris() == true {
            // RX overflow
            r.icr().write(|w| {
                w.set_roric(true);
            })
        }
    }
}

mod sealed {
    use super::*;

    pub trait Mode {}

    pub trait Instance {
        const TX_DREQ: u8;
        const RX_DREQ: u8;

        type Interrupt: interrupt::typelevel::Interrupt;

        fn regs() -> pac::spi::Spi;

        fn dma_state() -> &'static DmaState;
    }
}

/// Mode.
pub trait Mode: sealed::Mode {}

/// SPI instance trait.
pub trait Instance: sealed::Instance {}

macro_rules! impl_instance {
    ($type:ident, $irq:ident, $tx_dreq:expr, $rx_dreq:expr) => {
        impl sealed::Instance for peripherals::$type {
            const TX_DREQ: u8 = $tx_dreq;
            const RX_DREQ: u8 = $rx_dreq;

            type Interrupt = crate::interrupt::typelevel::$irq;

            fn regs() -> pac::spi::Spi {
                pac::$type
            }

            fn dma_state() -> &'static DmaState {
                static STATE: DmaState = DmaState {
                    rx_waker: AtomicWaker::new(),
                    rx_irq_state: AtomicU8::new(0),
                    rx_timeout: AtomicBool::new(false),
                };
                &STATE
            }
        }
        impl Instance for peripherals::$type {}
    };
}

impl_instance!(SPI0, SPI0_IRQ, 16, 17);
impl_instance!(SPI1, SPI1_IRQ, 18, 19);

/// CLK pin.
pub trait ClkPin<T: Instance>: GpioPin {}
/// CS pin.
pub trait CsPin<T: Instance>: GpioPin {}
/// MOSI pin.
pub trait MosiPin<T: Instance>: GpioPin {}
/// MISO pin.
pub trait MisoPin<T: Instance>: GpioPin {}

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $function:ident) => {
        impl $function<peripherals::$instance> for peripherals::$pin {}
    };
}

impl_pin!(PIN_0, SPI0, MosiPin);
impl_pin!(PIN_1, SPI0, CsPin);
impl_pin!(PIN_2, SPI0, ClkPin);
impl_pin!(PIN_3, SPI0, MisoPin);
impl_pin!(PIN_4, SPI0, MosiPin);
impl_pin!(PIN_5, SPI0, CsPin);
impl_pin!(PIN_6, SPI0, ClkPin);
impl_pin!(PIN_7, SPI0, MisoPin);
impl_pin!(PIN_8, SPI1, MosiPin);
impl_pin!(PIN_9, SPI1, CsPin);
impl_pin!(PIN_10, SPI1, ClkPin);
impl_pin!(PIN_11, SPI1, MisoPin);
impl_pin!(PIN_12, SPI1, MosiPin);
impl_pin!(PIN_13, SPI1, CsPin);
impl_pin!(PIN_14, SPI1, ClkPin);
impl_pin!(PIN_15, SPI1, MisoPin);
impl_pin!(PIN_16, SPI0, MosiPin);
impl_pin!(PIN_17, SPI0, CsPin);
impl_pin!(PIN_18, SPI0, ClkPin);
impl_pin!(PIN_19, SPI0, MisoPin);
impl_pin!(PIN_20, SPI0, MosiPin);
impl_pin!(PIN_21, SPI0, CsPin);
impl_pin!(PIN_22, SPI0, ClkPin);
impl_pin!(PIN_23, SPI0, MisoPin);
impl_pin!(PIN_24, SPI1, MosiPin);
impl_pin!(PIN_25, SPI1, CsPin);
impl_pin!(PIN_26, SPI1, ClkPin);
impl_pin!(PIN_27, SPI1, MisoPin);
impl_pin!(PIN_28, SPI1, MosiPin);
impl_pin!(PIN_29, SPI1, CsPin);

macro_rules! impl_mode {
    ($name:ident) => {
        impl sealed::Mode for $name {}
        impl Mode for $name {}
    };
}

/// Blocking mode.
pub struct Blocking;
/// Async mode.
pub struct Async;

impl_mode!(Blocking);
impl_mode!(Async);
