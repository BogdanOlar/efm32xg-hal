//! Peripheral Reflex System

pub use crate::pac::prs::vals::ChCtrlEdsel;
use crate::{
    cmu::CmuClkOutId,
    gpio::exti::ExtiId,
    pac::{
        prs::{
            regs::{ChCtrl, Ctrl, Routepen, Swlevel, Swpulse},
            vals::ChCtrlSourcesel,
        },
        CMU,
    },
    peripherals,
    rtcc::RtccChannelId,
    timer::TimerChannelId,
    timer_le::LeTimerChannelId,
    Sealed,
};
use embassy_hal_internal::Peri;

/// A PRS peripheral instance usable by the HAL Prs driver.
pub trait PrsInstance: Sealed + embassy_hal_internal::PeripheralType + 'static {
    /// Returns the chiptool PAC register-block handle for this prs instance.
    fn regs(&self) -> crate::pac::prs::Prs;
}

impl Sealed for peripherals::Prs {}
impl PrsInstance for peripherals::Prs {
    fn regs(&self) -> crate::pac::prs::Prs {
        crate::pac::PRS
    }
}

/// Peripheral Reflex System driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Prs<'d, T: PrsInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: PrsInstance> Prs<'d, T> {
    /// New Peripheral Reflex System driver
    pub fn new(peri: Peri<'d, T>) -> Self {
        let instance = Self { peri };
        let p = instance.peri.regs();

        // Disable clock
        CMU.hfbusclken0().modify(|w| w.set_prs(false));

        // All registers have been verified to have a `0` reset value, so using the PAC default is fine in this case
        p.swpulse().write_value(Swpulse::default());
        p.swlevel().write_value(Swlevel::default());
        p.routepen().write_value(Routepen::default());
        p.ctrl().write_value(Ctrl::default());
        for i in 0..PrsChannelId::COUNT {
            p.ch_ctrl(i).write_value(ChCtrl::default());
        }

        // Enable clock
        CMU.hfbusclken0().modify(|w| w.set_prs(true));

        instance
    }

    /// Split the peripheral into channels
    pub fn split(self) -> PrsChannels {
        PrsChannels {
            ch0: PrsChannel::new(),
            ch1: PrsChannel::new(),
            ch2: PrsChannel::new(),
            ch3: PrsChannel::new(),
            ch4: PrsChannel::new(),
            ch5: PrsChannel::new(),
            ch6: PrsChannel::new(),
            ch7: PrsChannel::new(),
            ch8: PrsChannel::new(),
            ch9: PrsChannel::new(),
            ch10: PrsChannel::new(),
            ch11: PrsChannel::new(),
        }
    }
}

/// Container for all [`PrsChannel`]s
pub struct PrsChannels {
    /// PRS Channel 0
    pub ch0: PrsChannel<0>,
    /// PRS Channel 1
    pub ch1: PrsChannel<1>,
    /// PRS Channel 2
    pub ch2: PrsChannel<2>,
    /// PRS Channel 3
    pub ch3: PrsChannel<3>,
    /// PRS Channel 4
    pub ch4: PrsChannel<4>,
    /// PRS Channel 5
    pub ch5: PrsChannel<5>,
    /// PRS Channel 6
    pub ch6: PrsChannel<6>,
    /// PRS Channel 7
    pub ch7: PrsChannel<7>,
    /// PRS Channel 8
    pub ch8: PrsChannel<8>,
    /// PRS Channel 9
    pub ch9: PrsChannel<9>,
    /// PRS Channel 10
    pub ch10: PrsChannel<10>,
    /// PRS Channel 11
    pub ch11: PrsChannel<11>,
}

/// PRS Channel
pub struct PrsChannel<const CN: u8> {
    _private: (),
}

impl<const CN: u8> PrsChannel<CN> {
    fn new() -> Self {
        let mut instance = Self { _private: () };

        instance.set_source(PrsChSource::default());

        instance
    }

    /// Get the PRS channel ID
    pub fn id(&self) -> PrsChannelId {
        PrsChannelId::from_u8_unchecked(CN)
    }

    pub fn set_edge(&mut self, edge_sel: ChCtrlEdsel) {
        let r = Self::peri().ch_ctrl(CN as usize);
        r.modify(|w| w.set_edsel(edge_sel));
    }

    /// Set the channel source (producer)
    pub fn set_source(&mut self, source: PrsChSource) {
        let r = Self::peri().ch_ctrl(CN as usize);
        match source {
            PrsChSource::None => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::None);
                    w.set_sigsel(0);
                });
            }
            PrsChSource::Prs(ch) => {
                let i = ch as u8 / u8::BITS as u8;
                let rem = ch as u8 % u8::BITS as u8;
                r.modify(|w| {
                    if i == 0 {
                        w.set_sourcesel(ChCtrlSourcesel::Prsl);
                        w.set_sigsel(rem);
                    } else {
                        w.set_sourcesel(ChCtrlSourcesel::Prsh);
                        w.set_sigsel(rem);
                    }
                });
            }
            PrsChSource::Acmp0 => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Acmp0);
                    w.set_sigsel(0);
                });
            }
            PrsChSource::Acmp1 => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Acmp1);
                    w.set_sigsel(0);
                });
            }
            PrsChSource::Adc0(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Adc0);
                    w.set_sigsel(signal as u8);
                });
            }
            PrsChSource::Usart0(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Usart0);
                    w.set_sigsel(signal as u8);
                });
            }
            PrsChSource::Usart1(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Usart1);
                    w.set_sigsel(signal as u8);
                });
            }
            PrsChSource::Timer0(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Timer0);
                    match signal {
                        PrsTimerSignal::Underflow => w.set_sigsel(0),
                        PrsTimerSignal::Overflow => w.set_sigsel(1),
                        PrsTimerSignal::Cc(id) => w.set_sigsel((id as u8) + 2),
                    }
                });
            }
            PrsChSource::Timer1(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Timer1);
                    match signal {
                        PrsTimerSignal::Underflow => w.set_sigsel(0),
                        PrsTimerSignal::Overflow => w.set_sigsel(1),
                        PrsTimerSignal::Cc(id) => w.set_sigsel((id as u8) + 2),
                    }
                });
            }
            PrsChSource::Rtcc(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Rtcc);
                    w.set_sigsel(signal as u8 + 1);
                });
            }
            PrsChSource::Gpio(id) => {
                let i = id as u8 / u8::BITS as u8;
                let rem = id as u8 % u8::BITS as u8;
                r.modify(|w| {
                    if i == 0 {
                        w.set_sourcesel(ChCtrlSourcesel::Gpiol);
                        w.set_sigsel(rem);
                    } else {
                        w.set_sourcesel(ChCtrlSourcesel::Gpioh);
                        w.set_sigsel(rem);
                    }
                });
            }
            PrsChSource::Letimer0(id) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Letimer0);
                    w.set_sigsel(id as u8);
                });
            }
            PrsChSource::Pcnt0(signal) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Pcnt0);
                    w.set_sigsel(signal as u8);
                });
            }
            PrsChSource::Cryotimer => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Cryotimer);
                    w.set_sigsel(0);
                });
            }
            PrsChSource::Cmu(id) => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Cmu);
                    w.set_sigsel(id as u8);
                });
            }
            PrsChSource::Cm4 => {
                r.modify(|w| {
                    w.set_sourcesel(ChCtrlSourcesel::Cm4);
                    w.set_sigsel(0);
                });
            }
        }
    }

    fn peri() -> crate::pac::prs::Prs {
        crate::pac::PRS
    }
}

/// PRS channel ID
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsChannelId {
    /// Channel 0
    #[default]
    Ch0,
    /// Channel 1
    Ch1,
    /// Channel 2
    Ch2,
    /// Channel 3
    Ch3,
    /// Channel 4
    Ch4,
    /// Channel 5
    Ch5,
    /// Channel 6
    Ch6,
    /// Channel 7
    Ch7,
    /// Channel 8
    Ch8,
    /// Channel 9
    Ch9,
    /// Channel 10
    Ch10,
    /// Channel 11
    Ch11,
}

impl PrsChannelId {
    /// Number of PRS channels
    const COUNT: usize = 12;
    const fn from_u8_unchecked(n: u8) -> Self {
        match n {
            0 => Self::Ch0,
            1 => Self::Ch1,
            2 => Self::Ch2,
            3 => Self::Ch3,
            4 => Self::Ch4,
            5 => Self::Ch5,
            6 => Self::Ch6,
            7 => Self::Ch7,
            8 => Self::Ch8,
            9 => Self::Ch9,
            10 => Self::Ch10,
            11 => Self::Ch11,
            _ => unreachable!(),
        }
    }
}

/// PRS sources (producers)
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PrsChSource {
    /// No source selected
    #[default]
    None,
    /// Peripheral Reflex System
    Prs(PrsChannelId),
    /// Analog Comparator 0
    Acmp0,
    /// Analog Comparator 1
    Acmp1,
    /// Analog to Digital Converter 0
    Adc0(PrsAdcSignal),
    /// Universal Synchronous/Asynchronous Receiver/Transmitter 0
    Usart0(PrsUsart0Signal),
    /// Universal Synchronous/Asynchronous Receiver/Transmitter 1
    Usart1(PrsUsart1Signal),
    /// Timer 0
    Timer0(PrsTimerSignal),
    /// Timer 1
    Timer1(PrsTimerSignal),
    /// Real-Time Counter and Calendar
    Rtcc(RtccChannelId),
    /// General purpose Input/Output
    ///
    /// Note that GPIO producers are selected in the GPIO module using the edge interrupt configuration settings
    /// described in 26.3.5.1 Edge Interrupt Generation.
    ///
    /// GPIOPIN0 uses the selection for the EXTI0 interrupt, GPIOPIN1 uses the selection for the EXTI1 interrupt, etc.
    Gpio(ExtiId),
    /// Low Energy Timer 0
    Letimer0(LeTimerChannelId),
    /// Pulse Counter 0
    Pcnt0(PrsPcntSignal),
    /// Cryotimer period
    Cryotimer,
    /// Clock Management Unit
    Cmu(CmuClkOutId),
    /// Core
    Cm4,
}

/// ADC signals
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsAdcSignal {
    /// ADC single conversion done
    #[default]
    Single = 0,
    /// ADC scan conversion done
    Scan,
}

/// Usart0 signals
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsUsart0Signal {
    /// USART IRDA out USART0IRTX
    #[default]
    IrTx = 0b000,
    /// USART TX complete
    TxComplete = 0b001,
    /// USART RX Data Valid
    RxDataValid = 0b010,
    /// USART RTS
    Rts = 0b011,
    /// USART TX
    Tx = 0b101,
    /// USART CS
    Cs = 0b110,
}

/// Usart1 signals
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsUsart1Signal {
    /// USART TX complete
    #[default]
    TxComplete = 0b001,
    /// USART RX Data Valid
    RxDataValid = 0b010,
    /// USART RTS
    Rts = 0b011,
    /// USART TX
    Tx = 0b101,
    /// USART CS
    Cs = 0b110,
}

/// Timer signals
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsTimerSignal {
    /// Timer Underflow
    #[default]
    Underflow = 0,
    /// Timer Overflow
    Overflow,
    /// Timer Capture/Compare
    Cc(TimerChannelId),
}

/// Pcnt signals
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PrsPcntSignal {
    /// Triggered compare match
    #[default]
    Compare = 0,
    /// Counter overflow or underflow
    CounterOfUf,
    /// Counter direction
    CounterDir,
}
