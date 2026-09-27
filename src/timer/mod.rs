//! Timer/Counter
//!

pub use crate::pac::{
    prs::vals::Prssel,
    timer::vals::{
        CcCtrlCmoa, CcCtrlIcedge, CcCtrlIcevctrl, CcCtrlMode, CcLoc, Clksel as Source,
        CtrlMode as Mode, Presc as Prescaler,
    },
};
use crate::{
    cmu::Clocks,
    gpio::pin::Pin,
    pac::{CMU, TIMER0, TIMER1},
    peripherals, Sealed,
};
use core::{convert::Infallible, marker::PhantomData};
use embassy_hal_internal::Peri;
use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    pwm::{ErrorType, SetDutyCycle},
};

/// A timer peripheral instance usable by the HAL timer driver.
///
/// This is a sealed trait implemented only for the singleton types in [`crate::peripherals`]:
/// [`peripherals::Timer0`] and [`peripherals::Timer1`]. Because each instance is a distinct,
/// uninstantiable type obtained only from [`crate::efm32_init`], a timer peripheral cannot be
/// driven by two [`Timer`] instances at once — the singleton is moved into the first driver and
/// any second use fails to compile.
pub trait TimerInstance: Sealed + embassy_hal_internal::PeripheralType + 'static {
    /// Returns the chiptool PAC register-block handle for this timer instance.
    fn regs(&self) -> crate::pac::timer::Timer;
    /// Enables the HF peripheral clock for this timer instance.
    fn enable_clock(&self);
}

impl Sealed for peripherals::Timer0 {}
impl TimerInstance for peripherals::Timer0 {
    fn regs(&self) -> crate::pac::timer::Timer {
        TIMER0
    }
    fn enable_clock(&self) {
        CMU.hfperclken0().modify(|w| w.set_timer0(true));
    }
}

impl Sealed for peripherals::Timer1 {}
impl TimerInstance for peripherals::Timer1 {
    fn regs(&self) -> crate::pac::timer::Timer {
        TIMER1
    }
    fn enable_clock(&self) {
        CMU.hfperclken0().modify(|w| w.set_timer1(true));
    }
}

/// Timer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<'d, T: TimerInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: TimerInstance> Timer<'d, T> {
    /// FIXME: take a (timer counter) frequency as parameter and do a best effort to set the timer prescaler and the
    ///        `top` value to get as close as possible
    pub fn new(peri: Peri<'d, T>, config: TimerConfig) -> Self {
        // Enable the timer peripheral clock for this instance.
        peri.enable_clock();
        let p = peri.regs();

        p.cmd().write(|w| w.set_stop(true));
        let instance = Self { peri };

        p.ctrl().write(|w| {
            w.set_mode(config.mode);
            w.set_clksel(config.clock);
            w.set_presc(config.presc);
        });
        p.cnt().write(|w| w.set_cnt(config.count));
        p.top().write(|w| w.set_top(config.top));
        p.top().write(|w| w.set_top(config.top));

        for (i, ch_config) in config.channels.configs.iter().enumerate() {
            let id = ChannelId::from_u8_unchecked(i as u8);
            let ch = instance.ch(id);
            ch.ctrl().write(|w| {
                w.set_mode(ch_config.mode);
                match ch_config.input_sel {
                    CcInputSel::Pin(loc) => {
                        w.set_insel(false);
                        instance.set_ch_loc(id, loc);
                    }
                    CcInputSel::Prs(prssel) => {
                        w.set_insel(true);
                        w.set_prssel(prssel);
                    }
                }
                w.set_icedge(ch_config.ic_edge_select);
                w.set_icevctrl(ch_config.ic_event_control);
            });
        }

        instance
    }

    /// Split the timer into channels which may be specialised for various uses (delay, pwm, etc.)
    pub fn into_channels(
        self,
    ) -> (
        TimerChannel<'d, T, 0>,
        TimerChannel<'d, T, 1>,
        TimerChannel<'d, T, 2>,
        TimerChannel<'d, T, 3>,
    ) {
        let p = self.peri.regs();

        // Enable timer
        p.cmd().write(|w| w.set_start(true));

        // Split the peripheral into its channels. Each channel drives a distinct
        // capture/compare channel, and the `Timer` is consumed, so the original singleton
        // cannot be reused.
        // SAFETY: the clones are disjoint by channel number; the source `peri` is not
        // reused after this call.
        (
            TimerChannel {
                peri: unsafe { self.peri.clone_unchecked() },
            },
            TimerChannel {
                peri: unsafe { self.peri.clone_unchecked() },
            },
            TimerChannel {
                peri: unsafe { self.peri.clone_unchecked() },
            },
            TimerChannel {
                peri: unsafe { self.peri.clone_unchecked() },
            },
        )
    }

    fn ch(&self, id: ChannelId) -> crate::pac::timer::Channel {
        match id {
            ChannelId::Id0 => self.peri.regs().cc0(),
            ChannelId::Id1 => self.peri.regs().cc1(),
            ChannelId::Id2 => self.peri.regs().cc2(),
            ChannelId::Id3 => self.peri.regs().cc3(),
        }
    }

    fn set_ch_loc(&self, id: ChannelId, loc: CcLoc) {
        self.peri.regs().routeloc0().modify(|w| match id {
            ChannelId::Id0 => w.set_cc0loc(loc),
            ChannelId::Id1 => w.set_cc1loc(loc),
            ChannelId::Id2 => w.set_cc2loc(loc),
            ChannelId::Id3 => w.set_cc3loc(loc),
        });
    }
}

/// Timer driver config
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerConfig {
    /// Counting mode for the Timer (`MODE`)
    pub mode: Mode,
    /// Clock source for the timer (`CLKSEL`)
    pub clock: Source,
    /// Prescaling factor (`PRESC`)
    pub presc: Prescaler,
    /// Initial counter value (`CNT`)
    pub count: u16,
    /// Top value for the counter (`TOP`)
    ///
    /// Default value is `u16::MAX`, since it makes more sense than `0`
    pub top: u16,
    /// Capture/Compare configs for channels
    pub channels: ChannelCofigs,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            mode: Mode::Up,
            clock: Source::Preschfperclk,
            presc: Prescaler::Div1,
            count: Default::default(),
            top: u16::MAX,
            channels: Default::default(),
        }
    }
}

/// Timer Capture/Compare Channel ID
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelId {
    /// Timer Capture/Compare Channel 0
    #[default]
    Id0,
    /// Timer Capture/Compare Channel 1
    Id1,
    /// Timer Capture/Compare Channel 2
    Id2,
    /// Timer Capture/Compare Channel 3
    Id3,
}

impl ChannelId {
    /// Number of Timer CC channels
    pub const COUNT: usize = 4;

    pub(crate) const fn from_u8_unchecked(id: u8) -> Self {
        match id & 0b11 {
            0 => ChannelId::Id0,
            1 => ChannelId::Id1,
            2 => ChannelId::Id2,
            3 => ChannelId::Id3,
            _ => unreachable!(),
        }
    }
}

/// Timer Compare/Capture channel config
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CcConfig {
    /// channel mode
    pub mode: CcCtrlMode,
    /// channel input select
    pub input_sel: CcInputSel,
    /// Input Capture Edge Select
    pub ic_edge_select: CcCtrlIcedge,
    /// Input Capture Event Control
    pub ic_event_control: CcCtrlIcevctrl,
}

impl Default for CcConfig {
    fn default() -> Self {
        Self {
            mode: CcCtrlMode::Off,
            input_sel: Default::default(),
            ic_edge_select: CcCtrlIcedge::Rising,
            ic_event_control: CcCtrlIcevctrl::Everyedge,
        }
    }
}

/// Configurations for all Capture/Compare channels of a Timer
///
/// Implements [`Default`], and provides a method for initializing the configs for one specific channel.
///
/// # Example
///
/// Initialize configs with default values except for channel `2`:
///
/// ```no_run
///
/// ChannelCofigs::default()
///     .with_cc_config(
///         ChannelId::Id2,
///         CcConfig {
///             mode: CcCtrlMode::Pwm,
///             ..Default::default()
///         }
///     )
/// ```
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelCofigs {
    configs: [CcConfig; ChannelId::COUNT],
}

impl ChannelCofigs {
    /// Set config for the Capture/Compare channel with the given `id`
    pub fn with_cc_config(mut self, id: ChannelId, config: CcConfig) -> Self {
        self.configs[id as usize] = config;
        self
    }
}

/// Timer Capture/Compare channel input selection
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CcInputSel {
    /// TIMERnCCx pin is selected
    Pin(CcLoc),
    /// PRS input (selected by PRSSEL) is selected
    Prs(Prssel),
}

impl Default for CcInputSel {
    fn default() -> Self {
        Self::Pin(CcLoc::Loc0)
    }
}

/// Timer channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerChannel<'d, T: TimerInstance, const CN: u8> {
    peri: Peri<'d, T>,
}

impl<'d, T: TimerInstance, const CN: u8> TimerChannel<'d, T, CN> {
    /// Convert timer channel to a PWM
    pub fn into_pwm<PIN>(self, pin: PIN) -> TimerChannelPwm<'d, T, CN, PIN>
    where
        PIN: OutputPin + TimerPin<CN>,
    {
        let p = self.peri.regs();

        // FIXME: PWM - Set the resolution of the counter to MAX - 1 because if the timer is going to be split into
        // channels and any of them is used as PWM, we need to allow the PWM channel to set its compare value to TOP + 1
        // in order to achieve 100% duty cycle

        match CN {
            0 => {
                p.routeloc0().write(|w| w.set_cc0loc(pin.loc()));
                p.cc0().ctrl().write(|w| {
                    w.set_icedge(CcCtrlIcedge::Both);
                    w.set_cmoa(CcCtrlCmoa::Toggle);
                    w.set_mode(CcCtrlMode::Pwm)
                });
                p.routepen().modify(|w| w.set_cc0pen(true));
            }
            1 => {
                p.routeloc0().write(|w| w.set_cc1loc(pin.loc()));
                p.cc1().ctrl().write(|w| {
                    w.set_icedge(CcCtrlIcedge::Both);
                    w.set_cmoa(CcCtrlCmoa::Toggle);
                    w.set_mode(CcCtrlMode::Pwm)
                });
                p.routepen().modify(|w| w.set_cc1pen(true));
            }
            2 => {
                p.routeloc0().write(|w| w.set_cc2loc(pin.loc()));
                p.cc2().ctrl().write(|w| {
                    w.set_icedge(CcCtrlIcedge::Both);
                    w.set_cmoa(CcCtrlCmoa::Toggle);
                    w.set_mode(CcCtrlMode::Pwm)
                });
                p.routepen().modify(|w| w.set_cc2pen(true));
            }
            3 => {
                p.routeloc0().write(|w| w.set_cc3loc(pin.loc()));
                p.cc3().ctrl().write(|w| {
                    w.set_icedge(CcCtrlIcedge::Both);
                    w.set_cmoa(CcCtrlCmoa::Toggle);
                    w.set_mode(CcCtrlMode::Pwm)
                });
                p.routepen().modify(|w| w.set_cc3pen(true));
            }
            _ => unreachable!(),
        }

        TimerChannelPwm {
            peri: self.peri,
            _pwm_pin: PhantomData,
        }
    }

    /// Convert timer to a Delay
    pub fn into_delay(self, clocks: &Clocks) -> TimerChannelDelay<'d, T, CN> {
        let p = self.peri.regs();
        let timer_div: u8 = p.ctrl().read().presc().to_bits();
        let timer_freq = clocks.hf_per_clk() / (timer_div + 1) as u32;

        match CN {
            0 => p
                .cc0()
                .ctrl()
                .write(|w| w.set_mode(CcCtrlMode::Outputcompare)),
            1 => p
                .cc1()
                .ctrl()
                .write(|w| w.set_mode(CcCtrlMode::Outputcompare)),
            2 => p
                .cc2()
                .ctrl()
                .write(|w| w.set_mode(CcCtrlMode::Outputcompare)),
            3 => p
                .cc3()
                .ctrl()
                .write(|w| w.set_mode(CcCtrlMode::Outputcompare)),
            _ => unreachable!(),
        };

        TimerChannelDelay {
            peri: self.peri,
            timer_freq,
        }
    }
}

/// Specialize the timer channel to be used for delays
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerChannelDelay<'d, T: TimerInstance, const CN: u8> {
    peri: Peri<'d, T>,
    timer_freq: u32,
}

impl<'d, T: TimerInstance, const CN: u8> DelayNs for TimerChannelDelay<'d, T, CN> {
    fn delay_ns(&mut self, ns: u32) {
        let microsecs = ns / 1000;

        // FIXME: converting ns to us is just a band-aid in order to avoid the delay duration being smaller than this
        //        code can handle. Worst case scenario is if the timer frequency is the same as the core frequency,
        //        in which case wanting to wait for a few nanoseconds may take longer than expected because the code
        //        below needs to calculate a Compare value which may have already elapsed by the time it's written to
        //        the compare field of the CC channel.
        //        A better accuracy may be obtained if we implement `DelayNs` for `Timer` instead of `TimerChannelDelay`
        //        since we can control when the timer starts.
        if microsecs > 0 {
            let p = self.peri.regs();
            let ticks_left = self.timer_freq as u64 * microsecs as u64 / 1_000_000_u64;
            let reload_max = p.top().read().top() as u32;
            let reference_count = p.cnt().read().cnt() as u32;

            let mut ticks_left = ticks_left as u32;
            let mut reload = ticks_left.min(reload_max);
            let mut compare = (reference_count + reload) % reload_max;

            while ticks_left > 0 {
                match CN {
                    0 => {
                        // clear interrupt flag
                        p.ifc().write(|w| w.set_cc0(true));

                        // set compare
                        p.cc0().ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        p.ien().write(|w| w.set_cc0(true));
                    }
                    1 => {
                        // clear interrupt flag
                        p.ifc().write(|w| w.set_cc1(true));

                        // set compare
                        p.cc1().ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        p.ien().write(|w| w.set_cc1(true));
                    }
                    2 => {
                        // clear interrupt flag
                        p.ifc().write(|w| w.set_cc2(true));

                        // set compare
                        p.cc2().ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        p.ien().write(|w| w.set_cc2(true));
                    }
                    3 => {
                        // clear interrupt flag
                        p.ifc().write(|w| w.set_cc3(true));

                        // set compare
                        p.cc3().ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        p.ien().write(|w| w.set_cc3(true));
                    }
                    _ => unreachable!(),
                }

                // calculate next loop's values _before_ waiting so that the jitter between loops is minimal
                ticks_left -= reload;
                reload = ticks_left.min(reload_max);
                compare = (reference_count + reload) % reload_max;

                match CN {
                    0 => while !p.if_().read().cc0() {},
                    1 => while !p.if_().read().cc1() {},
                    2 => while !p.if_().read().cc2() {},
                    3 => while !p.if_().read().cc3() {},
                    _ => unreachable!(),
                }
            }
        }
    }
}

/// PWM
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerChannelPwm<'d, T: TimerInstance, const CN: u8, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    peri: Peri<'d, T>,
    _pwm_pin: PhantomData<PIN>,
}

impl<'d, T: TimerInstance, const CN: u8, PIN> SetDutyCycle for TimerChannelPwm<'d, T, CN, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    fn max_duty_cycle(&self) -> u16 {
        // A 100% duty cycle is obtained by setting the channel Capture/Compare value to `top + 1`
        self.peri.regs().top().read().top().saturating_add(1)
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let p = self.peri.regs();

        match CN {
            0 => p.cc0().ccvb().write(|w| w.set_ccvb(duty)),
            1 => p.cc1().ccvb().write(|w| w.set_ccvb(duty)),
            2 => p.cc2().ccvb().write(|w| w.set_ccvb(duty)),
            3 => p.cc3().ccvb().write(|w| w.set_ccvb(duty)),
            _ => unreachable!(),
        };

        Ok(())
    }
}

impl<'d, T: TimerInstance, const CN: u8, PIN> ErrorType for TimerChannelPwm<'d, T, CN, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    type Error = Infallible;
}

/// Trait to specify the location values for TIMERn_ROUTELOC0 and TIMERn_ROUTELOC1 for pins which can be used as PWM
pub trait TimerPin<const CN: u8> {
    /// TIMERn_ROUTELOC0 and TIMERn_ROUTELOC1 values for each pin which implements this trait
    fn loc(&self) -> CcLoc;
}

/// Implement pin location trait for each of the timer channels and their sets of 32 pins
///
/// (timer_channel, loc, port, pin)
macro_rules! impl_timer_channel_loc {
    ($channel:literal, $loc:expr, $port:literal, $pin:literal) => {
        impl<ANY> TimerPin<$channel> for Pin<$port, $pin, ANY> {
            fn loc(&self) -> CcLoc {
                $loc
            }
        }
    };
}

impl_timer_channel_loc!(0, CcLoc::Loc0, 'A', 0);
impl_timer_channel_loc!(0, CcLoc::Loc1, 'A', 1);
impl_timer_channel_loc!(0, CcLoc::Loc2, 'A', 2);
impl_timer_channel_loc!(0, CcLoc::Loc3, 'A', 3);
impl_timer_channel_loc!(0, CcLoc::Loc4, 'A', 4);
impl_timer_channel_loc!(0, CcLoc::Loc5, 'A', 5);
impl_timer_channel_loc!(0, CcLoc::Loc6, 'B', 11);
impl_timer_channel_loc!(0, CcLoc::Loc7, 'B', 12);
impl_timer_channel_loc!(0, CcLoc::Loc8, 'B', 13);
impl_timer_channel_loc!(0, CcLoc::Loc9, 'B', 14);
impl_timer_channel_loc!(0, CcLoc::Loc10, 'B', 15);
impl_timer_channel_loc!(0, CcLoc::Loc11, 'C', 6);
impl_timer_channel_loc!(0, CcLoc::Loc12, 'C', 7);
impl_timer_channel_loc!(0, CcLoc::Loc13, 'C', 8);
impl_timer_channel_loc!(0, CcLoc::Loc14, 'C', 9);
impl_timer_channel_loc!(0, CcLoc::Loc15, 'C', 10);
impl_timer_channel_loc!(0, CcLoc::Loc16, 'C', 11);
impl_timer_channel_loc!(0, CcLoc::Loc17, 'D', 9);
impl_timer_channel_loc!(0, CcLoc::Loc18, 'D', 10);
impl_timer_channel_loc!(0, CcLoc::Loc19, 'D', 11);
impl_timer_channel_loc!(0, CcLoc::Loc20, 'D', 12);
impl_timer_channel_loc!(0, CcLoc::Loc21, 'D', 13);
impl_timer_channel_loc!(0, CcLoc::Loc22, 'D', 14);
impl_timer_channel_loc!(0, CcLoc::Loc23, 'D', 15);
impl_timer_channel_loc!(0, CcLoc::Loc24, 'F', 0);
impl_timer_channel_loc!(0, CcLoc::Loc25, 'F', 1);
impl_timer_channel_loc!(0, CcLoc::Loc26, 'F', 2);
impl_timer_channel_loc!(0, CcLoc::Loc27, 'F', 3);
impl_timer_channel_loc!(0, CcLoc::Loc28, 'F', 4);
impl_timer_channel_loc!(0, CcLoc::Loc29, 'F', 5);
impl_timer_channel_loc!(0, CcLoc::Loc30, 'F', 6);
impl_timer_channel_loc!(0, CcLoc::Loc31, 'F', 7);

impl_timer_channel_loc!(1, CcLoc::Loc0, 'A', 1);
impl_timer_channel_loc!(1, CcLoc::Loc1, 'A', 2);
impl_timer_channel_loc!(1, CcLoc::Loc2, 'A', 3);
impl_timer_channel_loc!(1, CcLoc::Loc3, 'A', 4);
impl_timer_channel_loc!(1, CcLoc::Loc4, 'A', 5);
impl_timer_channel_loc!(1, CcLoc::Loc5, 'B', 11);
impl_timer_channel_loc!(1, CcLoc::Loc6, 'B', 12);
impl_timer_channel_loc!(1, CcLoc::Loc7, 'B', 13);
impl_timer_channel_loc!(1, CcLoc::Loc8, 'B', 14);
impl_timer_channel_loc!(1, CcLoc::Loc9, 'B', 15);
impl_timer_channel_loc!(1, CcLoc::Loc10, 'C', 6);
impl_timer_channel_loc!(1, CcLoc::Loc11, 'C', 7);
impl_timer_channel_loc!(1, CcLoc::Loc12, 'C', 8);
impl_timer_channel_loc!(1, CcLoc::Loc13, 'C', 9);
impl_timer_channel_loc!(1, CcLoc::Loc14, 'C', 10);
impl_timer_channel_loc!(1, CcLoc::Loc15, 'C', 11);
impl_timer_channel_loc!(1, CcLoc::Loc16, 'D', 9);
impl_timer_channel_loc!(1, CcLoc::Loc17, 'D', 10);
impl_timer_channel_loc!(1, CcLoc::Loc18, 'D', 11);
impl_timer_channel_loc!(1, CcLoc::Loc19, 'D', 12);
impl_timer_channel_loc!(1, CcLoc::Loc20, 'D', 13);
impl_timer_channel_loc!(1, CcLoc::Loc21, 'D', 14);
impl_timer_channel_loc!(1, CcLoc::Loc22, 'D', 15);
impl_timer_channel_loc!(1, CcLoc::Loc23, 'F', 0);
impl_timer_channel_loc!(1, CcLoc::Loc24, 'F', 1);
impl_timer_channel_loc!(1, CcLoc::Loc25, 'F', 2);
impl_timer_channel_loc!(1, CcLoc::Loc26, 'F', 3);
impl_timer_channel_loc!(1, CcLoc::Loc27, 'F', 4);
impl_timer_channel_loc!(1, CcLoc::Loc28, 'F', 5);
impl_timer_channel_loc!(1, CcLoc::Loc29, 'F', 6);
impl_timer_channel_loc!(1, CcLoc::Loc30, 'F', 7);
impl_timer_channel_loc!(1, CcLoc::Loc31, 'A', 0);

impl_timer_channel_loc!(2, CcLoc::Loc0, 'A', 2);
impl_timer_channel_loc!(2, CcLoc::Loc1, 'A', 3);
impl_timer_channel_loc!(2, CcLoc::Loc2, 'A', 4);
impl_timer_channel_loc!(2, CcLoc::Loc3, 'A', 5);
impl_timer_channel_loc!(2, CcLoc::Loc4, 'B', 11);
impl_timer_channel_loc!(2, CcLoc::Loc5, 'B', 12);
impl_timer_channel_loc!(2, CcLoc::Loc6, 'B', 13);
impl_timer_channel_loc!(2, CcLoc::Loc7, 'B', 14);
impl_timer_channel_loc!(2, CcLoc::Loc8, 'B', 15);
impl_timer_channel_loc!(2, CcLoc::Loc9, 'C', 6);
impl_timer_channel_loc!(2, CcLoc::Loc10, 'C', 7);
impl_timer_channel_loc!(2, CcLoc::Loc11, 'C', 8);
impl_timer_channel_loc!(2, CcLoc::Loc12, 'C', 9);
impl_timer_channel_loc!(2, CcLoc::Loc13, 'C', 10);
impl_timer_channel_loc!(2, CcLoc::Loc14, 'C', 11);
impl_timer_channel_loc!(2, CcLoc::Loc15, 'D', 9);
impl_timer_channel_loc!(2, CcLoc::Loc16, 'D', 10);
impl_timer_channel_loc!(2, CcLoc::Loc17, 'D', 11);
impl_timer_channel_loc!(2, CcLoc::Loc18, 'D', 12);
impl_timer_channel_loc!(2, CcLoc::Loc19, 'D', 13);
impl_timer_channel_loc!(2, CcLoc::Loc20, 'D', 14);
impl_timer_channel_loc!(2, CcLoc::Loc21, 'D', 15);
impl_timer_channel_loc!(2, CcLoc::Loc22, 'F', 0);
impl_timer_channel_loc!(2, CcLoc::Loc23, 'F', 1);
impl_timer_channel_loc!(2, CcLoc::Loc24, 'F', 2);
impl_timer_channel_loc!(2, CcLoc::Loc25, 'F', 3);
impl_timer_channel_loc!(2, CcLoc::Loc26, 'F', 4);
impl_timer_channel_loc!(2, CcLoc::Loc27, 'F', 5);
impl_timer_channel_loc!(2, CcLoc::Loc28, 'F', 6);
impl_timer_channel_loc!(2, CcLoc::Loc29, 'F', 7);
impl_timer_channel_loc!(2, CcLoc::Loc30, 'A', 0);
impl_timer_channel_loc!(2, CcLoc::Loc31, 'A', 1);

impl_timer_channel_loc!(3, CcLoc::Loc0, 'A', 3);
impl_timer_channel_loc!(3, CcLoc::Loc1, 'A', 4);
impl_timer_channel_loc!(3, CcLoc::Loc2, 'A', 5);
impl_timer_channel_loc!(3, CcLoc::Loc3, 'B', 11);
impl_timer_channel_loc!(3, CcLoc::Loc4, 'B', 12);
impl_timer_channel_loc!(3, CcLoc::Loc5, 'B', 13);
impl_timer_channel_loc!(3, CcLoc::Loc6, 'B', 14);
impl_timer_channel_loc!(3, CcLoc::Loc7, 'B', 15);
impl_timer_channel_loc!(3, CcLoc::Loc8, 'C', 6);
impl_timer_channel_loc!(3, CcLoc::Loc9, 'C', 7);
impl_timer_channel_loc!(3, CcLoc::Loc10, 'C', 8);
impl_timer_channel_loc!(3, CcLoc::Loc11, 'C', 9);
impl_timer_channel_loc!(3, CcLoc::Loc12, 'C', 10);
impl_timer_channel_loc!(3, CcLoc::Loc13, 'C', 11);
impl_timer_channel_loc!(3, CcLoc::Loc14, 'D', 9);
impl_timer_channel_loc!(3, CcLoc::Loc15, 'D', 10);
impl_timer_channel_loc!(3, CcLoc::Loc16, 'D', 11);
impl_timer_channel_loc!(3, CcLoc::Loc17, 'D', 12);
impl_timer_channel_loc!(3, CcLoc::Loc18, 'D', 13);
impl_timer_channel_loc!(3, CcLoc::Loc19, 'D', 14);
impl_timer_channel_loc!(3, CcLoc::Loc20, 'D', 15);
impl_timer_channel_loc!(3, CcLoc::Loc21, 'F', 0);
impl_timer_channel_loc!(3, CcLoc::Loc22, 'F', 1);
impl_timer_channel_loc!(3, CcLoc::Loc23, 'F', 2);
impl_timer_channel_loc!(3, CcLoc::Loc24, 'F', 3);
impl_timer_channel_loc!(3, CcLoc::Loc25, 'F', 4);
impl_timer_channel_loc!(3, CcLoc::Loc26, 'F', 5);
impl_timer_channel_loc!(3, CcLoc::Loc27, 'F', 6);
impl_timer_channel_loc!(3, CcLoc::Loc28, 'F', 7);
impl_timer_channel_loc!(3, CcLoc::Loc29, 'A', 0);
impl_timer_channel_loc!(3, CcLoc::Loc30, 'A', 1);
impl_timer_channel_loc!(3, CcLoc::Loc31, 'A', 2);
