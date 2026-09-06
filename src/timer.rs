//! Timer/Counter
//!

pub use crate::pac::timer::vals::Presc as TimerDivider;
use crate::{
    cmu::Clocks,
    gpio::pin::Pin,
    pac::{
        timer::vals::{
            Cc0CtrlCmoa, Cc0CtrlIcedge, Cc0CtrlMode, Cc0loc, Cc1loc, Cc2loc, Cc3loc, CtrlMode,
        },
        CMU, TIMER0, TIMER1,
    },
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
    fn regs() -> crate::pac::timer::Timer;
    /// Enables the HF peripheral clock for this timer instance.
    fn enable_clock();
}

impl Sealed for peripherals::Timer0 {}
impl TimerInstance for peripherals::Timer0 {
    fn regs() -> crate::pac::timer::Timer {
        TIMER0
    }
    fn enable_clock() {
        CMU.hfperclken0().modify(|w| w.set_timer0(true));
    }
}

impl Sealed for peripherals::Timer1 {}
impl TimerInstance for peripherals::Timer1 {
    fn regs() -> crate::pac::timer::Timer {
        TIMER1
    }
    fn enable_clock() {
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
    pub fn new(peri: Peri<'d, T>, clock_divider: TimerDivider) -> Self {
        let timer = T::regs();

        timer.ctrl().write(|w| {
            w.set_presc(clock_divider);
            w.set_mode(CtrlMode::Up);
        });

        // Set the resolution of the counter to MAX - 1 because if the timer is going to be split into channels and
        // any of them is used as PWM, we need to allow the PWM channel to set its compare value to TOP + 1 in order
        // to achieve 100% duty cycle
        timer.top().write(|w| w.set_top(u16::MAX - 1));

        Self {
            peri: peri,
        }
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
        // Enable the timer peripheral clock for this instance.
        T::enable_clock();

        // Enable timer
        T::regs().cmd().write(|w| w.set_start(true));

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
        let timer = T::regs();

        match CN {
            0 => {
                timer
                    .routeloc0()
                    .write(|w| w.set_cc0loc(Cc0loc::from_bits(pin.loc())));
                timer.cc0_ctrl().write(|w| {
                    w.set_icedge(Cc0CtrlIcedge::Both);
                    w.set_cmoa(Cc0CtrlCmoa::Toggle);
                    w.set_mode(Cc0CtrlMode::Pwm)
                });
                timer.routepen().modify(|w| w.set_cc0pen(true));
            }
            1 => {
                timer
                    .routeloc0()
                    .write(|w| w.set_cc1loc(Cc1loc::from_bits(pin.loc())));
                timer.cc1_ctrl().write(|w| {
                    w.set_icedge(Cc0CtrlIcedge::Both);
                    w.set_cmoa(Cc0CtrlCmoa::Toggle);
                    w.set_mode(Cc0CtrlMode::Pwm)
                });
                timer.routepen().modify(|w| w.set_cc1pen(true));
            }
            2 => {
                timer
                    .routeloc0()
                    .write(|w| w.set_cc2loc(Cc2loc::from_bits(pin.loc())));
                timer.cc2_ctrl().write(|w| {
                    w.set_icedge(Cc0CtrlIcedge::Both);
                    w.set_cmoa(Cc0CtrlCmoa::Toggle);
                    w.set_mode(Cc0CtrlMode::Pwm)
                });
                timer.routepen().modify(|w| w.set_cc2pen(true));
            }
            3 => {
                timer
                    .routeloc0()
                    .write(|w| w.set_cc3loc(Cc3loc::from_bits(pin.loc())));
                timer.cc3_ctrl().write(|w| {
                    w.set_icedge(Cc0CtrlIcedge::Both);
                    w.set_cmoa(Cc0CtrlCmoa::Toggle);
                    w.set_mode(Cc0CtrlMode::Pwm)
                });
                timer.routepen().modify(|w| w.set_cc3pen(true));
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
        let timer = T::regs();
        let timer_div: u8 = timer.ctrl().read().presc().to_bits();
        let timer_freq = clocks.hf_per_clk() / (timer_div + 1) as u32;

        match CN {
            0 => timer
                .cc0_ctrl()
                .write(|w| w.set_mode(Cc0CtrlMode::Outputcompare)),
            1 => timer
                .cc1_ctrl()
                .write(|w| w.set_mode(Cc0CtrlMode::Outputcompare)),
            2 => timer
                .cc2_ctrl()
                .write(|w| w.set_mode(Cc0CtrlMode::Outputcompare)),
            3 => timer
                .cc3_ctrl()
                .write(|w| w.set_mode(Cc0CtrlMode::Outputcompare)),
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
            let timer = T::regs();
            let ticks_left = self.timer_freq as u64 * microsecs as u64 / 1_000_000_u64;
            let reload_max = timer.top().read().top() as u32;
            let reference_count = timer.cnt().read().cnt() as u32;

            let mut ticks_left = ticks_left as u32;
            let mut reload = ticks_left.min(reload_max);
            let mut compare = (reference_count + reload) % reload_max;

            while ticks_left > 0 {
                match CN {
                    0 => {
                        // clear interrupt flag
                        timer.ifc().write(|w| w.set_cc0(true));

                        // set compare
                        timer.cc0_ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        timer.ien().write(|w| w.set_cc0(true));
                    }
                    1 => {
                        // clear interrupt flag
                        timer.ifc().write(|w| w.set_cc1(true));

                        // set compare
                        timer.cc1_ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        timer.ien().write(|w| w.set_cc1(true));
                    }
                    2 => {
                        // clear interrupt flag
                        timer.ifc().write(|w| w.set_cc2(true));

                        // set compare
                        timer.cc2_ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        timer.ien().write(|w| w.set_cc2(true));
                    }
                    3 => {
                        // clear interrupt flag
                        timer.ifc().write(|w| w.set_cc3(true));

                        // set compare
                        timer.cc3_ccv().write(|w| w.set_ccv(compare as u16));

                        // enable channel interrupt
                        timer.ien().write(|w| w.set_cc3(true));
                    }
                    _ => unreachable!(),
                }

                // calculate next loop's values _before_ waiting so that the jitter between loops is minimal
                ticks_left -= reload;
                reload = ticks_left.min(reload_max);
                compare = (reference_count + reload) % reload_max;

                match CN {
                    0 => while !timer.if_().read().cc0() {},
                    1 => while !timer.if_().read().cc1() {},
                    2 => while !timer.if_().read().cc2() {},
                    3 => while !timer.if_().read().cc3() {},
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
        T::regs().top().read().top().saturating_add(1)
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let timer = T::regs();

        match CN {
            0 => timer.cc0_ccvb().write(|w| w.set_ccvb(duty)),
            1 => timer.cc1_ccvb().write(|w| w.set_ccvb(duty)),
            2 => timer.cc2_ccvb().write(|w| w.set_ccvb(duty)),
            3 => timer.cc3_ccvb().write(|w| w.set_ccvb(duty)),
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
    fn loc(&self) -> u8;
}

/// Implement pin location trait for each of the timer channels and their sets of 32 pins
///
/// (timer_channel, loc, port, pin)
macro_rules! impl_timer_channel_loc {
    ($channel:literal, $loc:literal, $port:literal, $pin:literal) => {
        impl<ANY> TimerPin<$channel> for Pin<$port, $pin, ANY> {
            fn loc(&self) -> u8 {
                $loc
            }
        }
    };
}

impl_timer_channel_loc!(0, 0, 'A', 0);
impl_timer_channel_loc!(0, 1, 'A', 1);
impl_timer_channel_loc!(0, 2, 'A', 2);
impl_timer_channel_loc!(0, 3, 'A', 3);
impl_timer_channel_loc!(0, 4, 'A', 4);
impl_timer_channel_loc!(0, 5, 'A', 5);
impl_timer_channel_loc!(0, 6, 'B', 11);
impl_timer_channel_loc!(0, 7, 'B', 12);
impl_timer_channel_loc!(0, 8, 'B', 13);
impl_timer_channel_loc!(0, 9, 'B', 14);
impl_timer_channel_loc!(0, 10, 'B', 15);
impl_timer_channel_loc!(0, 11, 'C', 6);
impl_timer_channel_loc!(0, 12, 'C', 7);
impl_timer_channel_loc!(0, 13, 'C', 8);
impl_timer_channel_loc!(0, 14, 'C', 9);
impl_timer_channel_loc!(0, 15, 'C', 10);
impl_timer_channel_loc!(0, 16, 'C', 11);
impl_timer_channel_loc!(0, 17, 'D', 9);
impl_timer_channel_loc!(0, 18, 'D', 10);
impl_timer_channel_loc!(0, 19, 'D', 11);
impl_timer_channel_loc!(0, 20, 'D', 12);
impl_timer_channel_loc!(0, 21, 'D', 13);
impl_timer_channel_loc!(0, 22, 'D', 14);
impl_timer_channel_loc!(0, 23, 'D', 15);
impl_timer_channel_loc!(0, 24, 'F', 0);
impl_timer_channel_loc!(0, 25, 'F', 1);
impl_timer_channel_loc!(0, 26, 'F', 2);
impl_timer_channel_loc!(0, 27, 'F', 3);
impl_timer_channel_loc!(0, 28, 'F', 4);
impl_timer_channel_loc!(0, 29, 'F', 5);
impl_timer_channel_loc!(0, 30, 'F', 6);
impl_timer_channel_loc!(0, 31, 'F', 7);

impl_timer_channel_loc!(1, 0, 'A', 1);
impl_timer_channel_loc!(1, 1, 'A', 2);
impl_timer_channel_loc!(1, 2, 'A', 3);
impl_timer_channel_loc!(1, 3, 'A', 4);
impl_timer_channel_loc!(1, 4, 'A', 5);
impl_timer_channel_loc!(1, 5, 'B', 11);
impl_timer_channel_loc!(1, 6, 'B', 12);
impl_timer_channel_loc!(1, 7, 'B', 13);
impl_timer_channel_loc!(1, 8, 'B', 14);
impl_timer_channel_loc!(1, 9, 'B', 15);
impl_timer_channel_loc!(1, 10, 'C', 6);
impl_timer_channel_loc!(1, 11, 'C', 7);
impl_timer_channel_loc!(1, 12, 'C', 8);
impl_timer_channel_loc!(1, 13, 'C', 9);
impl_timer_channel_loc!(1, 14, 'C', 10);
impl_timer_channel_loc!(1, 15, 'C', 11);
impl_timer_channel_loc!(1, 16, 'D', 9);
impl_timer_channel_loc!(1, 17, 'D', 10);
impl_timer_channel_loc!(1, 18, 'D', 11);
impl_timer_channel_loc!(1, 19, 'D', 12);
impl_timer_channel_loc!(1, 20, 'D', 13);
impl_timer_channel_loc!(1, 21, 'D', 14);
impl_timer_channel_loc!(1, 22, 'D', 15);
impl_timer_channel_loc!(1, 23, 'F', 0);
impl_timer_channel_loc!(1, 24, 'F', 1);
impl_timer_channel_loc!(1, 25, 'F', 2);
impl_timer_channel_loc!(1, 26, 'F', 3);
impl_timer_channel_loc!(1, 27, 'F', 4);
impl_timer_channel_loc!(1, 28, 'F', 5);
impl_timer_channel_loc!(1, 29, 'F', 6);
impl_timer_channel_loc!(1, 30, 'F', 7);
impl_timer_channel_loc!(1, 31, 'A', 0);

impl_timer_channel_loc!(2, 0, 'A', 2);
impl_timer_channel_loc!(2, 1, 'A', 3);
impl_timer_channel_loc!(2, 2, 'A', 4);
impl_timer_channel_loc!(2, 3, 'A', 5);
impl_timer_channel_loc!(2, 4, 'B', 11);
impl_timer_channel_loc!(2, 5, 'B', 12);
impl_timer_channel_loc!(2, 6, 'B', 13);
impl_timer_channel_loc!(2, 7, 'B', 14);
impl_timer_channel_loc!(2, 8, 'B', 15);
impl_timer_channel_loc!(2, 9, 'C', 6);
impl_timer_channel_loc!(2, 10, 'C', 7);
impl_timer_channel_loc!(2, 11, 'C', 8);
impl_timer_channel_loc!(2, 12, 'C', 9);
impl_timer_channel_loc!(2, 13, 'C', 10);
impl_timer_channel_loc!(2, 14, 'C', 11);
impl_timer_channel_loc!(2, 15, 'D', 9);
impl_timer_channel_loc!(2, 16, 'D', 10);
impl_timer_channel_loc!(2, 17, 'D', 11);
impl_timer_channel_loc!(2, 18, 'D', 12);
impl_timer_channel_loc!(2, 19, 'D', 13);
impl_timer_channel_loc!(2, 20, 'D', 14);
impl_timer_channel_loc!(2, 21, 'D', 15);
impl_timer_channel_loc!(2, 22, 'F', 0);
impl_timer_channel_loc!(2, 23, 'F', 1);
impl_timer_channel_loc!(2, 24, 'F', 2);
impl_timer_channel_loc!(2, 25, 'F', 3);
impl_timer_channel_loc!(2, 26, 'F', 4);
impl_timer_channel_loc!(2, 27, 'F', 5);
impl_timer_channel_loc!(2, 28, 'F', 6);
impl_timer_channel_loc!(2, 29, 'F', 7);
impl_timer_channel_loc!(2, 30, 'A', 0);
impl_timer_channel_loc!(2, 31, 'A', 1);

impl_timer_channel_loc!(3, 0, 'A', 3);
impl_timer_channel_loc!(3, 1, 'A', 4);
impl_timer_channel_loc!(3, 2, 'A', 5);
impl_timer_channel_loc!(3, 3, 'B', 11);
impl_timer_channel_loc!(3, 4, 'B', 12);
impl_timer_channel_loc!(3, 5, 'B', 13);
impl_timer_channel_loc!(3, 6, 'B', 14);
impl_timer_channel_loc!(3, 7, 'B', 15);
impl_timer_channel_loc!(3, 8, 'C', 6);
impl_timer_channel_loc!(3, 9, 'C', 7);
impl_timer_channel_loc!(3, 10, 'C', 8);
impl_timer_channel_loc!(3, 11, 'C', 9);
impl_timer_channel_loc!(3, 12, 'C', 10);
impl_timer_channel_loc!(3, 13, 'C', 11);
impl_timer_channel_loc!(3, 14, 'D', 9);
impl_timer_channel_loc!(3, 15, 'D', 10);
impl_timer_channel_loc!(3, 16, 'D', 11);
impl_timer_channel_loc!(3, 17, 'D', 12);
impl_timer_channel_loc!(3, 18, 'D', 13);
impl_timer_channel_loc!(3, 19, 'D', 14);
impl_timer_channel_loc!(3, 20, 'D', 15);
impl_timer_channel_loc!(3, 21, 'F', 0);
impl_timer_channel_loc!(3, 22, 'F', 1);
impl_timer_channel_loc!(3, 23, 'F', 2);
impl_timer_channel_loc!(3, 24, 'F', 3);
impl_timer_channel_loc!(3, 25, 'F', 4);
impl_timer_channel_loc!(3, 26, 'F', 5);
impl_timer_channel_loc!(3, 27, 'F', 6);
impl_timer_channel_loc!(3, 28, 'F', 7);
impl_timer_channel_loc!(3, 29, 'A', 0);
impl_timer_channel_loc!(3, 30, 'A', 1);
impl_timer_channel_loc!(3, 31, 'A', 2);
