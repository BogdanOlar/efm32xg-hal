//! Timer IRQs
//!

use crate::{pac::interrupt, timer::TimerIrqHandler};
use core::cell::RefCell;
use critical_section::Mutex;

/// Handler which does nothing
pub(crate) const fn default_handler() {}

/// Interrupt handlers for timers
pub(crate) static TIMER0_HANDLER: Mutex<RefCell<TimerIrqHandler>> =
    Mutex::new(RefCell::new(default_handler));
pub(crate) static TIMER1_HANDLER: Mutex<RefCell<TimerIrqHandler>> =
    Mutex::new(RefCell::new(default_handler));

#[interrupt]
fn TIMER0() {
    critical_section::with(|cs| TIMER0_HANDLER.borrow(cs).borrow()());
}

#[interrupt]
fn TIMER1() {
    critical_section::with(|cs| TIMER1_HANDLER.borrow(cs).borrow()());
}
