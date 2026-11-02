use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use embedded_hal::delay::DelayNs;

use crate::clocks::Clocks;

/// SysTick-based delay provider.
///
/// Implements [`DelayNs`] using the ARM Cortex-M SysTick timer.
/// The SysTick reload register is 24 bits wide, limiting single
/// delays to ~233 ms at 72 MHz. Longer delays are handled by
/// looping.
pub struct Delay {
    syst: SYST,
    cycles_per_us: u32,
}

impl Delay {
    /// Create a new `Delay` from the SysTick peripheral and frozen clocks.
    pub fn new(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        let cycles_per_us = clocks.core_clk().to_raw() / 1_000_000;
        Delay { syst, cycles_per_us }
    }

    /// Release the SysTick peripheral.
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // Convert ns to cycles, rounding up: ceil(ns * c / 1000).
        //
        // In 32-bit math, not u64: on this core a u64 divide is a call to
        // `__aeabi_uldivmod`, which links about 930 B of compiler_builtins
        // into a firmware that has no other 64-bit divide, and is slow (the
        // zero-delay case in mk20dx-testsuite has a 100-tick budget). With
        // ns = q * 1000 + r the value is q * c + ceil(r * c / 1000). With
        // c <= 120 (a 120 MHz core) both terms fit a u32, and a divide by the
        // constant 1000 is a multiply.
        #[allow(clippy::manual_div_ceil)]
        let mut remaining = {
            let c = self.cycles_per_us;
            (ns / 1000) * c + ((ns % 1000) * c + 999) / 1000
        };

        // SysTick reload is 24 bits (max 0x00FF_FFFF = 16_777_215)
        const MAX_RELOAD: u32 = 0x00FF_FFFF;

        while remaining > 0 {
            let reload = remaining.min(MAX_RELOAD);
            self.syst.set_reload(reload);
            self.syst.clear_current();
            self.syst.enable_counter();

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
            remaining -= reload;
        }
    }
}
