#![no_std]
#![doc = include_str!("../README.md")]

use num_traits::float::FloatCore;

/// Dead simple low pass filter implementation, used internally for
/// optionally preventing derivative kick but can be used by the end user
/// as well.
pub struct LowPassFilter<T: FloatCore> {
    accumulator: T,
    time_s: T,
}

impl<T> LowPassFilter<T>
where T: FloatCore {
    /// Create a low pass filter that will average inputs over a given duration.
    pub fn new(time_s: T, initial: T) -> Self {
        Self {
            time_s,
            accumulator: initial,
        }
    }

    /// Filter out the input given the time since the last filter call,
    /// and return the current computed low pass value.
    pub fn update(&mut self, input: T, dt_s: T) -> T {
        let time_ratio = dt_s / (dt_s + self.time_s);

        let multiplier_x = time_ratio;
        let multiplier_y = T::one() - time_ratio;

        self.accumulator = multiplier_x * input + multiplier_y * self.accumulator;

        self.accumulator
    }
}

/// Standard PID struct implementation.
///
/// ### Example:
/// 
/// ```rust
/// let mut temp_c: f32 = 25.0; // Pretend this is the input of a thermocouple
///
/// let mut ssr: bool = false; // And this is the output to a solid state relay
///
/// let mut pid: StandardPID = StandardPID::new(
///     1.4, // Kp
///     10.2, // Ti
///     20.8, // Td
///     100.0, // Setpoint of 100c
///     0.0, // Minimum output value
///     1.0 // Maximum output value
/// )
///     .d_low_pass(1.0) // Set the low pass filter to average over 1 second
///     .i_band(10.0); // Disable the integral when 10c outside the target temperature
///
/// loop {
///     // Insert misc. logic here..
///
///     // Turn on the SSR if the output is greater than 0.5
///     // Delta time is hard coded in this example
///     ssr = pid.update(temp_c, 0.1) > 0.5;
/// }
///
/// ```
pub struct StandardPID<T: FloatCore> {
    kp: T,
    one_over_ti_s: T,
    td_s: T,
    i_band: T,
    p: T,
    i: T,
    d: T,
    error: T,
    sp: T,
    out_range_min: T,
    out_range_max: T,
    d_low_pass: Option<LowPassFilter<T>>
}

impl<T> StandardPID<T>
where T: FloatCore {
    /// Create a new PID controller given the Kp, Ti, Td, Sp, and the out range.
    pub fn new(kp: T, ti_s: T, td_s: T, sp: T, out_range_min: T, out_range_max: T) -> Self {
        let ti_s = ti_s.max(T::min_positive_value());
        let td_s = td_s.max(T::zero());

        Self {
            kp,
            one_over_ti_s: T::one() / ti_s,
            td_s,
            i_band: T::max_value(),
            p: T::zero(),
            i: T::zero(),
            d: T::zero(),
            error: T::zero(),
            sp,
            out_range_min,
            out_range_max,
            d_low_pass: None,
        }
    }

    /// Update the PID values given a process varable and the time since the
    /// last update, and in exchange return the calculated output value.
    pub fn update(&mut self, pv: T, dt_s: T) -> T {
        let old_error = self.error;
        self.error = self.sp - pv;
        let delta_error = self.error - old_error;

        let delta_error = if let Some(mut lp_filter) = self.d_low_pass.take() {
            let val = lp_filter.update(delta_error, dt_s);

            self.d_low_pass = Some(lp_filter);

            val
        } else {
            delta_error
        };

        self.i = match self.error.abs() > self.i_band {
            false => self.i + self.one_over_ti_s * self.error * dt_s,
            true => T::zero(),
        };

        self.d = self.td_s * (delta_error / dt_s);

        self.p = self.kp * (self.error + self.i + self.d);

        let out = self.out_range_min.max(self.out_range_max.min(self.p));

        out
    }

    
    #[inline]
    /// Consuming variant of `set_d_low_pass`.
    pub fn d_low_pass(mut self, time_s: T) -> Self {
        self.set_d_low_pass(time_s);

        self
    }

    #[inline]
    /// Consuming variant of `set_i_band`.
    pub fn i_band(mut self, i_band: T) -> Self {
        self.set_i_band(i_band);

        self
    }

    #[inline]
    /// Reset the integral term to zero.
    pub fn reset_integral_term(&mut self) {
        self.i = T::zero();
    }
    
    #[inline]
    /// Enable the low pass filter on the derivative term and set the averaging time.
    pub fn set_d_low_pass(&mut self, time_s: T) {
        self.d_low_pass = Some(LowPassFilter::new(time_s, self.sp));
    }

    #[inline]
    /// Set the proportional gain.
    pub fn set_kp(&mut self, kp: T) {
        self.kp = kp;
    }
    
    #[inline]
    /// Set the integral time. **note: cannot less than or equal to zero; inputs are sanitized to
    /// prevent panics and unexpected behaviour**.
    pub fn set_ti_s(&mut self, ti_s: T) {
        let ti_s = ti_s.max(T::min_positive_value());
        self.one_over_ti_s = T::one() / ti_s;
    }

    #[inline]
    /// Set the derivative time. **note: cannot be less than zero; inputs are sanitized to prevent
    /// unexpected behaviour**.
    pub fn set_td_s(&mut self, td_s: T) {
        let td_s = td_s.max(T::zero());
        self.td_s = td_s;
    }

    #[inline]
    /// Enable and set the I band. If the absolute error between the process
    /// variable and the set point is greater than the I band, the integral
    /// term will be set to zero.
    pub fn set_i_band(&mut self, i_band: T) {
        let i_band = i_band.max(T::zero());
        self.i_band = i_band;
    }

    #[inline]
    /// Set the set point.
    pub fn set_sp(&mut self, sp: T) {
        self.sp = sp;
    }

    #[inline]
    /// Get the proportional gain.
    pub fn get_kp(&mut self) -> T {
        self.kp
    }

    #[inline]
    /// Get the integral time.
    pub fn get_ti_s(&mut self) -> T {
        T::one() / self.one_over_ti_s
    }

    #[inline]
    /// Get the derivative time.
    pub fn get_td_s(&mut self) -> T {
        self.td_s
    }

    #[inline]
    /// Get the set point.
    pub fn get_sp(&mut self) -> T {
        self.sp
    }
}

