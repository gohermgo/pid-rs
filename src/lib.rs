use std::{ops::Range, time::Duration};

pub trait Float
where
    Self: Copy
        + std::cmp::PartialOrd
        + std::ops::Add<Self, Output = Self>
        + std::ops::Sub<Self, Output = Self>
        + std::ops::Mul<Self, Output = Self>
        + std::ops::Div<Self, Output = Self>
        + Sized,
{
    fn negative() -> Self;
    fn double() -> Self;
    fn one() -> Self;
    fn half() -> Self;
    fn zero() -> Self;
    fn from_duration(dur: &Duration) -> Self;
}
impl Float for f32 {
    fn negative() -> Self {
        -1.
    }
    fn double() -> Self {
        2.0
    }
    fn half() -> Self {
        0.5
    }
    fn one() -> Self {
        1.0
    }
    fn zero() -> Self {
        0.
    }
    fn from_duration(dur: &Duration) -> Self {
        dur.as_secs_f32()
    }
}
impl Float for f64 {
    fn negative() -> Self {
        -1.
    }
    fn double() -> Self {
        2.0
    }
    fn half() -> Self {
        0.5
    }
    fn one() -> Self {
        1.0
    }
    fn zero() -> Self {
        0.
    }
    fn from_duration(dur: &Duration) -> Self {
        dur.as_secs_f64()
    }
}

pub trait ControllerComponent<T: Float> {
    fn init(&mut self);
    fn update(&mut self, setpoint: T, measurement: T, sample_time: &Duration) -> T;
}

pub struct Proportional<T: Float> {
    gain: T,
}
impl<T: Float> Proportional<T> {
    pub fn new(gain: T) -> Self {
        Self { gain }
    }
}
impl<T: Float> ControllerComponent<T> for Proportional<T> {
    fn init(&mut self) {}
    fn update(&mut self, setpoint: T, measurement: T, _: &Duration) -> T {
        let error = setpoint - measurement;
        self.gain * error
    }
}

pub struct Integrator<T: Float> {
    value: T,
    gain: T,
    previous_error: T,
    output_limit: Range<T>,
}
impl<T: Float> Integrator<T> {
    pub fn new(gain: T, output_limit: Range<T>) -> Self {
        Self {
            value: T::zero(),
            gain,
            previous_error: T::zero(),
            output_limit,
        }
    }
    fn clamp_value(&mut self) {
        if self.value > self.output_limit.end {
            self.value = self.output_limit.end;
        } else if self.value < self.output_limit.start {
            self.value = self.output_limit.start;
        }
    }
}
impl<T: Float> ControllerComponent<T> for Integrator<T> {
    fn init(&mut self) {
        self.value = T::zero();
        self.previous_error = T::zero();
    }
    fn update(&mut self, setpoint: T, measurement: T, sample_time: &Duration) -> T {
        let error = setpoint - measurement;
        let new_value =
            T::half() * self.gain * T::from_duration(sample_time) * (error + self.previous_error);
        self.value = self.value + new_value;
        self.clamp_value();
        self.previous_error = error;
        self.value
    }
}

pub struct Differentiator<T: Float> {
    value: T,
    gain: T,
    time_constant: T,
    previous_measurement: T,
}
impl<T: Float> Differentiator<T> {
    pub fn new(gain: T, time_constant: T) -> Self {
        Self {
            value: T::zero(),
            gain,
            time_constant,
            previous_measurement: T::zero(),
        }
    }
}
impl<T: Float> ControllerComponent<T> for Differentiator<T> {
    fn init(&mut self) {
        self.value = T::zero();
        self.previous_measurement = T::zero();
    }
    fn update(&mut self, _: T, measurement: T, sample_time: &Duration) -> T {
        let measurement_error = measurement - self.previous_measurement;

        let numerator = T::negative()
            * (T::double() * self.gain * measurement_error
                + (T::double() * self.time_constant - T::from_duration(sample_time)) * self.value);

        let denominator = T::double() * self.time_constant + T::from_duration(sample_time);

        self.previous_measurement = measurement;

        self.value = numerator / denominator;

        self.value
    }
}

pub struct Controller<T: Float> {
    output_limit: Range<T>,
    sample_time: Duration,
    p: Proportional<T>,
    i: Integrator<T>,
    d: Differentiator<T>,
    out: T,
}
impl<T: Float> Controller<T> {
    pub fn new(
        output_limit: Range<T>,
        sample_time: Duration,
        p: Proportional<T>,
        i: Integrator<T>,
        d: Differentiator<T>,
    ) -> Self {
        Self {
            output_limit,
            sample_time,
            p,
            i,
            d,
            out: T::zero(),
        }
    }
    pub fn init(&mut self) {
        self.p.init();
        self.i.init();
        self.d.init();
        self.out = T::zero();
    }
    pub fn update(&mut self, setpoint: T, measurement: T) -> T {
        let p = self.p.update(setpoint, measurement, &self.sample_time);
        let i = self.i.update(setpoint, measurement, &self.sample_time);
        let d = self.d.update(setpoint, measurement, &self.sample_time);
        self.out = p + i + d;
        if self.out > self.output_limit.end {
            self.out = self.output_limit.end;
        } else if self.out < self.output_limit.start {
            self.out = self.output_limit.start;
        };
        self.out
    }
}
