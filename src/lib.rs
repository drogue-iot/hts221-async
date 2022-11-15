#![no_std]
#![feature(type_alias_impl_trait)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

use core::fmt::{LowerHex, UpperHex};
use core::{
    fmt::{Debug, Display, Formatter},
    marker::PhantomData,
    ops::{Add, Div, Sub},
};
use embedded_hal_async::i2c::*;
use register::who_am_i::WhoAmI;
use register::{
    calibration::*,
    ctrl1::{BlockDataUpdate, Ctrl1, OutputDataRate},
    ctrl2::Ctrl2,
    ctrl3::Ctrl3,
    h_out::Hout,
    status::Status,
    t_out::Tout,
};

mod register;

const ADDR: u8 = 0x5F;

/// Error returned by Hts221 driver
pub enum Hts221Error<E> {
    /// Error from I2C.
    I2c(E),
    /// Attempting to read before calibration.
    NotCalibrated,
    /// Not the expected sensor device
    InvalidSensor,
}

/// An instance of the HTS221 driver using I2C transport from embedded-hal-async.
pub struct Hts221<I>
where
    I: I2c<SevenBitAddress> + 'static,
    <I as ErrorType>::Error: Send,
{
    i2c: I,
    address: I2cAddress,
    calibration: Option<Calibration>,
}

impl<I> Hts221<I>
where
    I: I2c<SevenBitAddress> + 'static,
    <I as ErrorType>::Error: Send,
{
    /// Create a new instance of the driver using a given I2C peripheral.
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            address: I2cAddress(ADDR),
            calibration: None,
        }
    }

    /// Initialize the driver. Must be run before reading sensor values.
    pub async fn initialize(&mut self) -> Result<(), Hts221Error<I::Error>> {
        let addr = WhoAmI::read(self.address, &mut self.i2c).await?;
        if addr != self.address {
            return Err(Hts221Error::InvalidSensor);
        }
        Ctrl2::modify(self.address, &mut self.i2c, |reg| {
            reg.boot();
        })
        .await?;

        Ctrl1::modify(self.address, &mut self.i2c, |reg| {
            reg.power_active()
                .output_data_rate(OutputDataRate::Hz1)
                .block_data_update(BlockDataUpdate::MsbLsbReading);
        })
        .await?;

        Ctrl3::modify(self.address, &mut self.i2c, |reg| {
            reg.enable(true);
        })
        .await?;

        loop {
            // Ensure status is emptied
            if let Ok(status) = Status::read(self.address, &mut self.i2c).await {
                if !status.any_available() {
                    break;
                }
            }
            Hout::read(self.address, &mut self.i2c).await?;
            Tout::read(self.address, &mut self.i2c).await?;
        }

        self.calibration
            .replace(Calibration::read(self.address, &mut self.i2c).await?);
        Ok(())
    }

    /// Read sensor values from driver.
    pub async fn read(&mut self) -> Result<SensorAcquisition<Celsius>, Hts221Error<I::Error>> {
        if let Some(calibration) = &self.calibration {
            let t_out = Tout::read(self.address, &mut self.i2c).await? as i16;
            let temperature = calibration.calibrated_temperature(t_out);

            let h_out = Hout::read(self.address, &mut self.i2c).await?;
            let relative_humidity = calibration.calibrated_humidity(h_out);

            Ok(SensorAcquisition {
                temperature,
                relative_humidity,
            })
        } else {
            Err(Hts221Error::NotCalibrated)
        }
    }
}

impl<E> From<E> for Hts221Error<E>
where
    E: Send,
{
    fn from(e: E) -> Hts221Error<E> {
        Hts221Error::I2c(e)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct I2cAddress(u8);

impl I2cAddress {
    pub fn new(val: u8) -> Self {
        Self(val)
    }
}

impl Into<u8> for I2cAddress {
    fn into(self) -> u8 {
        self.0
    }
}

impl Into<I2cAddress> for u8 {
    fn into(self) -> I2cAddress {
        I2cAddress::new(self)
    }
}

impl LowerHex for I2cAddress {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        LowerHex::fmt(&self.0, f)
    }
}

impl UpperHex for I2cAddress {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        UpperHex::fmt(&self.0, f)
    }
}

/// Trait representing a temperature scale.
pub trait TemperatureScale: Send {
    /// Letter describing temperature
    const LETTER: char;
}

/// Discriminant for the _Kelvin_ temperature scale.
#[derive(Clone)]
pub struct Kelvin;

impl TemperatureScale for Kelvin {
    const LETTER: char = 'K';
}

impl Debug for Kelvin {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.write_str("°K")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Kelvin {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "°K");
    }
}

/// Discriminant for the _Celsius_ temperature scale.
#[derive(Clone)]
pub struct Celsius;

impl Debug for Celsius {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.write_str("°C")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Celsius {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "°C");
    }
}

impl TemperatureScale for Celsius {
    const LETTER: char = 'C';
}

/// Discriminant for the _Fahrenheit_ temperature scale.
#[derive(Clone)]
pub struct Fahrenheit;

impl Debug for Fahrenheit {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.write_str("°F")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Fahrenheit {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "°F");
    }
}

impl TemperatureScale for Fahrenheit {
    const LETTER: char = 'F';
}

/// A temperature value with its associated scale.
pub struct Temperature<S: TemperatureScale> {
    value: f32,
    _marker: PhantomData<S>,
}

impl<S: TemperatureScale> Clone for Temperature<S> {
    fn clone(&self) -> Self {
        Self {
            value: self.value,
            _marker: PhantomData::default(),
        }
    }
}

impl<S: TemperatureScale> Debug for Temperature<S> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}°{}", &self.value, S::LETTER)
    }
}

#[cfg(feature = "defmt")]
impl<S: TemperatureScale> defmt::Format for Temperature<S> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{}°{}", &self.value, S::LETTER)
    }
}

impl<S: TemperatureScale> Copy for Temperature<S> {}

impl<S: TemperatureScale> Temperature<S> {
    fn new(value: f32) -> Self {
        Self {
            value,
            _marker: PhantomData::default(),
        }
    }

    /// Read the raw value.
    pub fn raw_value(&self) -> f32 {
        self.value
    }
}

impl Temperature<Celsius> {
    /// Convert celsius into fahrenheit
    pub fn into_fahrenheit(self) -> Temperature<Fahrenheit> {
        Temperature::new((self.value * 9.0 / 5.0) + 32.0)
    }
}

impl Into<Temperature<Celsius>> for i16 {
    fn into(self) -> Temperature<Celsius> {
        Temperature::<Celsius>::new(self as f32)
    }
}

impl Into<Temperature<Celsius>> for f32 {
    fn into(self) -> Temperature<Celsius> {
        Temperature::<Celsius>::new(self as f32)
    }
}

impl<S: TemperatureScale> Sub for Temperature<S> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.value - rhs.value)
    }
}

impl<S: TemperatureScale> Add for Temperature<S> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.value + rhs.value)
    }
}

impl<S: TemperatureScale> Add<f32> for Temperature<S> {
    type Output = Self;

    fn add(self, rhs: f32) -> Self::Output {
        Self::new(self.value + rhs)
    }
}

impl<S: TemperatureScale> Div<f32> for Temperature<S> {
    type Output = f32;

    fn div(self, rhs: f32) -> Self::Output {
        self.value / rhs
    }
}

impl<S: TemperatureScale> Display for Temperature<S> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Display::fmt(&self.value, f)?;
        write!(f, "°{}", S::LETTER)
    }
}

/// Values read from the driver with the given scale.
#[derive(Copy, Clone)]
pub struct SensorAcquisition<S: TemperatureScale> {
    /// Sensor temperature value.
    pub temperature: Temperature<S>,
    /// Relative humidity.
    pub relative_humidity: f32,
}

impl<S: TemperatureScale> Debug for SensorAcquisition<S> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SensorAcquisition")
            .field("temperature", &self.temperature)
            .field("relative_humidity", &self.relative_humidity)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<S: TemperatureScale> defmt::Format for SensorAcquisition<S> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "SensorAcquisition(temperature: {}, relative_humidity: {})",
            &self.temperature,
            &self.relative_humidity
        );
    }
}
