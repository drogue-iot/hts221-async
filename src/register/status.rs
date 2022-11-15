use super::super::I2cAddress;
use embedded_hal_async::i2c::*;

const STATUS: u8 = 0x27;

pub(crate) struct Status {
    temperature_available: bool,
    humidity_available: bool,
}

impl Status {
    pub async fn read<I: I2c>(address: I2cAddress, i2c: &mut I) -> Result<Status, I::Error> {
        let mut buf = [0; 1];
        let _ = i2c.write_read(address.into(), &[STATUS], &mut buf).await?;
        Ok(buf[0].into())
    }

    #[allow(dead_code)]
    pub fn temperature_available(&self) -> bool {
        self.temperature_available
    }

    #[allow(dead_code)]
    pub fn humidity_available(&self) -> bool {
        self.humidity_available
    }

    #[allow(dead_code)]
    pub fn any_available(&self) -> bool {
        self.temperature_available || self.humidity_available
    }
}

impl Into<Status> for u8 {
    fn into(self) -> Status {
        Status {
            temperature_available: ((self & 0b01) != 0),
            humidity_available: ((self & 0b10) != 0),
        }
    }
}
