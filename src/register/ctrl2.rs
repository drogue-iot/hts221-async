use super::super::I2cAddress;
use embedded_hal_async::i2c::*;

const CTRL_REG2: u8 = 0x21;

#[derive(Debug, Copy, Clone)]
pub struct Ctrl2 {
    boot: bool,
    heater: bool,
    enable_one_shot: bool,
}

impl Ctrl2 {
    pub async fn read<I: I2c>(address: I2cAddress, i2c: &mut I) -> Result<Ctrl2, I::Error> {
        let mut buf = [0; 1];
        let _ = i2c
            .write_read(address.into(), &[CTRL_REG2], &mut buf)
            .await?;
        Ok(buf[0].into())
    }

    pub async fn write<I: I2c>(
        address: I2cAddress,
        i2c: &mut I,
        reg: Ctrl2,
    ) -> Result<(), I::Error> {
        Ok(i2c.write(address.into(), &[CTRL_REG2, reg.into()]).await?)
    }

    pub async fn modify<I: I2c, F: FnOnce(&mut Ctrl2)>(
        address: I2cAddress,
        i2c: &mut I,
        modify: F,
    ) -> Result<(), I::Error> {
        let mut reg = Self::read(address.into(), i2c).await?;
        modify(&mut reg);
        Self::write(address.into(), i2c, reg).await
    }

    pub fn boot(&mut self) -> &mut Self {
        self.boot = true;
        self
    }

    pub fn heater(&mut self, on: bool) -> &mut Self {
        self.heater = on;
        self
    }

    pub fn enable_one_shot(&mut self) -> &mut Self {
        self.enable_one_shot = true;
        self
    }
}

impl Into<Ctrl2> for u8 {
    fn into(self) -> Ctrl2 {
        let boot = (self & 0b10000000) != 0;
        let heater = (self & 0b00000010) != 0;
        let enable_one_shot = (self & 0b00000001) != 0;

        Ctrl2 {
            boot,
            heater,
            enable_one_shot,
        }
    }
}

impl Into<u8> for Ctrl2 {
    fn into(self) -> u8 {
        let mut val = 0;

        if self.boot {
            val |= 0b10000000;
        }

        if self.heater {
            val |= 0b00000010;
        }

        if self.enable_one_shot {
            val |= 0b00000001;
        }

        val
    }
}
