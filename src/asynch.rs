//! Asynchronous driver implementations for the 24x series serial EEPROMs.
//!
//! Available when the `async` feature is enabled.

use crate::{
    eeprom24x::MultiSizeAddr,
    page_size, Eeprom24x, Error, Storage,
};
use core::cmp::min;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

/// Asynchronous PageWrite trait for EEPROMs.
#[allow(async_fn_in_trait)]
pub trait AsyncPageWrite<E> {
    /// Write up to a page to the EEPROM asynchronously.
    async fn async_page_write(&mut self, address: u32, data: &[u8]) -> Result<(), Error<E>>;

    /// Returns the page size in bytes.
    fn page_size(&self) -> usize;
}

/// Asynchronous common read/write methods
impl<I2C, E, PS, AS, SN> Eeprom24x<I2C, PS, AS, SN>
where
    I2C: I2c<Error = E>,
    AS: MultiSizeAddr,
{
    /// Write a single byte to an address asynchronously.
    pub async fn write_byte_async(&mut self, address: u32, data: u8) -> Result<(), Error<E>> {
        let devaddr = self.get_device_address(address)?;
        let mut payload = [0; 3];
        AS::fill_address(address, &mut payload);
        payload[AS::ADDRESS_BYTES] = data;
        self.i2c
            .write(devaddr, &payload[..=AS::ADDRESS_BYTES])
            .await
            .map_err(Error::I2C)
    }

    /// Read a single byte from an address asynchronously.
    pub async fn read_byte_async(&mut self, address: u32) -> Result<u8, Error<E>> {
        let devaddr = self.get_device_address(address)?;
        let mut memaddr = [0; 2];
        AS::fill_address(address, &mut memaddr);
        let mut data = [0; 1];
        self.i2c
            .write_read(devaddr, &memaddr[..AS::ADDRESS_BYTES], &mut data)
            .await
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    /// Read starting at an address as many bytes as necessary to fill `data` asynchronously.
    pub async fn read_data_async(&mut self, address: u32, data: &mut [u8]) -> Result<(), Error<E>> {
        let devaddr = self.get_device_address(address)?;
        let mut memaddr = [0; 2];
        AS::fill_address(address, &mut memaddr);
        self.i2c
            .write_read(devaddr, &memaddr[..AS::ADDRESS_BYTES], data)
            .await
            .map_err(Error::I2C)
    }
}

/// Asynchronous read current address for platforms with simple read
impl<I2C, E, PS, AS, SN> Eeprom24x<I2C, PS, AS, SN>
where
    I2C: I2c<Error = E>,
{
    /// Read the contents of the last address accessed incremented by one asynchronously.
    pub async fn read_current_address_async(&mut self) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .read(self.address.addr(), &mut data)
            .await
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}

macro_rules! impl_async_for_page_size {
    ($PS:ident, $page_size:expr) => {
        impl<I2C, E, AS, SN> Eeprom24x<I2C, page_size::$PS, AS, SN>
        where
            I2C: I2c<Error = E>,
            AS: MultiSizeAddr,
        {
            /// Write up to a page starting at `address` asynchronously.
            pub async fn write_page_async(
                &mut self,
                address: u32,
                data: &[u8],
            ) -> Result<(), Error<E>> {
                if data.is_empty() {
                    return Ok(());
                }

                if data.len() > $page_size {
                    return Err(Error::TooMuchData);
                }

                let page_boundary = address | ($page_size as u32 - 1);
                if address + data.len() as u32 > page_boundary + 1 {
                    return Err(Error::TooMuchData);
                }

                let devaddr = self.get_device_address(address)?;
                let mut payload = [0u8; 2 + $page_size];
                AS::fill_address(address, &mut payload);
                let addr_bytes = AS::ADDRESS_BYTES;
                payload[addr_bytes..addr_bytes + data.len()].copy_from_slice(data);
                self.i2c
                    .write(devaddr, &payload[..addr_bytes + data.len()])
                    .await
                    .map_err(Error::I2C)
            }
        }

        impl<I2C, E, AS, SN> AsyncPageWrite<E> for Eeprom24x<I2C, page_size::$PS, AS, SN>
        where
            I2C: I2c<Error = E>,
            AS: MultiSizeAddr,
        {
            async fn async_page_write(
                &mut self,
                address: u32,
                data: &[u8],
            ) -> Result<(), Error<E>> {
                self.write_page_async(address, data).await
            }

            fn page_size(&self) -> usize {
                $page_size
            }
        }
    };
}

impl_async_for_page_size!(B8, 8);
impl_async_for_page_size!(B16, 16);
impl_async_for_page_size!(B32, 32);
impl_async_for_page_size!(B64, 64);
impl_async_for_page_size!(B128, 128);
impl_async_for_page_size!(B256, 256);

// ── embedded-storage-async implementations for Storage ───────────────────────

impl<I2C, E, PS, AS, SN, D> embedded_storage_async::ReadStorage
    for Storage<I2C, PS, AS, SN, D>
where
    I2C: I2c<Error = E>,
    AS: MultiSizeAddr,
    D: DelayNs,
{
    type Error = Error<E>;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.eeprom.read_data_async(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        1 << self.eeprom.address_bits
    }
}

impl<I2C, E, PS, AS, SN, D> embedded_storage_async::Storage
    for Storage<I2C, PS, AS, SN, D>
where
    I2C: I2c<Error = E>,
    AS: MultiSizeAddr,
    Eeprom24x<I2C, PS, AS, SN>: AsyncPageWrite<E>,
    D: DelayNs,
{
    async fn write(&mut self, mut offset: u32, mut bytes: &[u8]) -> Result<(), Self::Error> {
        use embedded_storage_async::ReadStorage;
        if offset as usize + bytes.len() > self.capacity() {
            return Err(Error::TooMuchData);
        }
        let page_size = self.eeprom.page_size();
        while !bytes.is_empty() {
            let this_page_offset = offset as usize % page_size;
            let this_page_remaining = page_size - this_page_offset;
            let chunk_size = min(bytes.len(), this_page_remaining);
            self.eeprom
                .async_page_write(offset, &bytes[..chunk_size])
                .await?;
            offset += chunk_size as u32;
            bytes = &bytes[chunk_size..];

            // Wait 5ms for EEPROM internal write completion cycle asynchronously
            self.delay.delay_ms(5).await;
        }
        Ok(())
    }
}

