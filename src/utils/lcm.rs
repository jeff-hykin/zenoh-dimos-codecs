//! An LCM reader for dimos messages over zenoh: an 8-byte type fingerprint, then big-endian packed fields.
//! Zero-copy over the sample payload.

use anyhow::{Context, Result, ensure};

/// Reads LCM: big endian, packed, strings as `i32 length (incl. NUL) | bytes | NUL`.
pub(crate) struct Lcm<'a> {
    body: &'a [u8],
    position: usize,
}

impl<'a> Lcm<'a> {
    pub(crate) fn new(payload: &'a [u8], fingerprint: [u8; 8], type_name: &str) -> Result<Self> {
        ensure!(payload.len() >= 8, "LCM payload shorter than its fingerprint");
        ensure!(payload[..8] == fingerprint, "not an LCM {type_name} (fingerprint {:02x?})", &payload[..8]);
        Ok(Lcm { body: payload, position: 8 })
    }

    pub(crate) fn take(&mut self, length: usize) -> Result<&'a [u8]> {
        let end = self.position.checked_add(length).context("LCM length overflow")?;
        ensure!(end <= self.body.len(), "LCM message truncated (need {end} bytes, have {})", self.body.len());
        let bytes = &self.body[self.position..end];
        self.position = end;
        Ok(bytes)
    }

    pub(crate) fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }

    pub(crate) fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_be_bytes(self.take(4)?.try_into()?))
    }

    pub(crate) fn length(&mut self) -> Result<usize> {
        let value = self.i32()?;
        usize::try_from(value).context("negative LCM length")
    }

    pub(crate) fn string(&mut self) -> Result<String> {
        let length = self.length()?;
        let bytes = self.take(length)?;
        Ok(String::from_utf8_lossy(bytes.strip_suffix(&[0]).unwrap_or(bytes)).into_owned())
    }

    /// std_msgs.Header: i32 seq, std_msgs.Time stamp (i32 sec, i32 nsec), string frame_id
    pub(crate) fn skip_header(&mut self) -> Result<()> {
        self.take(12)?;
        self.string()?;
        Ok(())
    }
}
