//! A CDR (XCDR1) reader for ROS 2 messages over rmw_zenoh: a 4-byte encapsulation header, then primitives aligned
//! to their size; little or big endian. Zero-copy over the sample payload.

use anyhow::{Context, Result, bail, ensure};

/// Reads CDR: primitives aligned to their size relative to the end of the encapsulation header.
pub(crate) struct Cdr<'a> {
    body: &'a [u8],
    position: usize,
    little_endian: bool,
}

impl<'a> Cdr<'a> {
    pub(crate) fn new(payload: &'a [u8]) -> Result<Self> {
        ensure!(payload.len() >= 4, "CDR payload shorter than its encapsulation header");
        // representation identifier: 0x0000 CDR_BE, 0x0001 CDR_LE (XCDR2 forms 0x0006..0x000b too)
        let little_endian = match payload[1] {
            0x00 | 0x02 | 0x06 | 0x08 | 0x0a => false,
            0x01 | 0x03 | 0x07 | 0x09 | 0x0b => true,
            other => bail!("unknown CDR representation 0x{:02x}{other:02x}", payload[0]),
        };
        Ok(Cdr { body: &payload[4..], position: 0, little_endian })
    }

    pub(crate) fn take(&mut self, length: usize, alignment: usize) -> Result<&'a [u8]> {
        self.position = self.position.next_multiple_of(alignment);
        let end = self.position.checked_add(length).context("CDR length overflow")?;
        ensure!(end <= self.body.len(), "CDR message truncated (need {end} bytes, have {})", self.body.len());
        let bytes = &self.body[self.position..end];
        self.position = end;
        Ok(bytes)
    }

    pub(crate) fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1, 1)?[0])
    }

    pub(crate) fn u32(&mut self) -> Result<u32> {
        let bytes: [u8; 4] = self.take(4, 4)?.try_into()?;
        Ok(if self.little_endian { u32::from_le_bytes(bytes) } else { u32::from_be_bytes(bytes) })
    }

    pub(crate) fn string(&mut self) -> Result<String> {
        let length = self.u32()? as usize;
        let bytes = self.take(length, 1)?;
        Ok(String::from_utf8_lossy(bytes.strip_suffix(&[0]).unwrap_or(bytes)).into_owned())
    }

    pub(crate) fn byte_sequence(&mut self) -> Result<&'a [u8]> {
        let length = self.u32()? as usize;
        self.take(length, 1)
    }

    /// std_msgs/Header: builtin_interfaces/Time stamp (i32 sec, u32 nanosec) + string frame_id
    pub(crate) fn skip_header(&mut self) -> Result<()> {
        self.u32()?;
        self.u32()?;
        self.string()?;
        Ok(())
    }
}
