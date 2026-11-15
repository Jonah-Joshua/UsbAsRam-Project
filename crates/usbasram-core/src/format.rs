use crate::error::{Error, Result};
use crate::io::align_up;
use crate::PAGE_SIZE;

pub(crate) const SUPERBLOCK_SIZE: usize = 4096;
pub(crate) const SUPERBLOCK_SLOTS: u64 = 2;
pub(crate) const DATA_OFFSET: u64 = SUPERBLOCK_SIZE as u64 * SUPERBLOCK_SLOTS;
pub(crate) const SUPERBLOCK_MAGIC: [u8; 8] = *b"USRSTORE";
pub(crate) const SUPERBLOCK_VERSION: u32 = 1;
pub(crate) const SUPERBLOCK_FLAG_COMPRESSED: u32 = 1;

const SUPERBLOCK_CRC_OFFSET: usize = 64;

/// The commit record at the head of every log.
///
/// Two slots are used in an A/B fashion. A crash can only damage the slot being
/// written; the other slot remains a valid checkpoint. If both slots are valid,
/// the one with the highest `checkpoint_generation` wins.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Superblock {
    pub capacity_pages: u64,
    pub log_bytes: u64,
    pub head: u64,
    pub epoch: u64,
    pub checkpoint_generation: u64,
    pub flags: u32,
}

impl Superblock {
    pub(crate) fn new(
        capacity_pages: u64,
        log_bytes: u64,
        head: u64,
        epoch: u64,
        checkpoint_generation: u64,
        compressed: bool,
    ) -> Self {
        Self {
            capacity_pages,
            log_bytes,
            head,
            epoch,
            checkpoint_generation,
            flags: if compressed {
                SUPERBLOCK_FLAG_COMPRESSED
            } else {
                0
            },
        }
    }

    pub(crate) fn compressed(&self) -> bool {
        self.flags & SUPERBLOCK_FLAG_COMPRESSED != 0
    }

    pub(crate) fn encode(&self) -> [u8; SUPERBLOCK_SIZE] {
        let mut out = [0u8; SUPERBLOCK_SIZE];
        out[0..8].copy_from_slice(&SUPERBLOCK_MAGIC);
        out[8..12].copy_from_slice(&SUPERBLOCK_VERSION.to_le_bytes());
        out[12..16].copy_from_slice(&(PAGE_SIZE as u32).to_le_bytes());
        out[16..24].copy_from_slice(&self.capacity_pages.to_le_bytes());
        out[24..32].copy_from_slice(&self.log_bytes.to_le_bytes());
        out[32..40].copy_from_slice(&self.head.to_le_bytes());
        out[40..48].copy_from_slice(&self.epoch.to_le_bytes());
        out[48..56].copy_from_slice(&self.checkpoint_generation.to_le_bytes());
        out[56..60].copy_from_slice(&self.flags.to_le_bytes());
        let crc = crc32c::crc32c(&out[..SUPERBLOCK_CRC_OFFSET]);
        out[SUPERBLOCK_CRC_OFFSET..SUPERBLOCK_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        out
    }

    pub(crate) fn decode(buf: &[u8]) -> Result<Self> {
        if buf.len() < SUPERBLOCK_CRC_OFFSET + 4 {
            return Err(Error::corrupt("superblock is too short"));
        }
        if buf[0..8] != SUPERBLOCK_MAGIC[..] {
            return Err(Error::corrupt("superblock magic does not match"));
        }
        let version = u32::from_le_bytes(buf[8..12].try_into().expect("fixed slice"));
        if version != SUPERBLOCK_VERSION {
            return Err(Error::UnsupportedVersion(version));
        }
        let page_size = u32::from_le_bytes(buf[12..16].try_into().expect("fixed slice"));
        if page_size as usize != PAGE_SIZE {
            return Err(Error::corrupt(format!(
                "page size {page_size} does not match {PAGE_SIZE}"
            )));
        }
        let stored_crc = u32::from_le_bytes(
            buf[SUPERBLOCK_CRC_OFFSET..SUPERBLOCK_CRC_OFFSET + 4]
                .try_into()
                .expect("fixed slice"),
        );
        let actual_crc = crc32c::crc32c(&buf[..SUPERBLOCK_CRC_OFFSET]);
        if stored_crc != actual_crc {
            return Err(Error::corrupt("superblock checksum mismatch"));
        }

        let capacity_pages = u64::from_le_bytes(buf[16..24].try_into().expect("fixed slice"));
        let log_bytes = u64::from_le_bytes(buf[24..32].try_into().expect("fixed slice"));
        let head = u64::from_le_bytes(buf[32..40].try_into().expect("fixed slice"));
        let epoch = u64::from_le_bytes(buf[40..48].try_into().expect("fixed slice"));
        let checkpoint_generation =
            u64::from_le_bytes(buf[48..56].try_into().expect("fixed slice"));
        let flags = u32::from_le_bytes(buf[56..60].try_into().expect("fixed slice"));

        if capacity_pages == 0 {
            return Err(Error::corrupt("superblock has zero capacity"));
        }
        if log_bytes < PAGE_SIZE as u64 {
            return Err(Error::corrupt("superblock log is smaller than one page"));
        }
        if head < DATA_OFFSET || head > DATA_OFFSET + log_bytes {
            return Err(Error::corrupt(format!(
                "superblock head {head} is outside the log"
            )));
        }
        if epoch == 0 {
            return Err(Error::corrupt("superblock epoch is zero"));
        }
        if checkpoint_generation == 0 {
            return Err(Error::corrupt("superblock generation is zero"));
        }
        if flags & !SUPERBLOCK_FLAG_COMPRESSED != 0 {
            return Err(Error::corrupt(format!(
                "superblock has unknown flags 0x{flags:08x}"
            )));
        }

        Ok(Self {
            capacity_pages,
            log_bytes,
            head,
            epoch,
            checkpoint_generation,
            flags,
        })
    }
}

pub(crate) const RECORD_MAGIC: u32 = 0x5553_5252; // "USRR" in little-endian order
pub(crate) const RECORD_VERSION: u16 = 1;
pub(crate) const RECORD_HEADER_SIZE: usize = 48;
pub(crate) const RECORD_FLAG_COMPRESSED: u16 = 1;
pub(crate) const MAX_STORED_LEN: usize = PAGE_SIZE + 1024;

const RECORD_CRC_OFFSET: usize = 44;

/// Header of an append-only page record. The payload follows immediately and
/// is padded with zeroes to an 8-byte boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RecordHeader {
    pub flags: u16,
    pub page_no: u64,
    pub generation: u64,
    pub epoch: u64,
    pub raw_len: u32,
    pub stored_len: u32,
    pub payload_crc: u32,
}

impl RecordHeader {
    pub(crate) fn is_compressed(&self) -> bool {
        self.flags & RECORD_FLAG_COMPRESSED != 0
    }

    pub(crate) fn record_len(&self) -> u64 {
        align_up(RECORD_HEADER_SIZE as u64 + self.stored_len as u64, 8)
    }

    pub(crate) fn encode(&self) -> [u8; RECORD_HEADER_SIZE] {
        let mut out = [0u8; RECORD_HEADER_SIZE];
        out[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        out[4..6].copy_from_slice(&RECORD_VERSION.to_le_bytes());
        out[6..8].copy_from_slice(&self.flags.to_le_bytes());
        out[8..16].copy_from_slice(&self.page_no.to_le_bytes());
        out[16..24].copy_from_slice(&self.generation.to_le_bytes());
        out[24..32].copy_from_slice(&self.epoch.to_le_bytes());
        out[32..36].copy_from_slice(&self.raw_len.to_le_bytes());
        out[36..40].copy_from_slice(&self.stored_len.to_le_bytes());
        out[40..44].copy_from_slice(&self.payload_crc.to_le_bytes());
        let crc = crc32c::crc32c(&out[..RECORD_CRC_OFFSET]);
        out[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        out
    }

    pub(crate) fn decode(buf: &[u8]) -> Result<Self> {
        if buf.len() < RECORD_HEADER_SIZE {
            return Err(Error::corrupt("record header is too short"));
        }
        let magic = u32::from_le_bytes(buf[0..4].try_into().expect("fixed slice"));
        if magic != RECORD_MAGIC {
            return Err(Error::corrupt("record magic does not match"));
        }
        let version = u16::from_le_bytes(buf[4..6].try_into().expect("fixed slice"));
        if version != RECORD_VERSION {
            return Err(Error::corrupt(format!(
                "unsupported record version {version}"
            )));
        }
        let stored_crc = u32::from_le_bytes(
            buf[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4]
                .try_into()
                .expect("fixed slice"),
        );
        let actual_crc = crc32c::crc32c(&buf[..RECORD_CRC_OFFSET]);
        if stored_crc != actual_crc {
            return Err(Error::corrupt("record header checksum mismatch"));
        }
        let flags = u16::from_le_bytes(buf[6..8].try_into().expect("fixed slice"));
        if flags & !RECORD_FLAG_COMPRESSED != 0 {
            return Err(Error::corrupt(format!(
                "record has unknown flags 0x{flags:04x}"
            )));
        }
        let raw_len = u32::from_le_bytes(buf[32..36].try_into().expect("fixed slice"));
        let stored_len = u32::from_le_bytes(buf[36..40].try_into().expect("fixed slice"));
        let generation = u64::from_le_bytes(buf[16..24].try_into().expect("fixed slice"));
        if generation == 0 {
            return Err(Error::corrupt("record generation is zero"));
        }
        if raw_len as usize != PAGE_SIZE {
            return Err(Error::corrupt(format!(
                "record raw length {raw_len} does not match {PAGE_SIZE}"
            )));
        }
        if stored_len == 0 || stored_len as usize > MAX_STORED_LEN {
            return Err(Error::corrupt(format!(
                "record stored length {stored_len} is invalid"
            )));
        }
        Ok(Self {
            flags,
            page_no: u64::from_le_bytes(buf[8..16].try_into().expect("fixed slice")),
            generation,
            epoch: u64::from_le_bytes(buf[24..32].try_into().expect("fixed slice")),
            raw_len,
            stored_len,
            payload_crc: u32::from_le_bytes(buf[40..44].try_into().expect("fixed slice")),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn superblock_round_trips() {
        let sb = Superblock::new(1024, 4 * 1024 * 1024, DATA_OFFSET + 1234, 7, 9, true);
        let encoded = sb.encode();
        let decoded = Superblock::decode(&encoded).expect("decode");
        assert_eq!(sb, decoded);
        assert!(decoded.compressed());
    }

    #[test]
    fn superblock_detects_corruption() {
        let sb = Superblock::new(1024, 4 * 1024 * 1024, DATA_OFFSET, 1, 1, false);
        let mut encoded = sb.encode();
        encoded[20] ^= 0xff;
        assert!(Superblock::decode(&encoded).unwrap_err().is_corrupt());
    }

    #[test]
    fn record_round_trips() {
        let header = RecordHeader {
            flags: 0,
            page_no: 42,
            generation: 11,
            epoch: 3,
            raw_len: PAGE_SIZE as u32,
            stored_len: 100,
            payload_crc: crc32c::crc32c(&[7u8; 100]),
        };
        let decoded = RecordHeader::decode(&header.encode()).expect("decode");
        assert_eq!(header, decoded);
        assert_eq!(decoded.record_len(), 152);
    }
}
