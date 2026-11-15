//! A small, dependency-free NBD (Network Block Device) server.
//!
//! The server implements fixed-newstyle negotiation, `NBD_OPT_LIST`,
//! `NBD_OPT_INFO`, `NBD_OPT_GO`, and simple replies. It advertises flush, FUA,
//! trim, write-zeroes, and multi-connection support.
#![forbid(unsafe_code)]

use std::io::{self, Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::thread;

use usbasram_core::{Error as StoreError, Store};

const NBDMAGIC: u64 = 0x4e42_444d_4147_4943;
const IHAVEOPT: u64 = 0x4948_4156_454f_5054;
const OPTION_REPLY_MAGIC: u64 = 0x0003_e889_0455_65a9;
const REQUEST_MAGIC: u32 = 0x2560_9513;
const SIMPLE_REPLY_MAGIC: u32 = 0x6744_6698;

const HANDSHAKE_FLAGS: u16 = 1 | 2; // FIXED_NEWSTYLE | NO_ZEROES
const CLIENT_FIXED_NEWSTYLE: u32 = 1;
const CLIENT_NO_ZEROES: u32 = 2;

const OPT_EXPORT_NAME: u32 = 1;
const OPT_ABORT: u32 = 2;
const OPT_LIST: u32 = 3;
const OPT_STARTTLS: u32 = 5;
const OPT_INFO: u32 = 6;
const OPT_GO: u32 = 7;
const OPT_STRUCTURED_REPLY: u32 = 8;

const REP_ACK: u32 = 1;
const REP_SERVER: u32 = 2;
const REP_INFO: u32 = 3;
const REP_ERR_UNSUP: u32 = 0x8000_0001;
const REP_ERR_POLICY: u32 = 0x8000_0002;
const REP_ERR_UNKNOWN: u32 = 0x8000_0006;

const INFO_EXPORT: u16 = 0;
const INFO_BLOCK_SIZE: u16 = 3;

const FLAG_HAS_FLAGS: u16 = 1;
const FLAG_SEND_FLUSH: u16 = 4;
const FLAG_SEND_FUA: u16 = 8;
const FLAG_SEND_TRIM: u16 = 32;
const FLAG_SEND_WRITE_ZEROES: u16 = 64;
const FLAG_CAN_MULTI_CONN: u16 = 256;
const TRANSMISSION_FLAGS: u16 = FLAG_HAS_FLAGS
    | FLAG_SEND_FLUSH
    | FLAG_SEND_FUA
    | FLAG_SEND_TRIM
    | FLAG_SEND_WRITE_ZEROES
    | FLAG_CAN_MULTI_CONN;

const CMD_READ: u16 = 0;
const CMD_WRITE: u16 = 1;
const CMD_DISC: u16 = 2;
const CMD_FLUSH: u16 = 3;
const CMD_TRIM: u16 = 4;
const CMD_WRITE_ZEROES: u16 = 6;
const CMD_FLAG_FUA: u16 = 1;

const EIO: u32 = 5;
const EINVAL: u32 = 22;
const ENOSPC: u32 = 28;
const ENOTSUP: u32 = 95;

const MAX_OPTION_LEN: u32 = 1024 * 1024;
const MAX_REQUEST_LEN: u32 = 32 * 1024 * 1024;

/// Serve NBD over TCP forever, one thread per connection.
pub fn serve_tcp(
    listener: TcpListener,
    store: Arc<Store>,
    export_name: impl Into<String>,
) -> io::Result<()> {
    let export_name = export_name.into();
    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                let _ = stream.set_nodelay(true);
                let store = Arc::clone(&store);
                let export_name = export_name.clone();
                thread::spawn(move || {
                    if let Err(error) = handle_connection(stream, &store, &export_name) {
                        eprintln!("usbasram-nbd: connection ended: {error}");
                    }
                });
            }
            Err(error) => eprintln!("usbasram-nbd: accept failed: {error}"),
        }
    }
    Ok(())
}

/// Serve NBD over a Unix domain socket on Unix platforms.
#[cfg(unix)]
pub fn serve_unix(
    listener: std::os::unix::net::UnixListener,
    store: Arc<Store>,
    export_name: impl Into<String>,
) -> io::Result<()> {
    let export_name = export_name.into();
    for connection in listener.incoming() {
        let stream = connection?;
        let store = Arc::clone(&store);
        let export_name = export_name.clone();
        thread::spawn(move || {
            if let Err(error) = handle_connection(stream, &store, &export_name) {
                eprintln!("usbasram-nbd: connection ended: {error}");
            }
        });
    }
    Ok(())
}

/// Handle one NBD connection over any byte stream.
pub fn handle_connection<S: Read + Write>(
    mut stream: S,
    store: &Store,
    export_name: &str,
) -> io::Result<()> {
    if negotiate(&mut stream, store.capacity_bytes(), export_name)? {
        transmission(&mut stream, store)
    } else {
        Ok(())
    }
}

fn negotiate<S: Read + Write>(
    stream: &mut S,
    capacity: u64,
    export_name: &str,
) -> io::Result<bool> {
    stream.write_all(&NBDMAGIC.to_be_bytes())?;
    stream.write_all(&IHAVEOPT.to_be_bytes())?;
    stream.write_all(&HANDSHAKE_FLAGS.to_be_bytes())?;

    let client_flags = read_u32(stream)?;
    if client_flags & CLIENT_FIXED_NEWSTYLE == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "client did not negotiate fixed newstyle",
        ));
    }
    if client_flags & !(CLIENT_FIXED_NEWSTYLE | CLIENT_NO_ZEROES) != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "client sent unknown handshake flags",
        ));
    }
    let no_zeroes = client_flags & CLIENT_NO_ZEROES != 0;

    loop {
        let magic = read_u64(stream)?;
        if magic != IHAVEOPT {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "client sent an invalid option magic",
            ));
        }
        let option = read_u32(stream)?;
        let length = read_u32(stream)?;
        if length > MAX_OPTION_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "option payload is too large",
            ));
        }
        let mut payload = vec![0u8; length as usize];
        stream.read_exact(&mut payload)?;

        match option {
            OPT_EXPORT_NAME => {
                if !export_matches(&payload, export_name) {
                    return Ok(false);
                }
                stream.write_all(&capacity.to_be_bytes())?;
                stream.write_all(&TRANSMISSION_FLAGS.to_be_bytes())?;
                if !no_zeroes {
                    stream.write_all(&[0u8; 124])?;
                }
                return Ok(true);
            }
            OPT_ABORT => {
                send_option_reply(stream, option, REP_ACK, &[])?;
                return Ok(false);
            }
            OPT_LIST => {
                let mut data = Vec::with_capacity(export_name.len() + 32);
                data.extend_from_slice(&(export_name.len() as u32).to_be_bytes());
                data.extend_from_slice(export_name.as_bytes());
                data.extend_from_slice(b"usbasram tiered RAM/USB store");
                send_option_reply(stream, option, REP_SERVER, &data)?;
                send_option_reply(stream, option, REP_ACK, &[])?;
            }
            OPT_INFO | OPT_GO => {
                let name = parse_go_name(&payload)?;
                if !export_matches(name, export_name) {
                    send_option_reply(stream, option, REP_ERR_UNKNOWN, &[])?;
                    continue;
                }
                send_option_reply(stream, option, REP_INFO, &info_export(capacity))?;
                send_option_reply(stream, option, REP_INFO, &info_block_size())?;
                send_option_reply(stream, option, REP_ACK, &[])?;
                if option == OPT_GO {
                    return Ok(true);
                }
            }
            OPT_STRUCTURED_REPLY => {
                send_option_reply(stream, option, REP_ERR_UNSUP, &[])?;
            }
            OPT_STARTTLS => {
                send_option_reply(stream, option, REP_ERR_POLICY, &[])?;
            }
            _ => {
                send_option_reply(stream, option, REP_ERR_UNSUP, &[])?;
            }
        }
    }
}

fn parse_go_name(payload: &[u8]) -> io::Result<&[u8]> {
    if payload.len() < 4 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "NBD_OPT_GO payload is too short",
        ));
    }
    let length = u32::from_be_bytes(payload[0..4].try_into().expect("fixed slice")) as usize;
    if 4 + length > payload.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "NBD_OPT_GO export name is truncated",
        ));
    }
    Ok(&payload[4..4 + length])
}

fn export_matches(requested: &[u8], configured: &str) -> bool {
    requested.is_empty() || requested == configured.as_bytes()
}

fn info_export(capacity: u64) -> Vec<u8> {
    let mut data = Vec::with_capacity(12);
    data.extend_from_slice(&INFO_EXPORT.to_be_bytes());
    data.extend_from_slice(&capacity.to_be_bytes());
    data.extend_from_slice(&TRANSMISSION_FLAGS.to_be_bytes());
    data
}

fn info_block_size() -> Vec<u8> {
    let mut data = Vec::with_capacity(14);
    data.extend_from_slice(&INFO_BLOCK_SIZE.to_be_bytes());
    data.extend_from_slice(&1u32.to_be_bytes());
    data.extend_from_slice(&4096u32.to_be_bytes());
    data.extend_from_slice(&MAX_REQUEST_LEN.to_be_bytes());
    data
}

fn send_option_reply<S: Write>(
    stream: &mut S,
    option: u32,
    reply_type: u32,
    data: &[u8],
) -> io::Result<()> {
    stream.write_all(&OPTION_REPLY_MAGIC.to_be_bytes())?;
    stream.write_all(&option.to_be_bytes())?;
    stream.write_all(&reply_type.to_be_bytes())?;
    stream.write_all(&(data.len() as u32).to_be_bytes())?;
    stream.write_all(data)
}

fn transmission<S: Read + Write>(stream: &mut S, store: &Store) -> io::Result<()> {
    let mut header = [0u8; 28];
    loop {
        match stream.read_exact(&mut header) {
            Ok(()) => {}
            Err(error) if error.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(error) => return Err(error),
        }
        let magic = u32::from_be_bytes(header[0..4].try_into().expect("fixed slice"));
        if magic != REQUEST_MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid NBD request magic",
            ));
        }
        let flags = u16::from_be_bytes(header[4..6].try_into().expect("fixed slice"));
        let command = u16::from_be_bytes(header[6..8].try_into().expect("fixed slice"));
        let cookie = u64::from_be_bytes(header[8..16].try_into().expect("fixed slice"));
        let offset = u64::from_be_bytes(header[16..24].try_into().expect("fixed slice"));
        let length = u32::from_be_bytes(header[24..28].try_into().expect("fixed slice"));

        if command == CMD_DISC {
            return Ok(());
        }
        if length > MAX_REQUEST_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "request payload is too large",
            ));
        }

        let in_range = offset
            .checked_add(length as u64)
            .map(|end| end <= store.capacity_bytes())
            .unwrap_or(false);
        let fua = flags & CMD_FLAG_FUA != 0;

        match command {
            CMD_READ => {
                if !in_range {
                    send_simple_reply(stream, EINVAL, cookie, None)?;
                    continue;
                }
                let mut data = vec![0u8; length as usize];
                match store.read_at(offset, &mut data) {
                    Ok(read) if read == data.len() => {
                        send_simple_reply(stream, 0, cookie, Some(&data))?;
                    }
                    Ok(_) => send_simple_reply(stream, EIO, cookie, None)?,
                    Err(error) => send_simple_reply(stream, store_error(&error), cookie, None)?,
                }
            }
            CMD_WRITE => {
                let mut data = vec![0u8; length as usize];
                stream.read_exact(&mut data)?;
                if !in_range {
                    send_simple_reply(stream, EINVAL, cookie, None)?;
                    continue;
                }
                let result = store
                    .write_at(offset, &data)
                    .and_then(|written| {
                        if written != data.len() {
                            return Err(StoreError::Config("short write".into()));
                        }
                        if fua {
                            store.flush()?;
                        }
                        Ok(())
                    });
                match result {
                    Ok(()) => send_simple_reply(stream, 0, cookie, None)?,
                    Err(error) => send_simple_reply(stream, store_error(&error), cookie, None)?,
                }
            }
            CMD_FLUSH => match store.flush() {
                Ok(()) => send_simple_reply(stream, 0, cookie, None)?,
                Err(error) => send_simple_reply(stream, store_error(&error), cookie, None)?,
            },
            CMD_TRIM | CMD_WRITE_ZEROES => {
                if !in_range {
                    send_simple_reply(stream, EINVAL, cookie, None)?;
                    continue;
                }
                let result = store.trim(offset, length as u64).and_then(|()| {
                    if fua {
                        store.flush()?;
                    }
                    Ok(())
                });
                match result {
                    Ok(()) => send_simple_reply(stream, 0, cookie, None)?,
                    Err(error) => send_simple_reply(stream, store_error(&error), cookie, None)?,
                }
            }
            _ => send_simple_reply(stream, ENOTSUP, cookie, None)?,
        }
    }
}

fn send_simple_reply<S: Write>(
    stream: &mut S,
    error: u32,
    cookie: u64,
    data: Option<&[u8]>,
) -> io::Result<()> {
    stream.write_all(&SIMPLE_REPLY_MAGIC.to_be_bytes())?;
    stream.write_all(&error.to_be_bytes())?;
    stream.write_all(&cookie.to_be_bytes())?;
    if let Some(data) = data {
        stream.write_all(data)?;
    }
    Ok(())
}

fn store_error(error: &StoreError) -> u32 {
    match error {
        StoreError::LogFull { .. } => ENOSPC,
        StoreError::Config(_) | StoreError::PageOutOfRange { .. } => EINVAL,
        _ => EIO,
    }
}

fn read_u32<R: Read>(reader: &mut R) -> io::Result<u32> {
    let mut buffer = [0u8; 4];
    reader.read_exact(&mut buffer)?;
    Ok(u32::from_be_bytes(buffer))
}

fn read_u64<R: Read>(reader: &mut R) -> io::Result<u64> {
    let mut buffer = [0u8; 8];
    reader.read_exact(&mut buffer)?;
    Ok(u64::from_be_bytes(buffer))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go_name_parsing_rejects_truncation() {
        assert!(parse_go_name(&[0, 0, 0, 8, b'x']).is_err());
        assert_eq!(parse_go_name(&[0, 0, 0, 1, b'x']).unwrap(), b"x");
    }

    #[test]
    fn empty_export_name_selects_the_default() {
        assert!(export_matches(b"", "usbasram"));
        assert!(export_matches(b"usbasram", "usbasram"));
        assert!(!export_matches(b"other", "usbasram"));
    }
}
