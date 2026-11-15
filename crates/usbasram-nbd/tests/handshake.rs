use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::thread;

use usbasram_core::{Store, StoreConfig, PAGE_SIZE};
use usbasram_nbd::handle_connection;

const NBDMAGIC: u64 = 0x4e42_444d_4147_4943;
const IHAVEOPT: u64 = 0x4948_4156_454f_5054;
const OPTION_REPLY_MAGIC: u64 = 0x0003_e889_0455_65a9;
const REQUEST_MAGIC: u32 = 0x2560_9513;
const SIMPLE_REPLY_MAGIC: u32 = 0x6744_6698;

#[test]
fn handshake_write_read_flush() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("nbd.usr");
    let store = Arc::new(
        Store::open(
            StoreConfig::new(&path, 64 * PAGE_SIZE as u64)
                .with_cache_bytes(64 * PAGE_SIZE as u64),
        )
        .unwrap(),
    );

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let server_store = Arc::clone(&store);
    let server = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        handle_connection(stream, &server_store, "usbasram").unwrap();
    });

    let mut client = TcpStream::connect(address).unwrap();
    let mut hello = [0u8; 18];
    client.read_exact(&mut hello).unwrap();
    assert_eq!(
        u64::from_be_bytes(hello[0..8].try_into().unwrap()),
        NBDMAGIC
    );
    assert_eq!(
        u64::from_be_bytes(hello[8..16].try_into().unwrap()),
        IHAVEOPT
    );
    let handshake_flags = u16::from_be_bytes(hello[16..18].try_into().unwrap());
    assert_eq!(handshake_flags & 1, 1);
    client.write_all(&3u32.to_be_bytes()).unwrap(); // FIXED_NEWSTYLE | NO_ZEROES

    let name = b"usbasram";
    let mut payload = Vec::new();
    payload.extend_from_slice(&(name.len() as u32).to_be_bytes());
    payload.extend_from_slice(name);
    payload.extend_from_slice(&0u16.to_be_bytes());
    client.write_all(&IHAVEOPT.to_be_bytes()).unwrap();
    client.write_all(&7u32.to_be_bytes()).unwrap(); // NBD_OPT_GO
    client.write_all(&(payload.len() as u32).to_be_bytes()).unwrap();
    client.write_all(&payload).unwrap();

    loop {
        let mut header = [0u8; 20];
        client.read_exact(&mut header).unwrap();
        assert_eq!(
            u64::from_be_bytes(header[0..8].try_into().unwrap()),
            OPTION_REPLY_MAGIC
        );
        assert_eq!(
            u32::from_be_bytes(header[8..12].try_into().unwrap()),
            7,
            "option reply is for the wrong option"
        );
        let reply_type = u32::from_be_bytes(header[12..16].try_into().unwrap());
        let length = u32::from_be_bytes(header[16..20].try_into().unwrap()) as usize;
        let mut data = vec![0u8; length];
        client.read_exact(&mut data).unwrap();
        if reply_type == 1 {
            break;
        }
    }

    let mut data = vec![0x5au8; PAGE_SIZE];
    data[0] = 1;
    send_request(&mut client, 1, 42, 3 * PAGE_SIZE as u64, PAGE_SIZE as u32, Some(&data));
    read_simple_reply(&mut client, 42);

    send_request(&mut client, 3, 43, 0, 0, None);
    read_simple_reply(&mut client, 43);

    send_request(&mut client, 0, 44, 3 * PAGE_SIZE as u64, PAGE_SIZE as u32, None);
    let mut reply = [0u8; 16];
    client.read_exact(&mut reply).unwrap();
    assert_eq!(
        u32::from_be_bytes(reply[0..4].try_into().unwrap()),
        SIMPLE_REPLY_MAGIC
    );
    assert_eq!(u32::from_be_bytes(reply[4..8].try_into().unwrap()), 0);
    assert_eq!(u64::from_be_bytes(reply[8..16].try_into().unwrap()), 44);
    let mut read_back = vec![0u8; PAGE_SIZE];
    client.read_exact(&mut read_back).unwrap();
    assert_eq!(read_back, data);

    send_request(&mut client, 2, 45, 0, 0, None);
    drop(client);
    server.join().unwrap();

    store.flush().unwrap();
    let mut out = vec![0u8; PAGE_SIZE];
    store.read_at(3 * PAGE_SIZE as u64, &mut out).unwrap();
    assert_eq!(out, data);
}

fn send_request(
    client: &mut TcpStream,
    command: u16,
    cookie: u64,
    offset: u64,
    length: u32,
    payload: Option<&[u8]>,
) {
    client.write_all(&REQUEST_MAGIC.to_be_bytes()).unwrap();
    client.write_all(&0u16.to_be_bytes()).unwrap();
    client.write_all(&command.to_be_bytes()).unwrap();
    client.write_all(&cookie.to_be_bytes()).unwrap();
    client.write_all(&offset.to_be_bytes()).unwrap();
    client.write_all(&length.to_be_bytes()).unwrap();
    if let Some(payload) = payload {
        client.write_all(payload).unwrap();
    }
}

fn read_simple_reply(client: &mut TcpStream, cookie: u64) {
    let mut reply = [0u8; 16];
    client.read_exact(&mut reply).unwrap();
    assert_eq!(
        u32::from_be_bytes(reply[0..4].try_into().unwrap()),
        SIMPLE_REPLY_MAGIC
    );
    assert_eq!(u32::from_be_bytes(reply[4..8].try_into().unwrap()), 0);
    assert_eq!(u64::from_be_bytes(reply[8..16].try_into().unwrap()), cookie);
}
