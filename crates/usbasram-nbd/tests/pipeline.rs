use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::thread;

use usbasram_core::{Store, StoreConfig, PAGE_SIZE};
use usbasram_nbd::serve_tcp;

const NBDMAGIC: u64 = 0x4e42_444d_4147_4943;
const IHAVEOPT: u64 = 0x4948_4156_454f_5054;
const REQUEST_MAGIC: u32 = 0x2560_9513;
const SIMPLE_REPLY_MAGIC: u32 = 0x6744_6698;

#[test]
fn pipelined_reads_and_writes_are_processed_concurrently() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("pipeline.usr");
    let pages = 64u64;
    let store = Arc::new(
        Store::open(
            StoreConfig::new(&path, pages * PAGE_SIZE as u64)
                .with_cache_bytes(pages * PAGE_SIZE as u64),
        )
        .unwrap(),
    );

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let server_store = Arc::clone(&store);
    thread::spawn(move || {
        let _ = serve_tcp(listener, server_store, "usbasram");
    });

    let mut client = TcpStream::connect(address).unwrap();
    client.set_nodelay(true).unwrap();
    handshake(&mut client);

    // Eight independent writes. Replies may arrive in any order.
    let write_pages = 8usize;
    for page in 0..write_pages {
        let data = page_data(page);
        send_request(
            &mut client,
            1,
            100 + page as u64,
            page as u64 * PAGE_SIZE as u64,
            PAGE_SIZE as u32,
            Some(&data),
        );
    }
    let mut write_replies = [false; 8];
    for _ in 0..write_pages {
        let (error, cookie) = read_reply_header(&mut client);
        assert_eq!(error, 0);
        let index = (cookie - 100) as usize;
        assert!(index < write_pages);
        write_replies[index] = true;
    }
    assert!(write_replies.iter().all(|seen| *seen));

    // Thirty-two pipelined reads. A single connection now keeps every worker
    // busy instead of executing requests serially.
    let read_count = 32usize;
    for index in 0..read_count {
        let page = index % write_pages;
        send_request(
            &mut client,
            0,
            1000 + index as u64,
            page as u64 * PAGE_SIZE as u64,
            PAGE_SIZE as u32,
            None,
        );
    }
    let mut read_replies = [false; 32];
    let mut buffer = vec![0u8; PAGE_SIZE];
    for _ in 0..read_count {
        let (error, cookie) = read_reply_header(&mut client);
        assert_eq!(error, 0);
        let index = (cookie - 1000) as usize;
        assert!(index < read_count);
        client.read_exact(&mut buffer).unwrap();
        let page = index % write_pages;
        assert_eq!(buffer, page_data(page), "cookie {cookie} returned wrong data");
        read_replies[index] = true;
    }
    assert!(read_replies.iter().all(|seen| *seen));

    send_request(&mut client, 3, 5000, 0, 0, None);
    let (error, cookie) = read_reply_header(&mut client);
    assert_eq!(error, 0);
    assert_eq!(cookie, 5000);

    send_request(&mut client, 2, 5001, 0, 0, None);
    drop(client);
}

fn handshake(client: &mut TcpStream) {
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
    client.write_all(&3u32.to_be_bytes()).unwrap(); // FIXED_NEWSTYLE | NO_ZEROES

    client.write_all(&IHAVEOPT.to_be_bytes()).unwrap();
    client.write_all(&1u32.to_be_bytes()).unwrap(); // NBD_OPT_EXPORT_NAME
    client.write_all(&0u32.to_be_bytes()).unwrap(); // default export, no payload

    let mut info = [0u8; 10];
    client.read_exact(&mut info).unwrap();
    let capacity = u64::from_be_bytes(info[0..8].try_into().unwrap());
    assert_eq!(capacity, 64 * PAGE_SIZE as u64);
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

fn read_reply_header(client: &mut TcpStream) -> (u32, u64) {
    let mut reply = [0u8; 16];
    client.read_exact(&mut reply).unwrap();
    assert_eq!(
        u32::from_be_bytes(reply[0..4].try_into().unwrap()),
        SIMPLE_REPLY_MAGIC
    );
    let error = u32::from_be_bytes(reply[4..8].try_into().unwrap());
    let cookie = u64::from_be_bytes(reply[8..16].try_into().unwrap());
    (error, cookie)
}

fn page_data(page: usize) -> Vec<u8> {
    vec![page as u8 + 1; PAGE_SIZE]
}
