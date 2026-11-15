# Security policy

## Supported versions

Security fixes are applied to the latest release on the `main` branch. There is
no long-term support branch at this time.

## Reporting a vulnerability

Please do not open a public issue for a vulnerability that could put users at
risk. Instead, use GitHub's private vulnerability reporting for this repository
(Security -> Report a vulnerability), or contact the maintainer through the
address listed on the repository profile. Include:

- a description of the issue,
- the affected version or commit,
- reproduction steps or a proof of concept,
- the impact you believe it has.

You will receive an acknowledgement as soon as possible. Please allow time for a
fix and a coordinated disclosure.

## Threat model

`usbasram` is a local storage engine with an NBD front end. It assumes:

- the host operating system and the user running the process are trusted,
- the backing file is protected by filesystem permissions,
- the NBD listener is not exposed to untrusted networks by default,
- the USB device may fail, be removed, or return corrupted data.

## Important limitations

- **No authentication or encryption in NBD.** The default bind is
  `127.0.0.1:10809`. Anyone who can connect to the listener has raw block-device
  access to the store. Use a firewall, a private network, or a TLS/SSH tunnel if
  you must access it remotely. Do not bind to `0.0.0.0` on an untrusted network.
- **No file locking.** Do not open the same store file from two processes at the
  same time. Concurrent writers can corrupt the log.
- **Corruption detection, not repair.** CRC32C detects damaged records and
  superblocks. It cannot reconstruct lost data. Keep backups of important
  stores.
- **Power loss.** A write is durable only after a successful flush (or an NBD
  `FLUSH`/`FUA`). Use a UPS or frequent flushes if the workload requires it.
- **USB removal.** Removing the device during a flush or compaction can leave a
  sidecar or a torn tail. Recovery handles the documented cases, but physical
  device failure can still lose data.
- **Denial of service.** The NBD server accepts up to 32 MiB requests and one
  thread per connection. It is intended for a trusted local client, not as a
  public service.

## Hardening checklist

- Bind to loopback unless remote access is required.
- Restrict the store file and directory to the service account.
- Run the server as an unprivileged user.
- Set a flush interval that matches your durability requirements.
- Run `usbasram verify` after an unclean shutdown or a suspected device fault.
- Back up the store file when the service is stopped.
