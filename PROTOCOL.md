# X1D-50c USB protocol notes

Decoded from USB recordings of Phocus 4.2 on Windows driving an X1D-50c (firmware 1.25.0) in November 2026.

## Transport
USB ID `2756:0002`, with one vendor-specific interface (class FF).

| Endpoint | Use |
| --- | --- |
| 0x02 OUT | Commands |
| 0x82 IN | Replies and events, in 512-byte packets |
| 0x81 IN | Image, live view and card data |

**Command frame:**
```
04 00 08 05 | u32 (9+n) | 00 | seq&0xff | u32 n | u16 seq | op | u16 code | value
```

**Reply frame:** the same layout, starting `03 00 05 08`.

**Long replies:**
- The first packet carries 260 bytes, with L=255.
- Each continuation packet has a 10-byte header (`03 00 05 08`, u32 cumulative length, `00`, seq low byte), followed by data.
- Reassemble the reply using `n`.

## Ops
| Op | Meaning |
| --- | --- |
| 1 | Action |
| 2 | Read |
| 3 | Write |
| 4 | Indexed read / query |
| 5 | List settings |
| 6 | Describe |
| 7 | Bulk transfer on 0x81 |

**Events on 0x82:**
- `0x40` new image: `04`, token u32, size u32, length-prefixed `"3FR"`.
- `0x42` setting changed: code byte, then a typed value.

## Typed values
| Prefix | Type | Example |
| --- | --- | --- |
| `iN` | Integer | |
| `f` + 16 hex digits | Big-endian double | ISO 200 = `f4069000000000000` |
| `r` + hex of `(num<<16)\|den` | Rational | f/8 = `r80001`, 1/125 = `r1007d` |
| `S<len>,text` | String | |

## Settings
| Code | Setting |
| --- | --- |
| 0x03 | Exposure mode (read-only) |
| 0x07 | EV |
| 0x08 | Aperture |
| 0x09 | Shutter |
| 0x0A | ISO |
| 0x2D | Images in buffer |
| 0x2F | Battery |
| 0x30, 0x46–0x48 | White balance |
| 0x31 | Volumes |
| 0x35 | Live video |
| 0x41 | Drive mode |
| 0x49 | Exposures left |
| 0x7A | Tethered mode |
| 0x94 | Session in progress (polled) |
| 0x95 | Exposure in progress |

## Actions
| Code | Action |
| --- | --- |
| 1 | Release |
| 2 | Autofocus |
| 3 | Focus step (int32) |
| 4 | Session (3 bytes) |
| 7 | Live view (10 bytes; first byte 1 = on, 0 = off) |
| 9 | Free image (token) |

## Connect
The camera only takes pictures for the host after the full connect sequence. Phocus's 147 commands are in `PHOCUS_CONNECT` in `tools/x1d.py` and `index.html`, and they are replayed verbatim.

**Key steps:**
1. Tethered mode `I2,1,1`.
2. Session `000000`.
3. Card listing: op 7 code 9, value `00×10 + u32 2770004`. The camera returns 2,770,432 bytes of length-prefixed paths.
4. White balance writes.
5. Tethered mode `I2,1,0`.
6. Session `010000`.
7. Calibration: op 7 code 2, which returns 1,024 bytes.
8. Write 0x93 `I3,0,1,2`.
9. Write EV back.

**Keep-alive:** after connecting, read 0x94 every 250 ms, and read 0x95 and 0x35 every second. Without the full sequence and the polling, a release is acknowledged but no image is produced, and live view returns ack 21.

**Disconnect:** tethered mode `I2,0,1`, then session `000001`.

## Shooting
1. Read 0x41, then write 0x41 `i0`.
2. Send action 1.
3. Wait for event 0x40, which carries the token and size.
4. Query: op 4 code 9.
5. Copy: op 7 code 4 with value `00 00 + token u32 + 8 zero bytes`.
6. The 3FR arrives on 0x81 in 2 MiB blocks: 52 × 2 MiB + 1,327,104 = 110,379,008 bytes. Read the last block with its exact size.
7. Send action 9 with the token.

The 3FR is TIFF. IFD0 is an uncompressed 1440 × 1080 RGB preview (strip at offset 7680); the 8384 × 6304 raw follows.

## Live view
- **On:** op 4 code 7, then action 7 `01` + 9 zero bytes. If the camera refuses, Phocus first opens the aperture fully (`r20000a`).
- **Stream:** continuous on 0x81 at about 40 MB/s.
- **Frame:** 7,097,344 bytes. A 1,024-byte header, then a 2816 × 1260 16-bit little-endian RGGB Bayer image.
- **Header fields:** `"LIVE"`, u32 2, u32 header size, u32 frame size, u32 width, u32 height, …
- **Row layout:** about 2774 active columns, then 16 optical-black columns, a band of ~65510 markers, then padding.
- **Black level:** about 276, measured from the optical-black columns.
- **Rate and scaling:** about 5–6 fps. Each pixel covers roughly 3 × 5 sensor pixels; stretch to 4:3 for display.
