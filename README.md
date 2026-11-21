# X1D Remote

A remote for the original **Hasselblad X1D-50c**, run from a browser on an Android phone or a PC over USB. You don't need to install anything.

**Open it:** https://tether.rikardwallen.com (also at https://rikardwallen.github.io/x1d-remote/)

## What it does
- Fires the shutter, with a self-timer and bursts.
- Sets shutter speed, aperture and ISO.
- Autofocus, plus manual focus steps.
- Live view from the camera's own sensor stream, at about 5 fps.
- Imports each shot to the device as the full 110 MB **3FR RAW**, a JPEG made from the camera's built-in 1440 × 1080 preview, or both.
- Auto-imports photos taken with the camera's own shutter button while it's connected.
- Lists what's on the camera's card.

## Use it
1. Connect the camera to the phone or PC with a USB-C cable. Switch it on and close Phocus.
2. Open the page in **Chrome** (Android, Windows, macOS) or **Edge**. Firefox and Safari don't support WebUSB.
3. Tap **Connect camera** and choose the X1D-50c. Connecting takes about 5 seconds.

Before you connect, the page runs a demo camera, so you can try it without one.

**Windows:** Chrome needs the camera on the WinUSB driver. Android and macOS need no driver.

**White balance:** connecting sets white balance to 5000 K manual, the same as Phocus does. Change it back on the camera afterwards if you want.

## Command-line tool
`tools/x1d.py` is a Python driver using pyusb with the same features:

```
pip install pyusb libusb-package
python tools/x1d.py --phocus shoot --download .    # fire and copy the 3FR
python tools/x1d.py --phocus lvdump lv.bin 3       # 3 s of raw live view
python tools/x1d.py info
```

## How it works
The X1D-50c speaks a vendor-specific USB protocol, not PTP. See [PROTOCOL.md](PROTOCOL.md). Everything here is host-side and sends only the commands Phocus 4.2 itself sends. There is no firmware modification.

Unofficial; not affiliated with or endorsed by Hasselblad. Use at your own risk.
