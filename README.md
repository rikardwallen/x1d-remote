# X1D Remote

A remote for the original **Hasselblad X1D-50c**, run from a browser on an Android phone or a PC over USB. You don't need to install anything.

**Open it:** https://tether.rikardwallen.com (also at https://rikardwallen.github.io/x1d-remote/)

## What it does
- **Shutter:** fire it from the app, with a self-timer, bursts and time-lapse (interval shooting).
- **Exposure:** shutter speed, aperture, ISO and exposure compensation. In A, S and P modes the camera's metered shutter speed is shown.
- **Focus:** autofocus and manual focus steps.
- **Live view** from the camera's own sensor stream:
  - Starts automatically when the camera connects.
  - Focus aids: peaking, a 2×/4× magnifier (tap the picture to magnify that spot), a grid and a histogram.
- **Import:** every shot can be copied to the device as the full 110 MB **3FR RAW**, a JPEG, or both.
  - The JPEG is made from the camera's built-in 1440 × 1080 preview and carries the camera's metadata (date, camera, lens, exposure).
  - Optional direct upload to an **Immich** server.
- **Shots taken on the camera** with its own shutter button while connected are imported too.
- **Card contents:** lists what's on the camera's card.
- **Connection:**
  - Installs as an app from Chrome's menu or the *Install app* button.
  - Keeps the screen awake while connected.
  - Reconnects automatically when the cable is replugged.
- **Battery:** warns when the camera battery runs low.
- **Layout:** for the Fold's inner screen; a half-folded layout is included but untested.

## Use it
1. Connect the camera to the phone or PC with a USB-C cable. Switch it on and close Phocus.
2. Open the page in **Chrome** (Android, Windows, macOS) or **Edge**. Firefox and Safari don't support WebUSB.
3. Tap **Connect camera** and choose the X1D-50c. Connecting takes about 5 seconds.

Before you connect, the page runs a demo camera, so you can try it without one.

**Windows:** Chrome needs the camera on the WinUSB driver. Android and macOS need no driver.

**White balance:** connecting sets white balance to 5000 K manual, the same as Phocus does. The app then puts your camera's own white balance back. You can turn this off under Import.

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
