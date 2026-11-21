#!/usr/bin/env python3
"""
x1d.py - control a Hasselblad X1D-50c (original X1D) over USB, without Phocus.

Protocol worked out from a USB recording of Phocus 4.2 driving an X1D-50c
(firmware 1.25.0) on 2026-11-21. Every command frame this module builds was
checked byte-for-byte against the 2,348 commands Phocus sent in that recording.

Status of each feature (see the progress report for detail):
  connect / read settings / list / describe ......... decoded, matches recording
  set ISO, aperture, shutter ......................... decoded, matches recording
  fire shutter, autofocus, focus +/- steps ........... decoded, matches recording
  download the photo (3FR) ........................... decoded, matches recording
  camera events (dial turned, photo taken) ........... decoded from recording
  live view .......................................... start/stop decoded; image format NOT yet
NOTHING here has been run against the camera yet. Run `selftest` first (offline).

Setup (Windows / Mac / Linux):
  pip install pyusb libusb-package
  Quit Phocus first - it holds the camera's USB interface.

Usage:
  python x1d.py selftest                 # offline: checks the encoder/decoder against the recording
  python x1d.py sim info                 # offline: same commands against a simulated camera
  python x1d.py info                     # model, firmware, lens, battery, exposure
  python x1d.py get iso | aperture | shutter | battery | mode
  python x1d.py set iso 400
  python x1d.py set aperture 8
  python x1d.py set shutter 1/125
  python x1d.py shoot                    # fire, photo stays on the card
  python x1d.py shoot --download DIR     # fire and pull the 3FR into DIR
  python x1d.py af                       # autofocus
  python x1d.py focus +125               # focus drive steps (+ / -)
  python x1d.py watch [SECONDS]          # print camera events (turn a dial, press the shutter)
  python x1d.py lvdump out.bin 3         # record 3 s of raw live view data (to decode the format)
  python x1d.py dump                     # every setting: value + descriptor
  python x1d.py storage                  # EXPERIMENTAL: try to list the card (read-only)
"""
import os
import struct
import sys
import threading
import time
import queue

VID, PID = 0x2756, 0x0002          # Hasselblad, X1D-50c (X1D II is PID 0x0007, different framing)
EP_CMD_OUT, EP_CMD_IN, EP_DATA_IN = 0x02, 0x82, 0x81
PKT = 512                          # replies always arrive as 512-byte USB packets

# --- operations -------------------------------------------------------------
OP_ACTION, OP_READ, OP_WRITE, OP_INDEXED, OP_LIST, OP_DESCRIBE, OP_BULK = 1, 2, 3, 4, 5, 6, 7

# --- setting codes (op 2 read / op 3 write / op 6 describe) -----------------
P_FOCUS_MODE = 0x0002
P_EXPOSURE_MODE = 0x0003          # 0 M, 1 A, 2 S, 3 P (read-only: the body's dial owns it)
P_EV = 0x0007
P_APERTURE = 0x0008
P_SHUTTER = 0x0009
P_ISO = 0x000A
P_FIRMWARE = 0x001B
P_MODEL = 0x0025
P_SERIAL = 0x0027
P_MAX_BLOCK = 0x0029
P_FOCAL_LENGTH = 0x002E
P_BATTERY = 0x002F
P_LIVE_VIDEO = 0x0035
P_LENS_MOUNTED = 0x003D
P_APERTURE_NAMES = 0x003E
P_SHUTTER_NAMES = 0x003F
P_DRIVE_MODE = 0x0041
P_EXPOSURES_LEFT = 0x0049
P_TETHERED_MODE = 0x007A

# --- action codes (op 1) ----------------------------------------------------
A_RELEASE = 0x0001                # fire the shutter (empty value)
A_AUTOFOCUS = 0x0002              # run AF (empty value)
A_FOCUS_STEP = 0x0003             # move focus: signed int32 steps (Phocus +/- buttons send 125)
A_SESSION = 0x0004                # 3 bytes: 00 00 00 on connect, 00 00 01 on disconnect
A_LIVE_VIEW = 0x0007              # 10 bytes: 01 + 9 zeros = on, 10 zeros = off
A_RELEASE_IMAGE = 0x0009          # uint32 token: camera may drop the image from its tether buffer

EXPOSURE_MODES = {0: "M", 1: "A", 2: "S", 3: "P", 10: "Video"}

# Value tables as your camera reported them (descriptors 0x08, 0x09, 0x0A, 0x3E, 0x3F).
APERTURES = [(0x20, 0xA, "3.2"), (0x22, 0xA, "3.4"), (4, 1, "4"), (0x30, 0xA, "4.8"), (0x38, 0xA, "5.6"),
             (0x44, 0xA, "6.8"), (8, 1, "8"), (0x5F, 0xA, "9.5"), (11, 1, "11"), (13, 1, "13"),
             (16, 1, "16"), (19, 1, "19"), (22, 1, "22"), (27, 1, "27"), (32, 1, "32")]
_SHUTTER_RAW = ("ff00001,b500001,8000001,5a80001,4000001,2d40001,2000001,16a0001,1000001,b50001,800001,"
                "5b0001,400001,2d0001,200001,170001,100001,b0001,80001,60001,40001,30001,20001,e000a,30000,"
                "10000,10001,7000a,5000a,3000a,10004,10006,10008,1000a,1000f,10014,1001e,1002d,1003c,1005a,"
                "1007d,100b4,100fa,1015e,101f4,102ee,103e8,105dc,107d0").split(",")
_SHUTTER_NAMES = ("1:08\t48:16\t34:08\t24:08\t17:04\t12:04\t8:32\t6:02\t4:16\t3:01\t2:08\t1:31\t1:04\t45\t32\t23\t16\t11"
                  "\t8.0\t6.0\t4.0\t3.0\t2.0\t1.4\tT\tB\t1.0\t0.7\t0.5\t0.3\t1/4\t1/6\t1/8\t1/10\t1/15\t1/20\t1/30\t1/45"
                  "\t1/60\t1/90\t1/125\t1/180\t1/250\t1/350\t1/500\t1/750\t1/1000\t1/1500\t1/2000").split("\t")
SHUTTERS = [(int(r, 16), n) for r, n in zip(_SHUTTER_RAW, _SHUTTER_NAMES)]   # (packed rational, label)
ISOS = [100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600]


# =============================================================================
# Framing
# =============================================================================
def encode_frame(seq, op, code=None, value=b""):
    """Host -> camera frame. Verified byte-for-byte against all 2,348 Phocus commands.

    04 00 08 05 | u32 len (= 9 + n) | 00 | seq & 0xff | u32 n | u16 seq | op | [u16 code] | value
    where n = 3 + (2 if code) + len(value)
    """
    body = struct.pack("<HB", seq & 0xFFFF, op)
    if code is not None:
        body += struct.pack("<H", code)
    body += value
    n = len(body)
    return b"\x04\x00\x08\x05" + struct.pack("<I", 9 + n) + bytes([0, seq & 0xFF]) + struct.pack("<I", n) + body


class Reassembler:
    """Camera -> host. Frames start with 03 00 05 08; long ones continue in further
    512-byte packets that carry a 10-byte header (03 00 05 08 | u32 | 00 seq) then data."""

    def __init__(self):
        self.pending = None

    def feed(self, pkt):
        """Feed one USB packet; returns a list of complete (seq, op, payload) frames."""
        out = []
        if len(pkt) < 10 or pkt[:4] != b"\x03\x00\x05\x08":
            return out
        if self.pending is not None:
            need = self.pending["n"] - len(self.pending["buf"])
            self.pending["buf"] += pkt[10:10 + need]
            if len(self.pending["buf"]) >= self.pending["n"]:
                out.append(self._finish(self.pending["buf"]))
                self.pending = None
            return out
        if len(pkt) < 17:
            return out
        length = struct.unpack_from("<I", pkt, 4)[0]
        n = struct.unpack_from("<I", pkt, 10)[0]
        buf = pkt[14:min(len(pkt), 5 + length)]
        if len(buf) < n:
            self.pending = {"n": n, "buf": buf}
        else:
            out.append(self._finish(buf[:n]))
        return out

    @staticmethod
    def _finish(buf):
        seq, op = struct.unpack_from("<HB", buf, 0)
        return seq, op, bytes(buf[3:])


# =============================================================================
# Typed values
# =============================================================================
def decode_typed(b):
    """'i150' -> 150, 'f4059…' -> 100.0, 'r1007d' -> (1, 125), 'S10,abc' -> 'abc'."""
    s = b.decode("latin1") if isinstance(b, (bytes, bytearray)) else b
    if not s:
        return None
    tag, rest = s[0], s[1:]
    try:
        if tag == "i":
            return int(rest)
        if tag == "f":
            return struct.unpack(">d", bytes.fromhex(rest.zfill(16)))[0]
        if tag == "r":
            v = int(rest, 16)
            return (v >> 16, v & 0xFFFF)
        if tag == "S":
            n, _, txt = rest.partition(",")
            return txt
    except ValueError:
        pass
    return s


def enc_int(v):
    return ("i%d" % v).encode()


def enc_double(v):
    return ("f" + struct.pack(">d", float(v)).hex()).encode()


def enc_rational(num, den):
    return ("r%x" % ((num << 16) | den)).encode()


def fmt_shutter(rat):
    if not isinstance(rat, tuple):
        return str(rat)
    packed = (rat[0] << 16) | rat[1]
    for p, name in SHUTTERS:
        if p == packed:
            return name
    if packed == 0x7FFFFFFF:
        return "auto"
    return "%d/%d" % rat


def fmt_aperture(rat):
    if not isinstance(rat, tuple):
        return str(rat)
    for n, d, name in APERTURES:
        if (n, d) == rat:
            return name
    return "%g" % (rat[0] / rat[1]) if rat[1] else "?"


def parse_shutter(text):
    t = text.strip().lower().replace("s", "")
    for p, name in SHUTTERS:
        if name.lower() == t:
            return p >> 16, p & 0xFFFF
    raise ValueError("shutter must be one of: " + ", ".join(n for _, n in SHUTTERS))


def parse_aperture(text):
    t = text.strip().lower().lstrip("f/").strip("/")
    for n, d, name in APERTURES:
        if name == t:
            return n, d
    raise ValueError("aperture must be one of: " + ", ".join(a[2] for a in APERTURES))


# Phocus 4.2's own connect, replayed command for command from the 2026-11-21 20:37 recording
# (147 commands: "op:code:value-hex"; code "-" = none). Used by --phocus.
PHOCUS_CONNECT = (
    "5:-: 3:7a:49322c312c31 1:4:000000 2:11: 6:2: 6:3: 6:4: 6:6: 6:7: 2:17: 6:8: 6:9: 6:a: 6:e: 6:f: 6:10: "
    "6:11: 6:12: 6:13: 6:17: 6:19: 6:1b: 6:1c: 6:24: 6:25: 6:27: 6:28: 6:29: 6:2b: 6:2d: 6:2e: 6:2f: 6:30: "
    "6:33: 6:34: 6:35: 6:3d: 6:43: 6:44: 6:45: 6:46: 6:47: 6:48: 6:49: 2:17: 6:3e: 6:3f: 6:39: 2:40: 2:11: "
    "1:e:3200 2:79: 2:10: 2:e: 4:7: 2:94: 1:e:3200 2:a: 2:8: 2:9: 2:1b: 2:25: 2:27: 4:7: 6:1a: 2:3d: 2:4e: "
    "2:4f: 2:31: 2:29: 7:9:0000000000000000000054442a00 3:46:6937 3:47:6935303030 3:48:6930 2:2: 2:2f: 6:41: "
    "2:41: 2:4e: 2:4f: 2:4e: 2:4f: 2:4e: 2:4f: 2:4e: 2:4f: 2:4e: 2:4f: 2:94: 2:94: 2:95: 2:35: 4:1:07000000 "
    "4:1:07000000 4:1:08000000 2:41: 2:94: 4:1:07000000 4:1:07000000 4:1:08000000 2:41: 2:94: "
    "3:30:49322c353030302c30 3:7a:49322c312c30 1:4:010000 2:16: 2:2b: 7:2:0000010000000000000000040000 "
    "4:7:01 4:1:08000000 4:1:09000000 4:7: 4:7:02 2:89: 2:81: 2:8c: 2:41: 2:8e: 3:93:49332c302c312c32 2:82: "
    "6:8d: 6:91: 6:8f: 6:93: 6:83: 4:1:07000000 4:1:07000000 4:1:08000000 2:41: 2:94: 2:94: 2:95: 2:35: "
    "6:5c: 4:3: 4:2: 4:1:07000000 4:1:08000000 6:2c: 2:3: 4:1:07000000 4:1:07000000 4:1:08000000 2:7: 2:6: "
    "3:7:6662666630303030303030303030303030 2:41:"
)


# =============================================================================
# Transports
# =============================================================================
class UsbTransport:
    def __init__(self):
        import usb.core
        import usb.util
        backend = None
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except Exception:
            pass
        dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
        if dev is None:
            raise SystemExit("X1D-50c not found (USB %04x:%04x). Is it on, plugged in, and Phocus closed?" % (VID, PID))
        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass  # already configured
        usb.util.claim_interface(dev, 0)
        self.dev, self.usb = dev, usb

    def write(self, data):
        self.dev.write(EP_CMD_OUT, data, timeout=2000)

    def read_packet(self, timeout_ms):
        try:
            return bytes(self.dev.read(EP_CMD_IN, PKT, timeout=timeout_ms))
        except self.usb.core.USBTimeoutError:
            return None

    def read_data(self, size, timeout_ms):
        try:
            return bytes(self.dev.read(EP_DATA_IN, size, timeout=timeout_ms))
        except self.usb.core.USBTimeoutError:
            return b""

    def close(self):
        try:
            self.usb.util.release_interface(self.dev, 0)
            self.usb.util.dispose_resources(self.dev)
        except Exception:
            pass


class SimTransport:
    """A pretend X1D-50c that answers like the real one did in the recording."""

    def __init__(self):
        self.out = queue.Queue()
        self.data = queue.Queue()
        self.vals = {P_FOCUS_MODE: b"i0", P_EXPOSURE_MODE: b"i0", P_APERTURE: b"r80001", P_SHUTTER: b"r1005a",
                     P_ISO: b"f4059000000000000", P_FIRMWARE: b"S10,v1.25.0-42", P_MODEL: b"S18,Hasselblad X1D-50c",
                     P_SERIAL: b"S10,SIMULATED0", P_FOCAL_LENGTH: b"i90", P_BATTERY: b"i41", P_LENS_MOUNTED: b"i1",
                     P_LIVE_VIDEO: b"i0", P_EXPOSURES_LEFT: b"i8", P_MAX_BLOCK: b"i8388608", P_DRIVE_MODE: b"i0"}
        self.desc = {P_APERTURE: b"\x01\x01LR15," + ",".join("%x" % ((n << 16) | d) for n, d, _ in APERTURES).encode(),
                     P_SHUTTER: b"\x01\x01LR49," + ",".join(_SHUTTER_RAW).encode(),
                     P_ISO: b"\x01\x01LF9," + b",".join(enc_double(i)[1:] for i in ISOS)}
        self.token, self.evseq = 0x0610, 100
        self.image = b""

    def _reply(self, seq, op, payload):
        for pkt in self._packets(seq, op, payload):
            self.out.put(pkt)

    @staticmethod
    def _packets(seq, op, payload):
        body = struct.pack("<HB", seq & 0xFFFF, op) + payload
        n = len(body)
        frame = b"\x03\x00\x05\x08" + struct.pack("<I", 9 + n) + bytes([0, seq & 0xFF]) + struct.pack("<I", n) + body
        if len(frame) <= 260:
            return [frame.ljust(PKT, b"\0")]
        first = b"\x03\x00\x05\x08" + struct.pack("<I", 255) + frame[8:260]
        rest = frame[260:]
        pk = [first.ljust(PKT, b"\0")]
        sent = 260
        while rest:
            chunk, rest = rest[:PKT - 10], rest[PKT - 10:]
            sent += len(chunk)
            pk.append((b"\x03\x00\x05\x08" + struct.pack("<I", sent) + bytes([0, seq & 0xFF]) + chunk).ljust(PKT, b"\0"))
        return pk

    def _event(self, op, payload):
        self.evseq += 1
        self._reply(self.evseq, op, payload)

    def write(self, data):
        n = struct.unpack_from("<I", data, 10)[0]
        seq, op = struct.unpack_from("<HB", data, 14)
        code = struct.unpack_from("<H", data, 17)[0] if n >= 5 else None
        val = data[19:14 + n]
        if op == OP_LIST:
            self._reply(seq, 0x85, b"".join(struct.pack("<H", c) for c in sorted(self.vals)))
        elif op == OP_READ:
            self._reply(seq, 0x82, self.vals.get(code, b""))
        elif op == OP_DESCRIBE:
            self._reply(seq, 0x86, self.desc.get(code, b"\x01\x00MLi0,0,0"))
        elif op == OP_WRITE:
            if code in self.vals or code in (P_TETHERED_MODE,):
                self.vals[code] = val
                if code in (P_ISO, P_APERTURE, P_SHUTTER):
                    self._event(0x42, bytes([code]) + val)
            self._reply(seq, 0x83, struct.pack("<I", 13 if code in (P_TETHERED_MODE, P_DRIVE_MODE) else 0))
        elif op == OP_ACTION:
            self._reply(seq, 0x81, struct.pack("<I", 0))
            if code == A_RELEASE:
                self.token += 1
                self.image = self._fake_3fr()
                self._event(0x40, b"\x04" + struct.pack("<II", self.token, len(self.image)) + b"\x033FR\x00")
            elif code == A_LIVE_VIEW:
                self.vals[P_LIVE_VIDEO] = b"i1" if val[:1] == b"\x01" else b"i0"
                self._event(0x42, bytes([P_LIVE_VIDEO]) + self.vals[P_LIVE_VIDEO])
        elif op == OP_BULK:
            self._reply(seq, 0x87, struct.pack("<I", 0))
            if code == 0x0004:
                for i in range(0, len(self.image), 2 << 20):
                    self.data.put(self.image[i:i + (2 << 20)])
            elif code == 0x0009:
                names = [b"/DCIM/100HBLAD/B0000%03d.3FR" % i for i in range(139, 154)]
                self.data.put(b"\x00" * 25 + b"".join(bytes([len(n)]) + n for n in names))
        else:
            self._reply(seq, op | 0x80, struct.pack("<I", 0))

    @staticmethod
    def _fake_3fr():
        hdr = b"II*\x00" + struct.pack("<I", 8) + struct.pack("<H", 1) + struct.pack("<HHII", 273, 4, 1, 26) + b"\0\0\0\0"
        return hdr + b"SIMULATED 3FR " * 64

    def read_packet(self, timeout_ms):
        try:
            return self.out.get(timeout=timeout_ms / 1000)
        except queue.Empty:
            return None

    def read_data(self, size, timeout_ms):
        try:
            return self.data.get(timeout=timeout_ms / 1000)[:size]
        except queue.Empty:
            return b""

    def close(self):
        pass


# =============================================================================
# Camera
# =============================================================================
class CameraError(Exception):
    pass


class X1D:
    def __init__(self, transport):
        self.t = transport
        self.seq = 0
        self.lock = threading.Lock()
        self.asm = Reassembler()
        self.events = queue.Queue()
        self.stash = {}
        self.verbose = False
        self.settle = 0

    # --- low level ---
    def call(self, op, code=None, value=b"", timeout=3.0):
        with self.lock:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFFFF
            self.t.write(encode_frame(seq, op, code, value))
            deadline = time.time() + timeout
            while time.time() < deadline:
                if seq in self.stash:
                    return self.stash.pop(seq)
                pkt = self.t.read_packet(int(max(50, (deadline - time.time()) * 1000)))
                if pkt is None:
                    continue
                for rseq, rop, payload in self.asm.feed(pkt):
                    if 0x40 <= rop <= 0x4F:
                        self.events.put(self._event(rop, payload))
                    elif rop == (op | 0x80) and rseq == seq:
                        return payload
                    else:
                        self.stash[rseq] = payload
            raise CameraError("no reply to op %d code %s" % (op, code))

    def poll_events(self, timeout=0.2):
        """Read any pending camera events (call when idle)."""
        with self.lock:
            pkt = self.t.read_packet(int(timeout * 1000))
            if pkt:
                for rseq, rop, payload in self.asm.feed(pkt):
                    if 0x40 <= rop <= 0x4F:
                        self.events.put(self._event(rop, payload))
        out = []
        while not self.events.empty():
            out.append(self.events.get())
        return out

    @staticmethod
    def _event(op, payload):
        if op == 0x40 and len(payload) >= 9:      # new image: 04 | token u32 | size u32 | len "3FR" 00
            token, size = struct.unpack_from("<II", payload, 1)
            n = payload[9] if len(payload) > 9 else 0
            fmt = payload[10:10 + n].decode("latin1", "replace")
            return {"type": "image", "token": token, "size": size, "format": fmt}
        if op == 0x42 and payload:                # setting changed: code byte | typed value
            return {"type": "changed", "code": payload[0], "value": decode_typed(payload[1:]), "raw": payload[1:]}
        return {"type": "event", "op": op, "raw": payload}

    # --- settings ---
    def list_codes(self):
        p = self.call(OP_LIST)
        return list(struct.unpack("<%dH" % (len(p) // 2), p[:len(p) // 2 * 2]))

    def read_raw(self, code):
        return self.call(OP_READ, code)

    def read(self, code):
        return decode_typed(self.read_raw(code))

    def describe(self, code):
        p = self.call(OP_DESCRIBE, code)
        return p[2:].decode("latin1") if len(p) > 2 else ""

    def write(self, code, value_bytes):
        ack = self.call(OP_WRITE, code, value_bytes)
        return struct.unpack_from("<I", ack)[0] if len(ack) >= 4 else None

    def action(self, code, value=b""):
        ack = self.call(OP_ACTION, code, value)
        return struct.unpack_from("<I", ack)[0] if len(ack) >= 4 else None

    # --- session ---
    def connect(self, full=False, verbose=False):
        """Phocus's connect sequence. Part 1: list settings, tethered mode on, session start.
        Part 2 (full=True), as Phocus did ~8 s later: tethered mode I2,1,0, session 01 00 00,
        the 1 KB calibration block (op 7 code 2), setting 0x93 = I3,0,1,2. Phocus's white-balance
        write is left out on purpose (it would change the camera's setting)."""
        self.codes = self.list_codes()
        steps = [("tethered I2,1,1", self.write(P_TETHERED_MODE, b"I2,1,1")),
                 ("session 000000", self.action(A_SESSION, b"\x00\x00\x00"))]
        if full and self.settle:
            # Phocus spent ~8 s reading settings here before part 2
            end = time.time() + self.settle
            while time.time() < end:
                for c in (P_DRIVE_MODE, P_LIVE_VIDEO, P_EXPOSURES_LEFT, 0x002D):
                    try:
                        self.read_raw(c)
                    except CameraError:
                        pass
                for ev in self.poll_events(0.2):
                    if verbose:
                        print("  settle event", ev, flush=True)
        if full:
            steps.append(("tethered I2,1,0", self.write(P_TETHERED_MODE, b"I2,1,0")))
            steps.append(("session 010000", self.action(A_SESSION, b"\x01\x00\x00")))
            try:
                ack = self.call(OP_BULK, 0x0002, bytes.fromhex("0000010000000000000000040000"))
                blob = self.t.read_data(1024, 1500)
                steps.append(("calibration block", "ack %s, %d bytes" % (ack.hex(), len(blob))))
            except CameraError as e:
                steps.append(("calibration block", "error %s" % e))
            steps.append(("0x93 I3,0,1,2", self.write(0x0093, b"I3,0,1,2")))
        self.connect_log = steps
        if verbose:
            for name, ack in steps:
                print("connect: %-18s ack %s" % (name, ack), flush=True)
        return self

    # --- Phocus replay mode (--phocus) ---
    def connect_phocus(self, verbose=False):
        """Send exactly what Phocus 4.2 sent on connect (recorded 2026-11-21), then keep the
        camera polled every 250 ms like Phocus does. Two values are not copied blindly:
        the exposure-compensation write uses the value just read from the camera (as Phocus does),
        and the data pipe is drained after the card listing (op 7 code 9) and calibration block (op 7 code 2)."""
        last = {}
        log = []
        self.codes = []
        for item in PHOCUS_CONNECT.split():
            op, code, val = item.split(":")
            op = int(op); code = None if code == "-" else int(code, 16); val = bytes.fromhex(val)
            if op == OP_WRITE and code == P_EV:
                val = last.get(P_EV) or val
            try:
                reply = self.call(op, code, val)
            except CameraError as e:
                log.append((op, code, "no reply"))
                continue
            if op == OP_LIST:
                self.codes = list(struct.unpack("<%dH" % (len(reply) // 2), reply[:len(reply) // 2 * 2]))
            if op == OP_READ:
                last[code] = reply
            if op in (OP_ACTION, OP_WRITE, OP_INDEXED, OP_BULK):
                ack = struct.unpack_from("<I", reply)[0] if len(reply) >= 4 else None
                log.append((op, code, ack))
            if op == OP_BULK and code == 0x0009:
                got = 0
                while True:
                    chunk = self.t.read_data(2 << 20, 1500)
                    if not chunk:
                        break
                    got += len(chunk)
                log.append((op, code, "drained %d bytes" % got))
            if op == OP_BULK and code == 0x0002:
                log.append((op, code, "calibration %d bytes" % len(self.t.read_data(1024, 1500))))
        self.connect_log = log
        if verbose:
            names = {1: "action", 3: "write", 4: "query", 7: "bulk"}
            for op, code, ack in log:
                print("phocus-connect: %-6s %#06x -> %s" % (names.get(op, op), code or 0, ack), flush=True)
        self.start_keepalive()
        return self

    def start_keepalive(self):
        self._ka_stop = threading.Event()

        def run():
            n = 0
            while not self._ka_stop.wait(0.25):
                try:
                    self.read_raw(0x0094)
                    n += 1
                    if n % 4 == 0:
                        self.read_raw(0x0095)
                        self.read_raw(P_LIVE_VIDEO)
                except Exception:
                    pass
        self._ka = threading.Thread(target=run, daemon=True)
        self._ka.start()

    def stop_keepalive(self):
        if getattr(self, "_ka_stop", None):
            self._ka_stop.set()
            self._ka.join(2)
            self._ka_stop = None

    def disconnect_phocus(self):
        """Phocus's exit: tethered mode I2,0,1 then session 00 00 01 (Phocus sends the pair 4 times)."""
        self.stop_keepalive()
        try:
            for _ in range(2):
                self.write(P_TETHERED_MODE, b"I2,0,1")
                self.action(A_SESSION, b"\x00\x00\x01")
        finally:
            self.t.close()

    def disconnect(self):
        try:
            self.write(P_TETHERED_MODE, b"I2,0,1")
            self.action(A_SESSION, b"\x00\x00\x01")
        finally:
            self.t.close()

    # --- friendly ---
    def info(self):
        return {
            "model": self.read(P_MODEL), "firmware": self.read(P_FIRMWARE), "serial": self.read(P_SERIAL),
            "battery_pct": self.read(P_BATTERY), "lens_mounted": self.read(P_LENS_MOUNTED),
            "mode": EXPOSURE_MODES.get(self.read(P_EXPOSURE_MODE), "?"),
            "iso": self.read(P_ISO), "aperture": fmt_aperture(self.read(P_APERTURE)),
            "shutter": fmt_shutter(self.read(P_SHUTTER)),
        }

    def set_iso(self, iso):
        if int(iso) not in ISOS:
            raise ValueError("ISO must be one of %s" % ISOS)
        return self.write(P_ISO, enc_double(int(iso)))

    def set_aperture(self, text):
        return self.write(P_APERTURE, enc_rational(*parse_aperture(str(text))))

    def set_shutter(self, text):
        return self.write(P_SHUTTER, enc_rational(*parse_shutter(str(text))))

    def autofocus(self):
        return self.action(A_AUTOFOCUS)

    def focus_step(self, steps):
        return self.action(A_FOCUS_STEP, struct.pack("<i", int(steps)))

    def live_view(self, on):
        return self.action(A_LIVE_VIEW, (b"\x01" if on else b"\x00") + b"\x00" * 9)

    def shoot(self, download_dir=None, wait=20.0):
        """Fire the shutter exactly as Phocus does; optionally download the 3FR."""
        try:
            self.read_raw(P_DRIVE_MODE)            # Phocus reads drive mode first
        except CameraError:
            pass
        a1 = self.write(P_DRIVE_MODE, b"i0")
        a2 = self.action(A_RELEASE)
        if self.verbose:
            print("drive mode ack %s, release ack %s" % (a1, a2), flush=True)
        ev = self.wait_for_image(wait)
        if ev is None:
            raise CameraError("no photo-ready event (drive mode ack %s, release ack %s)" % (a1, a2))
        path = None
        if download_dir:
            try:
                self.call(OP_INDEXED, A_RELEASE_IMAGE)   # Phocus queries action 9 before each download
            except CameraError:
                pass
            path = os.path.join(download_dir, "X1D_%04X.3FR" % ev["token"])
            self.download(ev["token"], ev["size"], path)
        # Phocus always releases the image after downloading. Without a download this
        # step is untested: the photo is written to the card either way (to confirm).
        self.action(A_RELEASE_IMAGE, struct.pack("<I", ev["token"]))
        return ev, path

    def wait_for_image(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            for ev in self.poll_events(0.3):
                if self.verbose:
                    print("  event", ev, flush=True)
                if ev["type"] == "image":
                    return ev
        return None

    def storage(self, raw_path=None, cap=8 << 20):
        """EXPERIMENTAL: ask the camera for its card listing, the way the X1D II does
        (op 7, code 9). Untested on the X1D-50c. Read-only: it asks for data, changes nothing.
        Returns (ack, paths, blob)."""
        ack = self.call(OP_BULK, 0x0009, b"\x00" * 10 + struct.pack("<I", 2770004))
        code = struct.unpack_from("<I", ack)[0] if len(ack) >= 4 else None
        blob = b""
        if code == 0:
            while len(blob) < cap:
                chunk = self.t.read_data(1 << 20, 1500)
                if not chunk:
                    break
                blob += chunk
        if raw_path and blob:
            with open(raw_path, "wb") as f:
                f.write(blob)
        paths, i = [], 0
        while i < len(blob) - 1:
            n = blob[i]
            if 3 <= n <= 120 and blob[i + 1:i + 2] == b"/":
                chunk = blob[i + 1:i + 1 + n]
                if all(32 <= c < 127 for c in chunk):
                    paths.append(chunk.decode())
                    i += 1 + n
                    continue
            i += 1
        return code, paths, blob

    def download(self, token, size, path, progress=None):
        self.call(OP_BULK, 0x0004, b"\x00\x00" + struct.pack("<I", token) + b"\x00" * 8)
        got = 0
        with open(path, "wb") as f:
            while got < size:
                chunk = self.t.read_data(min(2 << 20, size - got if size - got > 0 else 2 << 20), 5000)
                if not chunk:
                    break
                if got == 0:                       # resync on the TIFF header if a stray block came first
                    i = chunk.find(b"II*\x00")
                    if i > 0:
                        chunk = chunk[i:]
                f.write(chunk)
                got += len(chunk)
                if progress:
                    progress(got, size)
        return got


# =============================================================================
# Self-test against the recording (offline)
# =============================================================================
# 20 real frames from the 2026-11-21 recording: (hex as Phocus sent it, seq, op, code, value hex)
RECORDED_COMMANDS = [
    ("040008050c000000000003000000000005", 0, 5, None, ""),
    ("04000805140000000001" "0b0000000100037a0049322c312c31", 1, 3, 0x7A, b"I2,1,1".hex()),
    ("04000805110000000002080000000200010400000000", 2, 1, 0x04, "000000"),
    ("040008050e0000000003050000000300021100", 3, 2, 0x11, ""),
]


def selftest():
    ok = True

    def check(name, cond):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name)
        ok &= bool(cond)

    for hx, seq, op, code, val in RECORDED_COMMANDS:
        check("encode seq %d op %d" % (seq, op), encode_frame(seq, op, code, bytes.fromhex(val)).hex() == hx)
    # writes seen in the recording, rebuilt from human values
    check("ISO 200 -> f4069000000000000", enc_double(200) == b"f4069000000000000")
    check("ISO 100 -> f4059000000000000", enc_double(100) == b"f4059000000000000")
    check("f/8 -> r80001", enc_rational(*parse_aperture("8")) == b"r80001")
    check("f/3.2 -> r20000a", enc_rational(*parse_aperture("3.2")) == b"r20000a")
    check("1/125 -> r1007d", enc_rational(*parse_shutter("1/125")) == b"r1007d")
    check("1/90 -> r1005a", enc_rational(*parse_shutter("1/90")) == b"r1005a")
    check("focus +125 value", struct.pack("<i", 125).hex() == "7d000000")
    check("focus -125 value", struct.pack("<i", -125).hex() == "83ffffff")
    # replies and events taken from the recording
    r = Reassembler()
    check("reply: body type read", r.feed(bytes.fromhex("030005081000000000090700000009008269313530"))
          == [(9, 0x82, b"i150")])
    ev = X1D._event(0x40, bytes.fromhex("040e060000004094060333465200"))
    check("event: image token 0x060e, 110,379,008 bytes, 3FR",
          ev == {"type": "image", "token": 0x060E, "size": 110379008, "format": "3FR"})
    ev = X1D._event(0x42, bytes.fromhex("09723130303561"))
    check("event: shutter changed to 1/90", ev["code"] == P_SHUTTER and fmt_shutter(ev["value"]) == "1/90")
    # split reply (shutter descriptor) as the camera sent it: first packet then a continuation
    p1 = bytes.fromhex("03000508ff000000000b490100000b008601014c5234392c") + b"x" * 236
    p1 = p1[:260]
    tail_len = 0x149 - (260 - 14)
    p2 = bytes.fromhex("0300050858010000000b") + b"y" * tail_len
    r = Reassembler()
    got = r.feed(p1) + r.feed(p2)
    check("reassembles a split reply to 329 bytes", len(got) == 1 and len(got[0][2]) == 0x149 - 3)
    # full round trip against the simulator
    cam = X1D(SimTransport()).connect()
    inf = cam.info()
    check("sim: info", inf["model"] == "Hasselblad X1D-50c" and inf["shutter"] == "1/90" and inf["aperture"] == "8")
    cam.set_iso(400)
    check("sim: set ISO 400", cam.read(P_ISO) == 400.0)
    ev, _ = cam.shoot()
    check("sim: shoot -> image event", ev["type"] == "image" and ev["format"] == "3FR")
    ack, paths, _ = cam.storage()
    check("sim: storage listing parses 15 files", ack == 0 and len(paths) == 15 and paths[0].endswith("B0000139.3FR"))
    print("\nALL PASS" if ok else "\nSOME TESTS FAILED")
    return ok


# =============================================================================
# CLI
# =============================================================================
def main(argv):
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    if argv[0] == "selftest":
        return 0 if selftest() else 1
    sim = argv[0] == "sim"
    if sim:
        argv = argv[1:]
    verbose = "-v" in argv
    argv = [a for a in argv if a != "-v"]
    phocus = "--phocus" in argv
    argv = [a for a in argv if a != "--phocus"]
    settle = 0
    if "--settle" in argv:
        i = argv.index("--settle"); settle = float(argv[i + 1]); del argv[i:i + 2]
    cmd, args = argv[0], argv[1:]
    cam = X1D(SimTransport() if sim else UsbTransport())
    cam.verbose = verbose
    cam.settle = settle
    if phocus:
        cam.connect_phocus(verbose=verbose)
    else:
        cam.connect(verbose=verbose)
    if verbose:
        for c, name in ((P_EXPOSURES_LEFT, "exposures left"), (0x002D, "images in buffer"), (0x0031, "volumes"), (P_LIVE_VIDEO, "live video")):
            try:
                print("state: %-16s %r" % (name, cam.read_raw(c)), flush=True)
            except CameraError as e:
                print("state: %-16s error %s" % (name, e), flush=True)
    try:
        if cmd == "info":
            for k, v in cam.info().items():
                print("%-13s %s" % (k, v))
        elif cmd == "get":
            what = args[0]
            code = {"iso": P_ISO, "aperture": P_APERTURE, "shutter": P_SHUTTER, "battery": P_BATTERY,
                    "mode": P_EXPOSURE_MODE}.get(what)
            code = code if code is not None else int(what, 0)
            v = cam.read(code)
            print(fmt_shutter(v) if code == P_SHUTTER else fmt_aperture(v) if code == P_APERTURE else v)
        elif cmd == "set":
            what, val = args[0], args[1]
            ack = {"iso": cam.set_iso, "aperture": cam.set_aperture, "shutter": cam.set_shutter}[what](val)
            print("ack", ack)
        elif cmd == "shoot":
            d = args[args.index("--download") + 1] if "--download" in args else None
            ev, path = cam.shoot(download_dir=d)
            print("photo taken: token %#06x, %d bytes %s" % (ev["token"], ev["size"], ev["format"]))
            if path:
                print("saved", path)
        elif cmd == "af":
            print("ack", cam.autofocus())
        elif cmd == "focus":
            print("ack", cam.focus_step(int(args[0])))
        elif cmd == "watch":
            secs = float(args[0]) if args else None
            print("Watching camera events - turn a dial or press the shutter." + (" (%g s)" % secs if secs else " Ctrl+C to stop."), flush=True)
            end = time.time() + secs if secs else None
            while end is None or time.time() < end:
                for ev in cam.poll_events(0.5):
                    print(time.strftime("%H:%M:%S"), ev, flush=True)
                    if ev.get("type") == "image":    # a photo taken on the body: release it as Phocus does
                        cam.action(A_RELEASE_IMAGE, struct.pack("<I", ev["token"]))
        elif cmd == "lvdump":
            path, secs = args[0], float(args[1]) if len(args) > 1 else 3
            old_ap = None
            if phocus:                                   # Phocus: query action 7, open the lens up, then live view on
                try:
                    print("live view query ack", cam.call(OP_INDEXED, A_LIVE_VIEW).hex(), flush=True)
                    old_ap = cam.read_raw(P_APERTURE)
                    print("aperture", old_ap, "-> r20000a, ack", cam.write(P_APERTURE, b"r20000a"), flush=True)
                except CameraError as e:
                    print("live view prep error", e, flush=True)
            ack = cam.live_view(True)
            print("live view on, ack", ack, flush=True)
            end, total = time.time() + secs, 0
            with open(path, "wb") as f:
                while time.time() < end:
                    chunk = cam.t.read_data(1 << 20, 2000)
                    if chunk:
                        f.write(struct.pack("<dI", time.time(), len(chunk)) + chunk)
                        total += len(chunk)
            print("live view off, ack", cam.live_view(False), flush=True)
            if old_ap:
                print("aperture restored, ack", cam.write(P_APERTURE, old_ap), flush=True)
            print("wrote %d bytes of live view data to %s" % (total, path))
        elif cmd == "storage":
            raw = args[0] if args else "x1d_storage.bin"
            ack, paths, blob = cam.storage(raw_path=raw)
            print("camera ack:", ack, "| bytes received:", len(blob), "| saved to", raw if blob else "(nothing)")
            print("files found:", len(paths))
            for pth in paths[:40]:
                print("  ", pth)
            if len(paths) > 40:
                print("   ... and", len(paths) - 40, "more")
        elif cmd == "dump":
            for c in cam.codes:
                try:
                    print("%#06x  %-30r %s" % (c, cam.read_raw(c), cam.describe(c)[:80]))
                except CameraError as e:
                    print("%#06x  error %s" % (c, e))
        else:
            print("unknown command", cmd)
            return 2
    finally:
        if phocus:
            cam.disconnect_phocus()
        else:
            cam.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
