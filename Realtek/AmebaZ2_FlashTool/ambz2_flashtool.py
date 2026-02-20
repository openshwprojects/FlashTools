#!/usr/bin/env python3
# AmebaZ2 (RTL8720C family) flashtool: read / write / erase via UART bootrom
# Mirrors the protocol used by ltchiptool's AmebaZ2 support.
# Dependencies: pyserial, xmodem

import argparse
import sys
import time
import os
from hashlib import sha256

import serial
from xmodem import XMODEM

# ==========
# Constants
# ==========
FALLBACK_CMD = b"Rtk8710C\n"
FALLBACK_RESP = [
    b"\r\n$8710c>" * 2,
    b"Rtk8710C\r\nCommand NOT found.\r\n$8710c>",
]

# Memory map constants
FLASH_MMAP_BASE = 0x98000000

# Registers
REG_BOOT_FLAGS = 0x40000038
REG_FLASH_CTRL = 0x40002800
REG_ROM_CMD_ARRAY_PTR = 0x1002F050 + 4
REG_CHIP_VER = 0x400001F0

# Hash computation throughput estimate (bytes/s)
CRC_SPEED_BPS = 1_500_000

# Defaults
DEFAULT_LINK_BAUD = 115200
DEFAULT_FLASH_SIZE = 0x400000  # 4 MiB

USED_COMMANDS = {
    "ping","disc","ucfg","DW","DB","EW","EB","WDTRST","hashq","fwd","fwdram"
}

def _getc_factory(ser: serial.Serial):
    def _getc(size, timeout=1):
        data = ser.read(size)
        return data if data else None
    return _getc

def _putc_factory(ser: serial.Serial):
    def _putc(data, timeout=1):
        return ser.write(data)
    return _putc

class AmebaZ2Serial:
    def __init__(self, port, baudrate=DEFAULT_LINK_BAUD, read_timeout=0.6, link_timeout=10.0, retry_count=10, verbose=False):
        self.s = serial.Serial(port=port, baudrate=baudrate, timeout=read_timeout, write_timeout=1.0)
        self.default_timeout = read_timeout
        self.link_timeout = link_timeout
        self.retry_count = retry_count
        self.verbose = verbose
        self._timeout_stack = []
        self.in_fallback_mode = False
        self.flash_mode = None
        self.flash_speed = 0  # SINGLE
        self.flash_hash_offset = None
        self.xm = XMODEM(getc=_getc_factory(self.s), putc=_putc_factory(self.s), mode='xmodem1k')

    # ---- utilities ----
    def _log(self, msg):
        if self.verbose:
            print(msg, file=sys.stderr)

    def set_baudrate(self, baudrate: int):
        self.s.baudrate = baudrate

    def push_timeout(self, t: float):
        self._timeout_stack.append(self.s.timeout)
        self.s.timeout = t

    def pop_timeout(self):
        if self._timeout_stack:
            self.s.timeout = self._timeout_stack.pop()
        else:
            self.s.timeout = self.default_timeout

    def flush(self):
        try:
            self.s.reset_input_buffer()
            self.s.reset_output_buffer()
        except Exception:
            pass

    def read(self, count: int = None) -> bytes:
        if count is None:
            # drain until timeout
            buf = bytearray()
            while True:
                b = self.s.read(1)
                if not b:
                    break
                buf += b
            return bytes(buf)
        data = self.s.read(count)
        if len(data) < (count or 0):
            raise TimeoutError("Timed out reading from serial")
        return data

    def write(self, data: bytes):
        self.s.write(data)
        self.s.flush()

    def read_all(self) -> bytes:
        time.sleep(0.02)
        try:
            n = self.s.in_waiting
        except Exception:
            n = 0
        if n:
            return self.s.read(n)
        return b''

    def readlines(self):
        """Yield lines ending with '\\n' until quiet for ~timeout."""
        buf = bytearray()
        idle_start = None
        while True:
            b = self.s.read(1)
            if b:
                buf += b
                idle_start = None
                if b == b'\n':
                    line = bytes(buf).decode(errors='ignore').strip()
                    yield line
                    buf.clear()
            else:
                # start idle timer
                if idle_start is None:
                    idle_start = time.time()
                # break if idle exceeds current read timeout (bounded)
                if time.time() - idle_start > max(0.3, float(self.s.timeout or 0.6)):
                    if buf:
                        # return any trailing partial line as last line
                        try:
                            yield bytes(buf).decode(errors='ignore').strip()
                        except Exception:
                            pass
                    break

    # --------------------------
    # Basic ROM console commands
    # --------------------------
    def command(self, cmd: str):
        self.flush()
        self._log(f">>> {cmd}")
        self.write(cmd.encode('ascii') + b'\n')
        if self.in_fallback_mode:
            # consume echo in fallback shell
            _ = self.s.read(len(cmd) + 2)

    def ping(self):
        self.command("ping")
        resp = self.read(4)
        if resp != b"ping":
            raise RuntimeError(f"Incorrect ping response: {resp!r}")
        # detect fallback shell prompt present
        extra = self.read_all()
        if b"$8710c" in extra:
            raise RuntimeError(f"Got fallback mode ping: {extra!r}")

    def disconnect(self):
        self.command("disc")

    def link(self):
        """Ensure device is in download mode and responsive to 'ping'."""
        self.link_fallback()
        end = time.time() + self.link_timeout
        while time.time() < end:
            try:
                self.ping()
                return
            except (RuntimeError, TimeoutError):
                pass
        raise TimeoutError("Timeout while linking (device not answering 'ping')")

    def link_fallback(self):
        """If ROM boots into monitor ($8710c), jump to download mode."""
        self.flush()
        self.write(FALLBACK_CMD)
        self.push_timeout(0.1)
        try:
            response = self.read()
            if response not in FALLBACK_RESP:
                return
        except TimeoutError:
            return
        finally:
            self.pop_timeout()
        self._log(f"[fallback] Found fallback monitor: {response!r}")
        self.in_fallback_mode = True
        # Check ROM version to choose jump address
        chip_ver = (self.register_read(REG_CHIP_VER) >> 4) & 0xF
        jump = 0x0 if chip_ver > 2 else 0x1443C
        self.memory_boot(jump)
        self.in_fallback_mode = False

    def change_baud(self, baudrate: int):
        """Ask ROM to switch baud; then we switch host and expect 'OK'."""
        self.command(f"ucfg {baudrate} 0 0")
        self.push_timeout(1.0)
        try:
            resp = self.read()
        except TimeoutError:
            raise RuntimeError("Timed out while changing baud rate")
        finally:
            self.pop_timeout()
        if not resp or not resp.lstrip().startswith(b"OK"):
            raise RuntimeError(f"Baud rate change not OK: {resp!r}")

        # Switch host side after ROM acknowledgement and clear any residual data
        self.set_baudrate(baudrate)
        self.flush()
        # relink to confirm
        self.link()

    # --------------------------
    # Dump & register primitives
    # --------------------------
    def dump_words(self, start: int, count: int):
        """Yield lists of 4 words printed per line by 'DW'."""
        # set timeout based on expected output rate
        self.push_timeout(max(min(count, 256), 16) * 1.5 / 500.0)
        try:
            # increase buffer on Windows if available
            if hasattr(self.s, "set_buffer_size"):
                # each line ~57 chars, holds 4 words
                self.s.set_buffer_size(rx_size=min(32768, 57 * max(1, count // 4)))
        except Exception:
            pass
        read_count = 0
        self.flush()
        self.command(f"DW {start:X} {count}")
        for line in self.readlines():
            parts = line.split()
            if not parts:
                continue
            try:
                addr = int(parts[0].rstrip(':'), 16)
            except Exception:
                continue
            if addr != start + read_count:
                raise ValueError("Got invalid read address")
            if len(parts) < 5:
                raise ValueError(f"Not enough data in line {line!r}")
            chunk = []
            for tok in parts[1:1+4]:
                try:
                    val = int(tok, 16)
                except Exception:
                    continue
                chunk.append(val)
                read_count += 4
                if read_count >= count:
                    break
            yield chunk
            if read_count >= count:
                break
        self.pop_timeout()

    def dump_bytes(self, start: int, count: int):
        """Yield blocks of up to 16 bytes as printed by 'DB'."""
        self.push_timeout(max(min(count, 1024), 64) * 0.5 / 500.0)
        try:
            if hasattr(self.s, "set_buffer_size"):
                # each line ~78 chars, holds 16 bytes
                self.s.set_buffer_size(rx_size=min(32768, 78 * max(1, count // 16)))
        except Exception:
            pass
        read_count = 0
        self.flush()
        self.command(f"DB {start:X} {count}")
        for line in self.readlines():
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "[Addr]":
                # header line
                continue
            try:
                addr = int(parts[0].rstrip(':'), 16)
            except Exception:
                continue
            if addr != start + read_count:
                raise ValueError("Got invalid read address")
            if len(parts) < 17:
                raise ValueError(f"Not enough data in line {line!r}")
            chunk = bytearray()
            for tok in parts[1:1+16]:
                try:
                    val = int(tok, 16)
                except Exception:
                    continue
                chunk.append(val)
                read_count += 1
                if read_count >= count:
                    break
            if chunk:
                yield bytes(chunk)
            if read_count >= count:
                break
        self.pop_timeout()

    def register_read(self, address: int) -> int:
        start = address & ~0x3
        regs = list(self.dump_words(start=start, count=4))
        return regs[0][(address - start)]

    def register_write(self, address: int, value: int):
        self.command(f"EW {address:X} {value:X}")
        # consume one response line
        for _ in self.readlines():
            break

    def register_read_bytes(self, address: int, length: int) -> bytes:
        start = address & ~0x3
        buff = bytearray()
        for blk in self.dump_bytes(start, length):
            buff += blk
        return bytes(buff)[:length]

    def register_write_bytes(self, address: int, data: bytes):
        start = address & ~0x3
        pad = (4 - (len(data) % 4)) % 4
        data = data + b"\x00" * pad
        words = []
        for i in range(0, len(data), 4):
            words.append(f"{int.from_bytes(data[i:i+4], 'little'):X}")
        for i in range(0, len(words), 7):  # keep command line < ~80 chars
            chunk = words[i:i+7]
            cmd = f"EW {start + i*4:X} " + " ".join(chunk)
            self.command(cmd)
            for _ in self.readlines():
                break

    # --------------------------
    # Fallback jump to download
    # --------------------------
    def memory_boot(self, address: int, force_find: bool = False):
        address |= 1  # thumb bit
        # Find an unused ROM console command entry
        if force_find or not hasattr(self, "_boot_cmd") or self._boot_cmd is None:
            cmd_array = self.register_read(REG_ROM_CMD_ARRAY_PTR)
            cmd_size = 4 * 3
            func_ptr = None
            name = None
            for cmd_ptr in range(cmd_array, cmd_array + 8 * cmd_size, cmd_size):
                name_ptr = self.register_read(cmd_ptr + 0)
                if name_ptr == 0:
                    break
                # read up to 16 bytes of name
                name_bytes = b''.join(self.dump_bytes(name_ptr, 16))
                name_bytes = name_bytes.split(b'\x00', 1)[0]
                try:
                    name_str = name_bytes.decode()
                except Exception:
                    continue
                if name_str in USED_COMMANDS:
                    continue
                func_ptr = cmd_ptr + 4
                name = name_str
                break
            if func_ptr is None:
                raise RuntimeError("No unused ROM command found; cannot boot from SRAM")
            self._boot_cmd = (func_ptr, name)
        func_ptr, name = self._boot_cmd
        # overwrite handler address
        self.register_write(func_ptr, address)
        self._log(f"[fallback] Jumping to 0x{address:X} using cmd '{name}'")
        # execute it
        self.command(name)

    # ----------------
    # Flash helpers
    # ----------------
    @property
    def flash_cfg(self) -> str:
        return f"{self.flash_speed} {self.flash_mode}"

    def flash_init(self, configure: bool = True):
        if self.flash_mode is None:
            reg = self.register_read(REG_BOOT_FLAGS)
            self.flash_mode = (reg >> 5) & 0b11
            # unprotect/configure flash controller
            self.register_write(REG_FLASH_CTRL, 0x7EFFFFFF)
            self._log(f"[flash] mode={self.flash_mode}")
        if self.flash_hash_offset is None and configure:
            # prime hash offset mechanism without sending data
            self.flash_read_hash(offset=None, length=0)
            self._log(f"[flash] cfg set: speed={self.flash_speed} mode={self.flash_mode}")

    def flash_transmit(self, stream, offset: int, progress_cb=None):
        # ensure flash config known
        self.flash_init(configure=False)
        # allow time for ROM to switch into XMODEM receiver
        self.push_timeout(3.0)
        self.command(f"fwd {self.flash_cfg} {offset:x}")
        self.flash_hash_offset = offset
        if stream is None:
            # empty XMODEM to set hash start offset
            resp = self.read(1)
            if resp != b"\x15":  # expect NAK
                raise RuntimeError(f"expected NAK, got {resp!r}")
            self.xm.abort()
            self.flush()
            resp = self.read(3)
            if resp != b"\x18ER":  # expect CAN echo
                raise RuntimeError(f"expected CAN, got {resp!r}")
        else:
            self._log(f"[xmodem] transmitting to 0x{offset:X}")
            if not self.xm.send(stream, callback=progress_cb):
                raise RuntimeError("XMODEM transmission failed")
        self.pop_timeout()
        self.link()

    def flash_read_hash(self, offset, length: int) -> bytes:
        # ensure hash start offset set
        if self.flash_hash_offset != offset:
            self.flash_transmit(None, offset)
        timeout = self.default_timeout
        if length:
            # add time proportional to length/CRC throughput
            import math
            timeout += math.ceil(length / CRC_SPEED_BPS * 10) / 10
        self.command(f"hashq {length} {self.flash_cfg}")
        self.push_timeout(timeout)
        resp = self.read(6 + 32)  # b"hashs " + 32 binary bytes
        self.pop_timeout()
        if not resp.startswith(b"hashs "):
            raise RuntimeError(f"Unexpected response to 'hashq': {resp!r}")
        return resp[6:6+32]

    # ========= API =========
    def read_flash(self, offset: int, length: int, out_stream, verify=True, progress=False):
        # Init flash (sets mode and primes hash)
        self.flash_init()
        total = 0
        hasher = sha256()
        # choose conservative chunk size based on baud rate (heuristic from ltchiptool)
        baud = int(self.s.baudrate or DEFAULT_LINK_BAUD)
        baud_coef = int(1 / (baud ** 0.5) * 2000)
        chunk_size = min((2 ** baud_coef) * 1024, 32 * 1024)
        start = offset | FLASH_MMAP_BASE
        remaining = length
        while remaining > 0:
            count = min(chunk_size, remaining)
            for blk in self.dump_bytes(start, count):
                out_stream.write(blk)
                total += len(blk)
                if verify:
                    hasher.update(blk)
                if progress:
                    print(f"\r[read] {total}/{length} bytes", end="", file=sys.stderr)
            start += count
            remaining -= count
        if progress:
            print(file=sys.stderr)
        if verify and length > 0:
            expected = self.flash_read_hash(offset, length)
            ours = hasher.digest()
            if ours != expected:
                raise RuntimeError(f"Hash mismatch! expected={expected.hex()} got={ours.hex()}")

    def write_flash(self, offset: int, in_stream, length: int, verify=True, progress=False):
        # XMODEM sender wrapper that honors length
        base_pos = in_stream.tell()
        class LimitedReader:
            def __init__(self, s, rem):
                self.s = s
                self.rem = rem
            def read(self, size):
                if self.rem <= 0:
                    return b""
                size = min(size, self.rem)
                data = self.s.read(size)
                self.rem -= len(data)
                return data

        def cb(total_packets, ok, err):
            if progress:
                sent = (ok + err) * 1024
                print(f"\r[write] ~{sent}/{length} bytes (ok={ok} err={err})", end="", file=sys.stderr)

        self.flash_transmit(LimitedReader(in_stream, length), offset, progress_cb=cb)
        if progress:
            print(file=sys.stderr)
        if verify and length > 0:
            in_stream.seek(base_pos)
            digest = sha256(in_stream.read(length)).digest()
            expected = self.flash_read_hash(offset, length)
            if digest != expected:
                raise RuntimeError(f"Verify failed! expected={expected.hex()} got={digest.hex()}")

    def erase_flash(self, offset: int, length: int, progress=False):
        # Erase by streaming 0xFF via the ROM write path (sectors get erased as needed)
        remaining = length
        pos = offset
        block_bytes = 64 * 1024
        class FFReader:
            def __init__(self, total):
                self.left = total
                self.buf = b"\xFF" * 1024
            def read(self, size):
                if self.left <= 0:
                    return b""
                n = min(size, self.left, len(self.buf))
                self.left -= n
                return self.buf[:n]
        while remaining > 0:
            chunk = min(block_bytes, remaining)
            def cb(total_packets, ok, err):
                if progress:
                    sent = (ok + err) * 1024
                    done = min(sent, chunk)
                    print(f"\r[erase] 0x{pos:06X} +{done}/{chunk} bytes", end="", file=sys.stderr)
            self.flash_transmit(FFReader(chunk), pos, progress_cb=cb)
            if progress:
                print(file=sys.stderr)
            pos += chunk
            remaining -= chunk


def parse_args():
    p = argparse.ArgumentParser(description="AmebaZ2 (RTL8720C) UART flashtool")
    p.add_argument("-p", "--port", required=True, help="Serial port (e.g. COM5, /dev/ttyUSB0)")
    p.add_argument("-b", "--baud", type=int, default=DEFAULT_LINK_BAUD, help=f"Initial baud (default {DEFAULT_LINK_BAUD})")
    p.add_argument("--baud-up", type=int, default=None, help="Optional higher baud after linking (e.g. 1500000)")
    p.add_argument("-v", "--verbose", action="store_true", help="Verbose logs")

    sub = p.add_subparsers(dest="cmd", required=True)

    pr = sub.add_parser("read", help="Read flash to file")
    pr.add_argument("--offset", type=lambda x:int(x,0), default=0)
    pr.add_argument("--length", type=lambda x:int(x,0), required=True)
    pr.add_argument("-o", "--output", required=True)
    pr.add_argument("--no-verify", action="store_true")

    pw = sub.add_parser("write", help="Write (flash) file to device")
    pw.add_argument("--offset", type=lambda x:int(x,0), default=0)
    pw.add_argument("-i", "--input", required=True)
    pw.add_argument("--no-verify", action="store_true")

    pe = sub.add_parser("erase", help="Erase flash region (by writing 0xFF)")
    pe.add_argument("--offset", type=lambda x:int(x,0), default=0)
    pe.add_argument("--length", type=lambda x:int(x,0), default=DEFAULT_FLASH_SIZE)

    return p.parse_args()

def main():
    args = parse_args()
    s = AmebaZ2Serial(args.port, baudrate=args.baud, verbose=args.verbose)
    try:
        print("[*] Linking...", file=sys.stderr)
        s.link()
        if args.baud_up:
            print(f"[*] Switching baud to {args.baud_up}...", file=sys.stderr)
            s.change_baud(args.baud_up)

        if args.cmd == "read":
            verify = not args.no_verify
            with open(args.output, "wb") as f:
                print(f"[*] Reading 0x{args.length:X} @ 0x{args.offset:X} ...", file=sys.stderr)
                s.read_flash(args.offset, args.length, f, verify=verify, progress=True)
            print("[+] Read complete.", file=sys.stderr)

        elif args.cmd == "write":
            verify = not args.no_verify
            size = os.path.getsize(args.input)
            with open(args.input, "rb") as f:
                print(f"[*] Writing 0x{size:X} @ 0x{args.offset:X} ...", file=sys.stderr)
                s.write_flash(args.offset, f, size, verify=verify, progress=True)
            print("[+] Write complete.", file=sys.stderr)

        elif args.cmd == "erase":
            print(f"[*] Erasing 0x{args.length:X} @ 0x{args.offset:X} ...", file=sys.stderr)
            s.erase_flash(args.offset, args.length, progress=True)
            print("[+] Erase complete.", file=sys.stderr)

    finally:
        try:
            s.disconnect()
        except Exception:
            pass
        s.close()

if __name__ == "__main__":
    main()
