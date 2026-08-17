"""
Author: Kennan Bays (Kenneract), Tristan Alderson
Updated: August 2026
Purpose: Extract telemetry from AimFlightRecorder over serial with RDES3 decompression.

The device must be running firmware that includes aim_console.
Supports listing stored flight logs (--list), dumping a specific log index (--index N),
or extracting all available flight logs (--all).
"""

from serial import Serial
import struct
import sys
import time
import datetime
import csv
import os
import argparse

sys.path.append(os.path.join(os.path.dirname(__file__), "../../aim_rdes/python"))
from rdes_ctypes import RDESDecompressorCTypes

BAUD_RATE    = 115200
DEFAULT_PORT = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
TIMEOUT      = 0.5

# Wire-protocol bytes — must match AimFlightRecorder.h
DUMP_START      = b'#'
DUMP_CMD_NEXT   = b'N'
DUMP_CMD_RESEND = b'L'

# Handshake field widths — must match AimFlightRecorder::kHandshake* constants
HANDSHAKE_NAME   = 32
HANDSHAKE_HEADER = 32


def connect_to_board(port=DEFAULT_PORT, baudrate=BAUD_RATE):
    print(f"Connecting to {port} @ {baudrate} baud...")
    try:
        device = Serial(port, baudrate=baudrate, timeout=TIMEOUT)
        device.dtr = False
        device.rts = False
        time.sleep(0.5)
        device.reset_input_buffer()
        return device
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None


def navigate_to_flash_menu(device):
    """Robustly navigate library-owned menus (q -> d -> f) by verifying prompt responses."""
    device.write(b'q')
    time.sleep(0.15)
    device.reset_input_buffer()
    
    # Enter root console
    device.write(b'd')
    start = time.time()
    while time.time() - start < 2.0:
        buf = device.read_all()
        if b"DBG [" in buf:
            break
        time.sleep(0.05)
        
    # Enter flash menu
    device.write(b'f')
    start = time.time()
    while time.time() - start < 2.0:
        buf = device.read_all()
        if b"FLS" in buf:
            break
        time.sleep(0.05)


def list_board_logs(device):
    """Query aim_console for a list of stored flight log files."""
    navigate_to_flash_menu(device)
    device.write(b'4')  # option 4: list logs
    
    buf = bytearray()
    start = time.time()
    while time.time() - start < 5.0:
        chunk = device.read_all()
        if chunk:
            buf.extend(chunk)
            if b"log(s) found" in buf or b"DBG > FLS" in buf:
                break
        time.sleep(0.05)
    
    output = buf.decode('ascii', errors='replace')
    print("\n--- Stored Board Flight Logs ---")
    print(output.strip())
    print("--------------------------------\n")
    device.write(b'q')


def retrieve_board_flash(device, log_index=None):
    """Navigate library-owned menus and download self-describing RDES3 dump."""
    navigate_to_flash_menu(device)
    
    if log_index is None:
        device.write(b'2')   # dump latest
    else:
        device.write(b'i')   # dump index
        time.sleep(0.1)
        device.write(bytes([log_index & 0xFF]))

    # Wait for handshake start byte '#'
    start = time.time()
    while True:
        c = device.read(1)
        if c == DUMP_START:
            break
        if time.time() - start > 5.0:
            raise IOError("No dump handshake ('#') — is the device running aim_console firmware?")

    # Fixed header: blockSize(2LE) + numBlocks(2LE) + totalBytes(4LE)
    block_size  = struct.unpack('<H', device.read(2))[0]
    num_blocks  = struct.unpack('<H', device.read(2))[0]
    total_bytes = struct.unpack('<I', device.read(4))[0]

    # Self-describing schema: boardName[32] + numCols(1) + headers[numCols][32]
    board_name = device.read(HANDSHAKE_NAME).rstrip(b'\x00').decode('ascii', errors='replace')
    num_cols   = struct.unpack('B', device.read(1))[0]
    headers    = [
        device.read(HANDSHAKE_HEADER).rstrip(b'\x00').decode('ascii', errors='replace')
        for _ in range(num_cols)
    ]

    print(f"Board : {board_name}")
    print(f"Schema: {num_cols} cols — {headers}")
    print(f"Log Size: {total_bytes}B [{num_blocks} blocks of {block_size}B]")

    # Block loop
    raw_payload = bytearray()
    device.write(DUMP_CMD_NEXT)

    start_time = time.time()
    for b in range(num_blocks):
        if b % 10 == 0 or b == num_blocks - 1:
            print(f"\r  {(b + 1) / num_blocks:.0%} ", end="", flush=True)

        block   = device.read(block_size)
        retries = 0
        while len(block) < block_size:
            retries += 1
            if retries > 5:
                raise IOError(f"Block {b}: got {len(block)}/{block_size}B after {retries - 1} retries")
            device.reset_input_buffer()
            device.write(DUMP_CMD_RESEND)
            block = device.read(block_size)

        raw_payload.extend(block)
        device.write(DUMP_CMD_NEXT)

    elapsed = time.time() - start_time
    speed   = (total_bytes / elapsed) / 1024 if elapsed > 0 else 0
    print(f"\n  Done — {speed:.1f} KiB/s")

    return raw_payload[:total_bytes], board_name, num_cols, headers


def write_csv(name, data, headers, outdir=".", suffix=""):
    os.makedirs(outdir, exist_ok=True)
    dt       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix_str = f"_{suffix}" if suffix else ""
    filename = os.path.join(outdir, f"{name}{suffix_str}_{dt}.csv")
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for row in data:
            writer.writerow(row)
    print(f"Saved to {filename}")


def main():
    parser = argparse.ArgumentParser(description="AimFlightRecorder Telemetry Extraction Tool")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Baud rate (default 115200)")
    parser.add_argument("--list", action="store_true", help="List all stored flight logs on the device")
    parser.add_argument("--index", type=int, help="Dump specific flight log by index number")
    parser.add_argument("--outdir", default=".", help="Directory to save extracted CSV files")
    
    args = parser.parse_args()

    device = connect_to_board(args.port, args.baud)
    if not device:
        return

    if args.list:
        list_board_logs(device)
        device.close()
        return

    suffix = f"log{args.index:03d}" if args.index is not None else "latest"

    raw, name, cols, headers = retrieve_board_flash(device, log_index=args.index)
    print(f"Decompressing RDES3 ({cols} columns)...")
    decomp = RDESDecompressorCTypes(numCols=cols)
    data   = decomp.decompress(raw)
    write_csv(name, data, headers, outdir=args.outdir, suffix=suffix)
    device.close()


if __name__ == "__main__":
    main()
