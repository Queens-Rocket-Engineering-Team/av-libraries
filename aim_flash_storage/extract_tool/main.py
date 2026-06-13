"""
Author: Kennan Bays (Kenneract), Tristan Alderson
Updated: June 2026
Purpose: Extract telemetry from AimFlightRecorder over serial.

The device must be running firmware that includes aim_console. The tool
navigates three fixed library-owned keys (d → f → 2) to trigger the dump.
All metadata (board name, column count, headers) comes from the
self-describing binary handshake — no menu parsing required.
"""

from serial import Serial
import struct
import sys
import time
import datetime
import csv
from rdes import RDESDecompressor

BAUD_RATE    = 115200
DEFAULT_PORT = "/dev/ttyUSB0"
TIMEOUT      = 0.5

# Wire-protocol bytes — must match AimFlightRecorder.h
DUMP_START      = b'#'
DUMP_CMD_NEXT   = b'N'
DUMP_CMD_RESEND = b'L'

# Handshake field widths — must match AimFlightRecorder::kHandshake* constants
HANDSHAKE_NAME   = 32
HANDSHAKE_HEADER = 32


def connect_to_board(port=DEFAULT_PORT):
    print(f"Connecting to {port}...")
    try:
        device = Serial(port, baudrate=BAUD_RATE, timeout=TIMEOUT)
        time.sleep(0.5)
        device.read_all()
        return device
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None


def retrieve_board_flash(device):
    """Navigate library-owned menus (d→f→2) and download the self-describing dump."""
    device.write(b'q')   # exit any active console state
    time.sleep(0.1)
    device.write(b'd')   # enter console  (library: "DBG [...")
    time.sleep(0.1)
    device.write(b'f')   # flash menu     (library: "FLS [...")
    time.sleep(0.1)
    device.write(b'2')   # dump

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
    print(f"Size  : {total_bytes}B ({num_blocks} blocks of {block_size}B)")

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


def write_csv(name, data, headers):
    dt       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{name}_{dt}.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for row in data:
            writer.writerow(row)
    print(f"Saved to {filename}")


def main():
    port   = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    device = connect_to_board(port)
    if not device:
        return

    raw, name, cols, headers = retrieve_board_flash(device)
    print(f"Decompressing ({cols} columns)...")
    decomp = RDESDecompressor(numCols=cols)
    data   = decomp.decompress(raw)
    write_csv(name, data, headers)


if __name__ == "__main__":
    main()
