"""
Author: Kennan Bays (Kenneract), Tristan Alderson
Updated: June 9, 2026
Purpose: Refactored extraction tool that interacts with the AimFlightRecorder 
         debug console directly via serial.
"""

from serial import Serial
import struct
import sys
import time
import datetime
import csv
import re
import json
from rdes import RDESDecompressor

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/ttyUSB0"
TIMEOUT = 0.5

# Dump wire-protocol bytes — must match AimFlightRecorder.h
# (kDumpStartChar / kDumpCmdNext / kDumpCmdResend).
DUMP_START = b'#'
DUMP_CMD_NEXT = b'N'
DUMP_CMD_RESEND = b'L'

def connect_to_board(port=DEFAULT_PORT):
    print(f"Connecting to {port}...")
    try:
        device = Serial(port, baudrate=BAUD_RATE, timeout=TIMEOUT)
        time.sleep(0.5)
        device.read_all() # Clear buffer
        return device
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def enter_console_root(device):
    """Returns the console to the root DBG menu from any state.

    'q' exits the console if it was active (ignored otherwise); console
    entry then uses 'd', the single debug-console entry key.
    """
    device.write(b'q')
    time.sleep(0.1)
    device.write(b'd')
    if wait_for_prompt(device, "DBG [") is None:
        raise IOError("Console entry failed: no 'DBG [' prompt")

def wait_for_prompt(device, prompt, timeout=2.0):
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        if device.in_waiting:
            char = device.read().decode("cp1252", errors="ignore")
            buf += char
            if prompt in buf:
                return buf
    return None

def get_board_info(device):
    """Navigates menus to find board name, flash usage, and telemetry schema."""
    enter_console_root(device)

    # Mute asynchronous logs to prevent UART corruption
    device.write(b'2') # LOG menu
    wait_for_prompt(device, "LOG [")
    device.write(b'0') # OFF
    time.sleep(0.1)
    device.write(b'b') # Back to root
    wait_for_prompt(device, "DBG [")
 
    # 1. Get Status (for name)
    device.write(b'1')
    status_text = wait_for_prompt(device, "DBG [", timeout=2.0)

    name = "UNKNOWN"
    if status_text:
        match = re.search(r"name=(\S+)", status_text)
        if match:
            name = match.group(1)

    # 2. Get Flash Info (Total/Used)
    device.write(b'3') # Flash menu
    wait_for_prompt(device, "FLS [")

    device.write(b'1') # Info
    info_text = wait_for_prompt(device, "FLS [", timeout=2.0)

    total = 0
    used = 0

    if info_text:
        m_total = re.search(r"total=(\d+)", info_text)
        m_used = re.search(r"used=(\d+)", info_text)

        if m_total:
            total = int(m_total.group(1))
        if m_used:
            used = int(m_used.group(1))

        if total == 0:
            m_bar = re.search(r"\[(\d+)B/(\d+)B\]", info_text)
            if m_bar:
                used = int(m_bar.group(1))
                total = int(m_bar.group(2))

    # 3. Get Telemetry Schema from config.json
    device.write(b'4') # CFG menu
    wait_for_prompt(device, "CFG [")
    device.write(b'5') # json dump
    
    json_buf = wait_for_prompt(device, "[/CFG]", timeout=5.0)
    
    cols = 8 # Default fallback
    headers = ["Time", "C1", "C2", "C3", "C4", "C5", "C6", "C7"]
    
    if json_buf and "[CFG]" in json_buf:
        # Extract everything between [CFG] and [/CFG]
        start_idx = json_buf.find("[CFG]") + 5
        end_idx = json_buf.find("[/CFG]")
        clean_json = json_buf[start_idx:end_idx].strip()
        
        try:
            config = json.loads(clean_json)
            if "telemetry" in config:
                cols = config["telemetry"].get("cols", cols)
                headers = config["telemetry"].get("headers", headers)
                print(f"Found dynamic schema: {cols} columns.")
        except Exception as e:
            print(f"Warning: Failed to parse config.json ({e}). Using default schema.")
            
    return name, total, used, cols, headers

def retrieve_board_flash(device):
    """Triggers the binary dump from the flash menu."""
    enter_console_root(device)

    device.write(b'3') # Flash menu
    if wait_for_prompt(device, "FLS [") is None:
        raise IOError("No 'FLS [' prompt")

    device.write(b'2') # Dump
    # Handshake starts with DUMP_START — bounded wait so a failed dump start
    # ("[ERR] dump failed to start") can't hang us forever
    start = time.time()
    while True:
        c = device.read()
        if c == DUMP_START:
            break
        if time.time() - start > 5.0:
            raise IOError("No dump handshake ('#') from board")
            
    # Read binary handshake
    block_size = struct.unpack('H', device.read(2))[0]
    num_blocks = struct.unpack('H', device.read(2))[0]
    num_real_bytes = struct.unpack('I', device.read(4))[0]
    
    print(f"Downloading {num_real_bytes}B ({num_blocks} blocks of {block_size}B)")
    
    raw_payload = bytearray()
    device.write(DUMP_CMD_NEXT) # ACK start
    
    start_time = time.time()
    for b in range(num_blocks):
        if (b + 1) % 10 == 0 or b == 0:
            prog = (b + 1) / num_blocks
            print(f"\rProgress: {prog:.1%} ", end="", flush=True)
            
        block = device.read(block_size)
        retries = 0
        while len(block) < block_size:
            # Resend-retry for partial blocks, bounded so a protocol fault
            # can't spin forever
            retries += 1
            if retries > 5:
                raise IOError(f"Block {b}: got {len(block)}/{block_size}B after {retries - 1} retries")
            device.reset_input_buffer()
            device.write(DUMP_CMD_RESEND)
            block = device.read(block_size)

        raw_payload.extend(block)
        # Ack every block — the final ack tells the board the dump is complete
        device.write(DUMP_CMD_NEXT)
            
    elapsed = time.time() - start_time
    speed = (num_real_bytes / elapsed) / 1024 if elapsed > 0 else 0
    print(f"\nDone. Speed: {speed:.1f} KiB/s")
    
    return raw_payload[:num_real_bytes]

def erase_board_flash(device):
    """Navigates to the erase confirm menu."""
    enter_console_root(device)

    device.write(b'3') # Flash
    wait_for_prompt(device, "FLS [")
    device.write(b'3') # Erase
    wait_for_prompt(device, "ERS [")
    device.write(b'1') # Confirm
    
    print("Formatting flash (may take up to 60s)...")
    wait_for_prompt(device, "FLS [", timeout=60.0)
    print("Erase complete.")

def write_csv(name, data, headers):
    dt = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{name}_{dt}.csv"
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for row in data:
            writer.writerow(row)
    print(f"Saved to {filename}")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    device = connect_to_board(port)
    if not device: return

    name, total, used, cols, headers = get_board_info(device)
    print(f"\nConnected to: {name}")

    if total > 0:
        print(f"Flash: {used/1024:.1f} KB / {total/1024:.1f} KB used ({used/total:.1%})")
    else:
        print(f"Flash: {used/1024:.1f} KB used / total unknown")

    while True:
        print("\nCommands:")
        print("1. Dump Telemetry")
        print("2. Erase Flash")
        print("3. Exit")
        cmd = input("> ").strip()
        
        if cmd == "1":
            raw = retrieve_board_flash(device)
            print(f"Decompressing RDES ({cols} columns)...")
            decomp = RDESDecompressor(numCols=cols)
            data = decomp.decompress(raw)
            write_csv(name, data, headers)
        elif cmd == "2":
            conf = input("Type 'Confirm' to erase: ")
            if conf == "Confirm":
                erase_board_flash(device)
        elif cmd == "3":
            break

if __name__ == "__main__":
    main()
