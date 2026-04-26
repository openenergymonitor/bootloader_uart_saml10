#!/usr/bin/env python3

"""
*
* \brief boot.py
*
* Copyright (c) 2019 Microchip Technology Inc.
                2025 Angus Logan (awjlogan@gmail.com)
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the Licence at
*
 * http://www.apache.org/licenses/LICENSE-2.0
*
 * Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    from rich.progress import Progress
except ImportError:  # Quiet mode should still work without rich.
    Progress = None


# ------------------------------------------------------------------------------
# Bootloader commands
BL_CMD_UNLOCK = 0xA0
BL_CMD_DATA = 0xA1
BL_CMD_VERIFY = 0xA2
BL_CMD_RESET = 0xA3

# Bootloader responses
BL_RESP_OK = 0x50
BL_RESP_ERROR = 0x51
BL_RESP_INVALID = 0x52
BL_RESP_CRC_OK = 0x53
BL_RESP_CRC_FAIL = 0x54

BOOTLOADER_SIZE = 1024
PAGE_SIZE = 256
GUARD_WORD = 0x2B620BC3
RETRY_COUNT = 3
RETRY_DELAY = 0.2
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1
REBOOT_PAYLOAD = bytes(16)

RESPONSE_NAMES = {
    BL_RESP_OK: "ok",
    BL_RESP_ERROR: "error",
    BL_RESP_INVALID: "invalid",
    BL_RESP_CRC_OK: "crc-ok",
    BL_RESP_CRC_FAIL: "crc-fail",
}


class BootError(Exception):
    pass


# ------------------------------------------------------------------------------
def warning(text):
    sys.stderr.write(f"Warning: {text}\n")


# ------------------------------------------------------------------------------
def verbose(enabled, text, nl=True):
    if enabled:
        if nl:
            print(text)
        else:
            print(text, end="")


# ------------------------------------------------------------------------------
def crc32_tab_gen():
    res = []

    for i in range(256):
        value = i

        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ 0xEDB88320
            else:
                value >>= 1

        res.append(value)

    return res


CRC32_TABLE = crc32_tab_gen()


# ------------------------------------------------------------------------------
def crc32(tab, data):
    crc = 0xFFFFFFFF

    for d in data:
        crc = tab[(crc ^ d) & 0xFF] ^ (crc >> 8)

    return crc


# ------------------------------------------------------------------------------
def uint32(value):
    return value.to_bytes(4, byteorder="little", signed=False)


# ------------------------------------------------------------------------------
def parse_offset(text):
    try:
        return int(text, 0)
    except ValueError as inst:
        raise BootError(f"Invalid offset value: {text}") from inst


# ------------------------------------------------------------------------------
def pad_file_data(data):
    remainder = len(data) % PAGE_SIZE
    if remainder != 0:
        data.extend(b"\xFF" * (PAGE_SIZE - remainder))
    return data


# ------------------------------------------------------------------------------
def iter_blocks(data):
    for start in range(0, len(data), PAGE_SIZE):
        yield data[start : start + PAGE_SIZE]


# ------------------------------------------------------------------------------
def list_ports():
    if serial is None:
        raise BootError("pyserial is required to list serial ports")

    print("Available ports:")
    for comport in serial.tools.list_ports.comports():
        print("  -", comport.device)


# ------------------------------------------------------------------------------
def response_name(resp):
    return RESPONSE_NAMES.get(resp, f"unknown(0x{resp:02x})")


# ------------------------------------------------------------------------------
def ensure_response(resp, expected, context):
    if resp == expected:
        return

    if context:
        raise BootError(
            f"{context} Received bootloader response {response_name(resp)} (0x{resp:02x})."
        )

    raise BootError(
        f"Unexpected bootloader response {response_name(resp)} (0x{resp:02x})."
    )


# ------------------------------------------------------------------------------
def read_binary_file(path):
    try:
        return bytearray(Path(path).read_bytes())
    except OSError as inst:
        raise BootError(inst) from inst


# ------------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        prog="boot.py",
        description="This program loads a binary file through UART to a SAML10 with bootloader.",
        epilog="github.com/awjlogan/bootloader_uart_saml10",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="enable verbose output",
        action="store_true",
    )
    parser.add_argument(
        "--list-ports",
        help="list available communication ports and exit",
        action="store_true",
    )
    parser.add_argument(
        "-i", "--interface", dest="port", help="Communication interface", metavar="PATH"
    )
    parser.add_argument(
        "-f", "--file", dest="file", help="Binary file to program", metavar="FILE"
    )
    parser.add_argument(
        "-o",
        "--offset",
        help="Destination offset (default 0x400)",
        default="0x400",
        metavar="OFFS",
    )
    parser.add_argument(
        "-r",
        "--reboot",
        help="send the reboot command",
        action="store_true",
    )
    parser.add_argument(
        "--boot",
        help="Enable write to the bootloader area",
        action="store_true",
    )
    return parser


class BootloaderClient:
    def __init__(self, port):
        self.port = port

    def get_response(self):
        response = self.port.read()

        if len(response) == 0:
            return None
        if len(response) > 1:
            raise BootError("Invalid response received (size > 1)")
        if response[0] not in RESPONSE_NAMES:
            raise BootError(f"Unknown response byte: 0x{response[0]:02x}")

        return response[0]

    def send_request(self, command, data):
        request = bytes([command]) + uint32(GUARD_WORD) + bytes(data)

        for attempt in range(1, RETRY_COUNT + 1):
            try:
                self.port.write(request)
                response = self.get_response()
            except serial.serialutil.SerialException as inst:
                raise BootError(inst) from inst

            if response is not None:
                return response

            warning(f"No response received, retrying {attempt}")
            time.sleep(RETRY_DELAY)

        raise BootError("No response received, giving up")

    def upload(self, options, offset):
        data = pad_file_data(read_binary_file(options.file))
        size = len(data)
        checksum = crc32(CRC32_TABLE, data)

        verbose(options.verbose, "Unlocking")
        response = self.send_request(
            BL_CMD_UNLOCK,
            uint32(offset) + uint32(size),
        )
        ensure_response(
            response,
            BL_RESP_OK,
            "Bootloader rejected unlock request. Check the file size and offset.",
        )

        self.send_blocks(options, data, offset)

        verbose(options.verbose, "Verification", nl=False)
        response = self.send_request(BL_CMD_VERIFY, uint32(checksum))
        ensure_response(response, BL_RESP_CRC_OK, "CRC verification failed.")

        if options.verbose:
            print("... success")

    def send_blocks(self, options, data, offset):
        total_blocks = len(data) // PAGE_SIZE
        verbose(
            options.verbose,
            "Uploading %d blocks at offset %d (0x%x)" % (total_blocks, offset, offset),
        )

        payload = bytearray(4 + PAGE_SIZE)
        payload[:4] = uint32(offset)

        progress = None
        task = None

        if options.verbose and Progress is not None:
            progress = Progress()
            progress.start()
            task = progress.add_task("Uploading...", total=total_blocks)
        elif options.verbose and Progress is None:
            warning("rich is not installed; falling back to simple verbose output")

        try:
            for index, block in enumerate(iter_blocks(data), start=1):
                addr = offset + ((index - 1) * PAGE_SIZE)
                payload[:4] = uint32(addr)
                payload[4:] = block

                response = self.send_request(BL_CMD_DATA, payload)
                ensure_response(response, BL_RESP_OK, "Bootloader rejected page write.")

                if progress is not None:
                    progress.update(task, advance=1)
                elif options.verbose:
                    print(f"  wrote block {index}/{total_blocks}")
        finally:
            if progress is not None:
                progress.stop()

    def reboot(self, options):
        verbose(options.verbose, "Rebooting")
        response = self.send_request(BL_CMD_RESET, REBOOT_PAYLOAD)
        ensure_response(response, BL_RESP_OK, "Bootloader rejected reset request.")


# ------------------------------------------------------------------------------
def validate_options(options):
    if options.list_ports:
        return

    if not options.port:
        raise BootError("Communication port is required. Use --list-ports to discover devices.")

    if options.file is None and not options.reboot:
        raise BootError("Nothing to do. Provide --file and/or --reboot.")

    offset = parse_offset(options.offset)

    if (offset < BOOTLOADER_SIZE) and not options.boot:
        raise BootError("Offset is within the bootloader area, use --boot to unlock writes")

    return offset


# ------------------------------------------------------------------------------
def run(options):
    if options.list_ports:
        list_ports()
        return

    offset = validate_options(options)

    if serial is None:
        raise BootError("pyserial is required to use the bootloader tool")

    try:
        port = serial.Serial(options.port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    except serial.serialutil.SerialException as inst:
        raise BootError(inst) from inst

    with port:
        client = BootloaderClient(port)
        if options.file is not None:
            client.upload(options, offset)
        if options.reboot:
            client.reboot(options)

    verbose(options.verbose, "Done!")


# ------------------------------------------------------------------------------
def main():
    parser = build_parser()
    options = parser.parse_args()

    try:
        run(options)
    except BootError as inst:
        sys.stderr.write(f"Error: {inst}\n")
        sys.exit(1)


# ------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
