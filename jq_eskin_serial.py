#!/usr/bin/env python3
"""
JQ Industries Fabric E-Skin Python Serial SDK

Protocol from product specification:
    frame = [AA 55 03 99] + [packet_seq] + [sensor_type] + [payload]

packet_seq:
    0x01 -> first packet, 128 pressure bytes
    0x02 -> second packet, 128 pressure bytes + 16 IMU bytes

sensor_type:
    0x01 -> LH (Left Hand)
    0x02 -> RH (Right Hand)
    0x03 -> LF (Left Foot)
    0x04 -> RF (Right Foot)
    0x05 -> WB (Whole Body)

Two packets are combined into 256 pressure ADC values (uint8, 0..255).

NOTE:
The specification says the last 16 bytes are quaternion data with
4 bytes per quaternion component, but does not explicitly define
endianness / numeric representation. This SDK exposes imu_raw and,
by default, also attempts float32 little-endian decoding.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterator, Optional, Tuple

import serial
from serial.tools import list_ports


HEADER = b"\xAA\x55\x03\x99"

SENSOR_TYPES = {
    0x01: "LH",
    0x02: "RH",
    0x03: "LF",
    0x04: "RF",
    0x05: "WB",
}

PACKET1_DATA_LEN = 128
PACKET2_DATA_LEN = 144

PACKET1_FRAME_LEN = len(HEADER) + 1 + 1 + PACKET1_DATA_LEN  # 134
PACKET2_FRAME_LEN = len(HEADER) + 1 + 1 + PACKET2_DATA_LEN  # 150


@dataclass
class Packet:
    seq: int
    sensor_type: int
    sensor_name: str
    pressure: bytes
    imu_raw: bytes = b""


@dataclass
class SensorFrame:
    sensor_type: int
    sensor_name: str
    pressure: Tuple[int, ...]       # 256 ADC values
    imu_raw: bytes                  # 16 bytes
    quaternion: Optional[Tuple[float, float, float, float]]
    timestamp: float


class JQESkinSerial:
    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        timeout: float = 0.05,
        imu_format: str = "<4f",
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.imu_format = imu_format

        self.ser: Optional[serial.Serial] = None
        self._rx = bytearray()

        # Hold packet-1 separately per sensor.
        self._pending_first: Dict[int, bytes] = {}

    def open(self) -> None:
        if self.ser and self.ser.is_open:
            return

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "JQESkinSerial":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @staticmethod
    def available_ports():
        return [
            {
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
            }
            for p in list_ports.comports()
        ]

    def _read_more(self) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port is not open")

        n = self.ser.in_waiting
        data = self.ser.read(n if n > 0 else 1)
        if data:
            self._rx.extend(data)

    def _extract_packet(self) -> Optional[Packet]:
        """
        Streaming parser:
        - resynchronizes on AA 55 03 99
        - validates packet sequence
        - validates sensor type
        """
        while True:
            idx = self._rx.find(HEADER)

            if idx < 0:
                # Keep last few bytes in case they are the beginning of HEADER.
                keep = len(HEADER) - 1
                if len(self._rx) > keep:
                    del self._rx[:-keep]
                return None

            if idx > 0:
                del self._rx[:idx]

            # Need header + seq + type.
            if len(self._rx) < 6:
                return None

            seq = self._rx[4]
            sensor_type = self._rx[5]

            if seq == 0x01:
                frame_len = PACKET1_FRAME_LEN
                payload_len = PACKET1_DATA_LEN
            elif seq == 0x02:
                frame_len = PACKET2_FRAME_LEN
                payload_len = PACKET2_DATA_LEN
            else:
                # False header / corrupt stream: shift one byte and resync.
                del self._rx[0]
                continue

            if sensor_type not in SENSOR_TYPES:
                del self._rx[0]
                continue

            if len(self._rx) < frame_len:
                return None

            frame = bytes(self._rx[:frame_len])
            del self._rx[:frame_len]

            payload = frame[6:6 + payload_len]

            if seq == 0x01:
                return Packet(
                    seq=seq,
                    sensor_type=sensor_type,
                    sensor_name=SENSOR_TYPES[sensor_type],
                    pressure=payload,
                )

            return Packet(
                seq=seq,
                sensor_type=sensor_type,
                sensor_name=SENSOR_TYPES[sensor_type],
                pressure=payload[:128],
                imu_raw=payload[128:144],
            )

    def read_packet(self, timeout: Optional[float] = None) -> Optional[Packet]:
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            pkt = self._extract_packet()
            if pkt is not None:
                return pkt

            if deadline is not None and time.monotonic() >= deadline:
                return None

            self._read_more()

    def _decode_quaternion(
        self, raw: bytes
    ) -> Optional[Tuple[float, float, float, float]]:
        if len(raw) != 16:
            return None

        try:
            values = struct.unpack(self.imu_format, raw)
            return tuple(float(x) for x in values)  # type: ignore
        except struct.error:
            return None

    def read_frame(self, timeout: Optional[float] = None) -> Optional[SensorFrame]:
        """
        Return one complete 256-point frame.

        Packet 1:
            128 pressure bytes

        Packet 2:
            128 pressure bytes + 16 IMU bytes
        """
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            remain = None
            if deadline is not None:
                remain = max(0.0, deadline - time.monotonic())
                if remain <= 0:
                    return None

            pkt = self.read_packet(timeout=remain)
            if pkt is None:
                return None

            if pkt.seq == 0x01:
                self._pending_first[pkt.sensor_type] = pkt.pressure
                continue

            first = self._pending_first.pop(pkt.sensor_type, None)
            if first is None:
                # Packet 2 arrived without matching packet 1.
                continue

            pressure = tuple(first + pkt.pressure)

            return SensorFrame(
                sensor_type=pkt.sensor_type,
                sensor_name=pkt.sensor_name,
                pressure=pressure,
                imu_raw=pkt.imu_raw,
                quaternion=self._decode_quaternion(pkt.imu_raw),
                timestamp=time.time(),
            )

    def frames(self) -> Iterator[SensorFrame]:
        while True:
            frame = self.read_frame()
            if frame is not None:
                yield frame


def main() -> None:
    parser = argparse.ArgumentParser(description="JQ Fabric E-Skin serial reader")
    parser.add_argument("--port", help="COM port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument(
        "--imu-format",
        default="<4f",
        help="struct format for 16-byte quaternion; default '<4f'",
    )
    parser.add_argument("--list", action="store_true", help="list serial ports")
    parser.add_argument("--raw", action="store_true", help="print all 256 ADC values")
    args = parser.parse_args()

    if args.list:
        for p in JQESkinSerial.available_ports():
            print(f"{p['device']:15s}  {p['description']}  {p['hwid']}")
        return

    if not args.port:
        parser.error("--port is required unless --list is used")

    with JQESkinSerial(
        args.port,
        baudrate=args.baud,
        imu_format=args.imu_format,
    ) as skin:
        print(f"Listening on {args.port} @ {args.baud} bps")

        for frame in skin.frames():
            if args.raw:
                print(
                    f"{frame.sensor_name} "
                    f"pressure={list(frame.pressure)} "
                    f"quat={frame.quaternion} "
                    f"imu_raw={frame.imu_raw.hex(' ')}"
                )
            else:
                p = frame.pressure
                print(
                    f"{frame.sensor_name:2s} "
                    f"min={min(p):3d} max={max(p):3d} "
                    f"mean={sum(p)/len(p):6.1f} "
                    f"quat={frame.quaternion}"
                )


if __name__ == "__main__":
    main()
