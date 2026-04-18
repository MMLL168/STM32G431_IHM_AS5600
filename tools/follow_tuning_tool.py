import json
import queue
import struct
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


ROOT_DIR = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT_DIR / "tools" / "follow_tuning_profile.json"
HEADER_PATH = ROOT_DIR / "Inc" / "tuning_config.h"
BUILD_SCRIPT = ROOT_DIR / "tools" / "cmake-debug.ps1"
PLAN_PATH = ROOT_DIR / "tools" / "follow_tuning_test_plan.md"
PRESET_DIR = ROOT_DIR / "tools" / "follow_tuning_presets"
RECOMMENDED_START_PRESET = "01_fixed_target_hold"
BASE_LAYOUT_WIDTH = 1140
BASE_LAYOUT_HEIGHT = 820

FOLLOW_TUNING_PROTOCOL_VERSION = 1
FOLLOW_TUNING_MCP_CALLBACK_ID = 0
ASPEP_DATA_PAYLOAD_GAP_SECONDS = 0.005
FOLLOW_TUNING_MCP_OPCODE_PING = 0
FOLLOW_TUNING_MCP_OPCODE_GET_PROFILE = 1
FOLLOW_TUNING_MCP_OPCODE_SET_PROFILE = 2
FOLLOW_TUNING_MCP_OPCODE_GET_SNAPSHOT = 3
FOLLOW_TUNING_MCP_OPCODE_RESET_PROFILE = 4

TEST_MODE_MACROS = {
    "normal_follow": "FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW",
    "fixed_target": "FOLLOW_CONTROL_TEST_MODE_FIXED_TARGET",
    "step_sequence": "FOLLOW_CONTROL_TEST_MODE_STEP_SEQUENCE",
    "direction_probe": "FOLLOW_CONTROL_TEST_MODE_DIRECTION_PROBE",
}
TEST_MODE_VALUES = {
    "normal_follow": 0,
    "fixed_target": 1,
    "step_sequence": 2,
    "direction_probe": 3,
}
TEST_MODE_NAMES = {value: key for key, value in TEST_MODE_VALUES.items()}

SECTIONS = {
    "Mode": [
        ("test_mode", "Test Mode", "enum"),
        ("fixed_target_offset_deg10", "Fixed Target Offset (deg10)", "int"),
        ("step_amplitude_deg10", "Step Amplitude (deg10)", "int"),
        ("step_dwell_ms", "Step Dwell (ms)", "int"),
        ("step_start_positive", "Step Starts Positive", "bool"),
    ],
    "Outer Loop": [
        ("hold_enter_deg10", "Hold Enter (deg10)", "int"),
        ("hold_exit_deg10", "Hold Exit (deg10)", "int"),
        ("speed_zero_window_rpm", "Speed Zero Window (rpm)", "int"),
        ("speed_kp_rpm_per_deg", "Speed Kp (rpm/deg)", "int"),
        ("position_current_ma_num", "Position Current Numerator", "int"),
        ("position_current_ma_den", "Position Current Denominator", "int"),
        ("damping_current_ma_per_rpm", "Damping Current (mA/rpm)", "int"),
        ("breakaway_current_ma", "Breakaway Current (mA)", "int"),
        ("breakaway_speed_window_rpm", "Breakaway Speed Window (rpm)", "int"),
        ("speed_ki_divisor", "Speed Ki Divisor", "int"),
        ("speed_ki_enable", "Speed Ki Enable", "bool"),
        ("speed_kd_rpm_per_rpm", "Speed Kd (rpm/rpm)", "int"),
        ("speed_to_current_kp_ma_per_rpm", "Speed to Current Kp", "int"),
        ("outer_angle_filter_shift", "Outer Angle Filter Shift", "int"),
        ("target_angle_slew_min_deg10_per_tick", "Slew Min (deg10/tick)", "int"),
        ("target_angle_slew_max_deg10_per_tick", "Slew Max (deg10/tick)", "int"),
        ("target_angle_slew_divisor", "Slew Divisor", "int"),
        ("target_angle_slew_accel_deg10_per_tick", "Slew Accel (deg10/tick)", "int"),
        ("target_angle_slew_decel_deg10_per_tick", "Slew Decel (deg10/tick)", "int"),
        ("settle_zone_deg10", "Settle Zone (deg10)", "int"),
        ("settle_max_rpm", "Settle Max RPM", "int"),
        ("settle_max_current_ma", "Settle Max Current (mA)", "int"),
        ("settle_min_current_ma", "Settle Min Current (mA)", "int"),
        ("max_rpm", "Max RPM", "int"),
        ("min_rpm", "Min RPM", "int"),
        ("max_current_ma", "Max Current (mA)", "int"),
        ("current_zero_window_ma", "Current Zero Window (mA)", "int"),
        ("source_direction_sign", "Source Direction Sign", "int"),
        ("follower_direction_sign", "Follower Direction Sign", "int"),
    ],
    "Probe": [
        ("direction_probe_target_abs_rpm", "Probe Target Abs RPM", "int"),
        ("direction_probe_max_current_ma", "Probe Max Current (mA)", "int"),
        ("direction_probe_speed_to_current_kp_ma_per_rpm", "Probe Speed to Current Kp", "int"),
        ("direction_probe_output_sign", "Probe Output Sign", "int"),
        ("direction_probe_phase_duration_ms", "Probe Phase Duration (ms)", "int"),
        ("direction_probe_sweep_enable", "Probe Sweep Enable", "bool"),
        ("direction_probe_fixed_phase_index", "Probe Fixed Phase Index", "int"),
    ],
    "Sensored": [
        ("user_sensored_direction_sign", "Sensored Direction Sign", "int"),
        ("user_sensored_align_lock_elec_deg10", "Align Lock Elec Angle (deg10)", "int"),
        ("user_sensored_elec_trim_deg10", "Electrical Trim (deg10)", "int"),
        ("user_sensored_speed_zero_window_rpm", "Sensored Speed Zero Window", "int"),
        ("user_sensored_speed_lpf_shift", "Sensored Speed LPF Shift", "int"),
        ("user_sensored_align_current_a", "Align Current (A)", "int"),
        ("user_sensored_align_duration_ms", "Align Duration (ms)", "int"),
    ],
}

PROFILE_FIELD_KEYS = [
    "test_mode",
    "fixed_target_offset_deg10",
    "step_amplitude_deg10",
    "step_dwell_ms",
    "step_start_positive",
    "hold_enter_deg10",
    "hold_exit_deg10",
    "speed_zero_window_rpm",
    "speed_kp_rpm_per_deg",
    "position_current_ma_num",
    "position_current_ma_den",
    "damping_current_ma_per_rpm",
    "breakaway_current_ma",
    "breakaway_speed_window_rpm",
    "speed_ki_divisor",
    "speed_ki_enable",
    "speed_kd_rpm_per_rpm",
    "speed_to_current_kp_ma_per_rpm",
    "outer_angle_filter_shift",
    "target_angle_slew_min_deg10_per_tick",
    "target_angle_slew_max_deg10_per_tick",
    "target_angle_slew_divisor",
    "target_angle_slew_accel_deg10_per_tick",
    "target_angle_slew_decel_deg10_per_tick",
    "settle_zone_deg10",
    "settle_max_rpm",
    "settle_max_current_ma",
    "settle_min_current_ma",
    "max_rpm",
    "min_rpm",
    "max_current_ma",
    "current_zero_window_ma",
    "source_direction_sign",
    "follower_direction_sign",
    "direction_probe_target_abs_rpm",
    "direction_probe_max_current_ma",
    "direction_probe_speed_to_current_kp_ma_per_rpm",
    "direction_probe_output_sign",
    "direction_probe_phase_duration_ms",
    "direction_probe_sweep_enable",
    "direction_probe_fixed_phase_index",
    "user_sensored_direction_sign",
    "user_sensored_align_lock_elec_deg10",
    "user_sensored_elec_trim_deg10",
    "user_sensored_speed_zero_window_rpm",
    "user_sensored_speed_lpf_shift",
    "user_sensored_align_current_a",
    "user_sensored_align_duration_ms",
]
PROFILE_STRUCT = struct.Struct("<" + ("i" * len(PROFILE_FIELD_KEYS)))

HELLO_FIELD_KEYS = [
    "protocol_version",
    "profile_size",
    "snapshot_size",
    "active_test_mode",
]
HELLO_STRUCT = struct.Struct("<4i")

SNAPSHOT_FIELD_KEYS = [
    "protocol_version",
    "test_mode",
    "command_angle_deg10",
    "target_angle_deg10",
    "actual_angle_deg10",
    "angle_error_deg10",
    "target_rpm",
    "filtered_speed_rpm",
    "integral_rpm",
    "current_command_ma",
    "active",
    "hold_active",
    "source_online",
    "follower_online",
    "settle_active",
    "knob_online",
    "knob_angle_deg10",
    "knob_raw_angle",
    "follower_raw_angle",
    "follower_angle_deg10",
    "follower_status",
    "follower_magnet_detected",
    "follower_magnet_too_weak",
    "follower_magnet_too_strong",
    "sensored_online",
    "sensored_aligned",
    "sensored_mechanical_angle_deg10",
    "sensored_electrical_angle_deg10",
    "sensored_mechanical_speed_rpm",
    "sensored_runtime_direction_sign",
    "sensored_runtime_electrical_trim_deg10",
]
SNAPSHOT_STRUCT = struct.Struct("<" + ("i" * len(SNAPSHOT_FIELD_KEYS)))

MCP_STATUS_NAMES = {
    0x00: "MCP_CMD_OK",
    0x01: "MCP_CMD_NOK",
    0x02: "MCP_CMD_UNKNOWN",
    0x03: "MCP_DATAID_UNKNOWN",
    0x04: "MCP_ERROR_RO_REG",
    0x05: "MCP_ERROR_UNKNOWN_REG",
    0x06: "MCP_ERROR_STRING_FORMAT",
    0x07: "MCP_ERROR_BAD_DATA_TYPE",
    0x08: "MCP_ERROR_NO_TXSYNC_SPACE",
    0x09: "MCP_ERROR_NO_TXASYNC_SPACE",
    0x0A: "MCP_ERROR_BAD_RAW_FORMAT",
    0x0B: "MCP_ERROR_WO_REG",
    0x0C: "MCP_ERROR_REGISTER_ACCESS",
    0x0D: "MCP_ERROR_CALLBACK_NOT_REGISTRED",
}

CRC4_LOOKUP8 = (
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x07, 0x05, 0x03, 0x01, 0x0F, 0x0D, 0x0B, 0x09,
    0x07, 0x05, 0x03, 0x01, 0x0F, 0x0D, 0x0B, 0x09, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
    0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0x00, 0x09, 0x0B, 0x0D, 0x0F, 0x01, 0x03, 0x05, 0x07,
    0x09, 0x0B, 0x0D, 0x0F, 0x01, 0x03, 0x05, 0x07, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0x00,
    0x0B, 0x09, 0x0F, 0x0D, 0x03, 0x01, 0x07, 0x05, 0x0C, 0x0E, 0x08, 0x0A, 0x04, 0x06, 0x00, 0x02,
    0x0C, 0x0E, 0x08, 0x0A, 0x04, 0x06, 0x00, 0x02, 0x0B, 0x09, 0x0F, 0x0D, 0x03, 0x01, 0x07, 0x05,
    0x05, 0x07, 0x01, 0x03, 0x0D, 0x0F, 0x09, 0x0B, 0x02, 0x00, 0x06, 0x04, 0x0A, 0x08, 0x0E, 0x0C,
    0x02, 0x00, 0x06, 0x04, 0x0A, 0x08, 0x0E, 0x0C, 0x05, 0x07, 0x01, 0x03, 0x0D, 0x0F, 0x09, 0x0B,
    0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x06, 0x04, 0x02, 0x00, 0x0E, 0x0C, 0x0A, 0x08,
    0x06, 0x04, 0x02, 0x00, 0x0E, 0x0C, 0x0A, 0x08, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F,
    0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x03, 0x01, 0x08, 0x0A, 0x0C, 0x0E, 0x00, 0x02, 0x04, 0x06,
    0x08, 0x0A, 0x0C, 0x0E, 0x00, 0x02, 0x04, 0x06, 0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x03, 0x01,
    0x0A, 0x08, 0x0E, 0x0C, 0x02, 0x00, 0x06, 0x04, 0x0D, 0x0F, 0x09, 0x0B, 0x05, 0x07, 0x01, 0x03,
    0x0D, 0x0F, 0x09, 0x0B, 0x05, 0x07, 0x01, 0x03, 0x0A, 0x08, 0x0E, 0x0C, 0x02, 0x00, 0x06, 0x04,
    0x04, 0x06, 0x00, 0x02, 0x0C, 0x0E, 0x08, 0x0A, 0x03, 0x01, 0x07, 0x05, 0x0B, 0x09, 0x0F, 0x0D,
    0x03, 0x01, 0x07, 0x05, 0x0B, 0x09, 0x0F, 0x0D, 0x04, 0x06, 0x00, 0x02, 0x0C, 0x0E, 0x08, 0x0A,
)
CRC4_LOOKUP4 = (0x00, 0x07, 0x0E, 0x09, 0x0B, 0x0C, 0x05, 0x02, 0x01, 0x06, 0x0F, 0x08, 0x0A, 0x0D, 0x04, 0x03)


def load_profile() -> dict:
    return json.loads(PROFILE_PATH.read_text(encoding="utf-8"))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def list_preset_paths() -> list[Path]:
    return sorted(PRESET_DIR.glob("*.json"))


def list_serial_ports() -> list:
    if list_ports is None:
        return []
    return sorted(list_ports.comports(), key=lambda port: port.device)


def bool_to_int(value: str) -> int:
    return 1 if str(value).strip().lower() in {"1", "true", "yes", "on"} else 0


def pack_profile(profile: dict) -> bytes:
    numeric_values = []
    for key in PROFILE_FIELD_KEYS:
        value = profile[key]
        if key == "test_mode":
            numeric_values.append(TEST_MODE_VALUES[value])
        else:
            numeric_values.append(int(value))
    return PROFILE_STRUCT.pack(*numeric_values)


def unpack_profile(payload: bytes) -> dict:
    if len(payload) != PROFILE_STRUCT.size:
        raise ValueError(f"Profile payload size mismatch: expected {PROFILE_STRUCT.size}, got {len(payload)}")
    values = PROFILE_STRUCT.unpack(payload)
    profile = dict(zip(PROFILE_FIELD_KEYS, values))
    profile["test_mode"] = TEST_MODE_NAMES.get(profile["test_mode"], "normal_follow")
    return profile


def unpack_named_struct(field_names: list[str], struct_def: struct.Struct, payload: bytes) -> dict:
    if len(payload) != struct_def.size:
        raise ValueError(f"Payload size mismatch: expected {struct_def.size}, got {len(payload)}")
    return dict(zip(field_names, struct_def.unpack(payload)))


def format_snapshot(snapshot: dict) -> str:
    test_mode_name = TEST_MODE_NAMES.get(snapshot["test_mode"], str(snapshot["test_mode"]))
    phase_label = "Probe Phase Index" if snapshot["test_mode"] == TEST_MODE_VALUES["direction_probe"] else "Settle Active"
    lines = [
        f"Protocol Version: {snapshot['protocol_version']}",
        f"Test Mode: {test_mode_name}",
        "",
        f"Command Angle: {snapshot['command_angle_deg10']} deg10",
        f"Target Angle: {snapshot['target_angle_deg10']} deg10",
        f"Actual Angle: {snapshot['actual_angle_deg10']} deg10",
        f"Angle Error: {snapshot['angle_error_deg10']} deg10",
        f"Target RPM: {snapshot['target_rpm']}",
        f"Filtered Speed RPM: {snapshot['filtered_speed_rpm']}",
        f"Integral RPM: {snapshot['integral_rpm']}",
        f"Current Command: {snapshot['current_command_ma']} mA",
        "",
        f"Follow Active: {snapshot['active']}",
        f"Hold Active: {snapshot['hold_active']}",
        f"Source Online: {snapshot['source_online']}",
        f"Follower Online: {snapshot['follower_online']}",
        f"{phase_label}: {snapshot['settle_active']}",
        "",
        f"Knob Online: {snapshot['knob_online']}",
        f"Knob Angle: {snapshot['knob_angle_deg10']} deg10",
        f"Knob Raw: {snapshot['knob_raw_angle']}",
        "",
        f"Follower Angle: {snapshot['follower_angle_deg10']} deg10",
        f"Follower Raw: {snapshot['follower_raw_angle']}",
        f"Follower Status: 0x{snapshot['follower_status']:02X}",
        f"Follower Magnet Detected: {snapshot['follower_magnet_detected']}",
        f"Follower Magnet Too Weak: {snapshot['follower_magnet_too_weak']}",
        f"Follower Magnet Too Strong: {snapshot['follower_magnet_too_strong']}",
        "",
        f"Sensored Online: {snapshot['sensored_online']}",
        f"Sensored Aligned: {snapshot['sensored_aligned']}",
        f"Sensored Mechanical Angle: {snapshot['sensored_mechanical_angle_deg10']} deg10",
        f"Sensored Electrical Angle: {snapshot['sensored_electrical_angle_deg10']} deg10",
        f"Sensored Mechanical Speed: {snapshot['sensored_mechanical_speed_rpm']} rpm",
        f"Runtime Direction Sign: {snapshot['sensored_runtime_direction_sign']}",
        f"Runtime Electrical Trim: {snapshot['sensored_runtime_electrical_trim_deg10']} deg10",
    ]
    return "\n".join(lines)


class FollowTuningProtocolError(RuntimeError):
    pass


class AutoScrollbar(ttk.Scrollbar):
    def set(self, first, last):
        if (float(first) <= 0.0) and (float(last) >= 1.0):
            self.grid_remove()
        else:
            self.grid()
        super().set(first, last)


class AspepMcpClient:
    PACKET_DATA = 0x9
    PACKET_ACK = 0xA
    PACKET_PING = 0x6
    PACKET_BEACON = 0x5
    PACKET_NACK = 0xF
    PACKET_ID_MASK = 0xF

    CAP_VERSION = 0
    CAP_DATA_CRC = 0
    CAP_RX_MAX_SIZE = 7
    CAP_TXS_MAX_SIZE = 7
    CAP_TXA_MAX_SIZE = 32

    def __init__(self, port: str, baudrate: int = 1843200, timeout: float = 0.35):
        if serial is None:
            raise RuntimeError("pyserial is not installed in the current Python environment")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None

    @staticmethod
    def _compute_header_crc(header: int) -> int:
        header &= 0x0FFFFFFF
        crc = 0
        crc = CRC4_LOOKUP8[crc ^ (header & 0xFF)]
        crc = CRC4_LOOKUP8[crc ^ ((header >> 8) & 0xFF)]
        crc = CRC4_LOOKUP8[crc ^ ((header >> 16) & 0xFF)]
        crc = CRC4_LOOKUP4[crc ^ ((header >> 24) & 0x0F)]
        return header | (crc << 28)

    @staticmethod
    def _validate_header_crc(header: int) -> bool:
        crc = 0
        crc = CRC4_LOOKUP8[crc ^ (header & 0xFF)]
        crc = CRC4_LOOKUP8[crc ^ ((header >> 8) & 0xFF)]
        crc = CRC4_LOOKUP8[crc ^ ((header >> 16) & 0xFF)]
        crc = CRC4_LOOKUP8[crc ^ ((header >> 24) & 0xFF)]
        return crc == 0

    def _build_beacon_header(self) -> int:
        header = (
            self.PACKET_BEACON
            | (self.CAP_VERSION << 4)
            | (self.CAP_DATA_CRC << 7)
            | (self.CAP_RX_MAX_SIZE << 8)
            | (self.CAP_TXS_MAX_SIZE << 14)
            | (self.CAP_TXA_MAX_SIZE << 21)
        )
        return self._compute_header_crc(header)

    def _build_ping_header(self, packet_number: int = 0) -> int:
        header = self.PACKET_PING | ((packet_number & 0xFFFF) << 12)
        return self._compute_header_crc(header)

    def _build_data_header(self, payload_length: int) -> int:
        header = ((payload_length & 0x1FFF) << 4) | self.PACKET_DATA
        return self._compute_header_crc(header)

    def _read_exact(self, size: int) -> bytes:
        if self.serial_port is None:
            raise FollowTuningProtocolError("Serial port is not open")
        data = bytearray()
        while len(data) < size:
            chunk = self.serial_port.read(size - len(data))
            if not chunk:
                raise FollowTuningProtocolError(f"Timeout waiting for {size} bytes, received {len(data)}")
            data.extend(chunk)
        return bytes(data)

    def _read_packet(self) -> tuple[int, bytes, int]:
        header_bytes = self._read_exact(4)
        header = struct.unpack("<I", header_bytes)[0]
        if not self._validate_header_crc(header):
            raise FollowTuningProtocolError(f"Invalid ASPEP header CRC: 0x{header:08X}")

        packet_type = header & self.PACKET_ID_MASK
        if packet_type == self.PACKET_NACK:
            error_info = (header >> 8) & 0xFF
            raise FollowTuningProtocolError(f"Received ASPEP NACK with error {error_info}")

        if packet_type in {self.PACKET_DATA, self.PACKET_ACK}:
            payload_length = (header >> 4) & 0x1FFF
            payload = self._read_exact(payload_length)
            return packet_type, payload, header

        return packet_type, b"", header

    def _send_control_header(self, header: int) -> None:
        if self.serial_port is None:
            raise FollowTuningProtocolError("Serial port is not open")
        self.serial_port.write(struct.pack("<I", header))
        self.serial_port.flush()

    def _send_request(self, payload: bytes) -> None:
        if self.serial_port is None:
            raise FollowTuningProtocolError("Serial port is not open")
        self.serial_port.write(struct.pack("<I", self._build_data_header(len(payload))))
        self.serial_port.flush()
        if payload:
            time.sleep(ASPEP_DATA_PAYLOAD_GAP_SECONDS)
            self.serial_port.write(payload)
        self.serial_port.flush()

    def connect(self) -> dict:
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.05)

            last_error = None
            packet_type = None
            header = 0

            for _ in range(3):
                try:
                    self._send_control_header(self._build_beacon_header())
                    packet_type, _payload, header = self._read_packet()
                    if packet_type == self.PACKET_BEACON:
                        break
                except FollowTuningProtocolError as exc:
                    last_error = exc
                    self.serial_port.reset_input_buffer()
                    time.sleep(0.05)
            else:
                if last_error is not None:
                    raise last_error
                raise FollowTuningProtocolError(
                    f"Expected BEACON after connect, got packet type 0x{packet_type:X} header 0x{header:08X}"
                )

            last_error = None
            for _ in range(3):
                try:
                    self._send_control_header(self._build_ping_header())
                    packet_type, _payload, header = self._read_packet()
                    if packet_type == self.PACKET_PING:
                        break
                except FollowTuningProtocolError as exc:
                    last_error = exc
                    self.serial_port.reset_input_buffer()
                    time.sleep(0.05)
            else:
                if last_error is not None:
                    raise last_error
                raise FollowTuningProtocolError(
                    f"Expected PING after beacon, got packet type 0x{packet_type:X} header 0x{header:08X}"
                )

            try:
                return self.ping()
            except FollowTuningProtocolError as exc:
                raise FollowTuningProtocolError(
                    "ASPEP control handshake succeeded, but MCP runtime commands did not answer. "
                    "Flash the latest firmware with VS Code Debug, then reconnect."
                ) from exc
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None

    def _transceive_user_command(self, opcode: int, payload: bytes = b"") -> bytes:
        mcp_header = 0x0101 | (FOLLOW_TUNING_MCP_CALLBACK_ID << 3)
        request_payload = struct.pack("<H", mcp_header) + bytes([opcode]) + payload
        self._send_request(request_payload)
        packet_type, response_payload, header = self._read_packet()
        if packet_type != self.PACKET_ACK:
            raise FollowTuningProtocolError(f"Expected ACK packet, got 0x{packet_type:X} header 0x{header:08X}")
        if not response_payload:
            raise FollowTuningProtocolError("Empty MCP response payload")
        status = response_payload[-1]
        if status != 0:
            status_name = MCP_STATUS_NAMES.get(status, f"0x{status:02X}")
            raise FollowTuningProtocolError(f"Board returned MCP status {status_name}")
        return response_payload[:-1]

    def ping(self) -> dict:
        payload = self._transceive_user_command(FOLLOW_TUNING_MCP_OPCODE_PING)
        hello = unpack_named_struct(HELLO_FIELD_KEYS, HELLO_STRUCT, payload)
        if hello["protocol_version"] != FOLLOW_TUNING_PROTOCOL_VERSION:
            raise FollowTuningProtocolError(
                f"Protocol version mismatch: board={hello['protocol_version']} tool={FOLLOW_TUNING_PROTOCOL_VERSION}"
            )
        if hello["profile_size"] != PROFILE_STRUCT.size:
            raise FollowTuningProtocolError(
                f"Profile size mismatch: board={hello['profile_size']} tool={PROFILE_STRUCT.size}"
            )
        if hello["snapshot_size"] != SNAPSHOT_STRUCT.size:
            raise FollowTuningProtocolError(
                f"Snapshot size mismatch: board={hello['snapshot_size']} tool={SNAPSHOT_STRUCT.size}"
            )
        return hello

    def read_profile(self) -> dict:
        payload = self._transceive_user_command(FOLLOW_TUNING_MCP_OPCODE_GET_PROFILE)
        return unpack_profile(payload)

    def write_profile(self, profile: dict) -> dict:
        payload = self._transceive_user_command(FOLLOW_TUNING_MCP_OPCODE_SET_PROFILE, pack_profile(profile))
        return unpack_profile(payload)

    def reset_profile(self) -> dict:
        payload = self._transceive_user_command(FOLLOW_TUNING_MCP_OPCODE_RESET_PROFILE)
        return unpack_profile(payload)

    def read_snapshot(self) -> dict:
        payload = self._transceive_user_command(FOLLOW_TUNING_MCP_OPCODE_GET_SNAPSHOT)
        return unpack_named_struct(SNAPSHOT_FIELD_KEYS, SNAPSHOT_STRUCT, payload)


def generate_header(profile: dict) -> str:
    lines = [
        "#ifndef TUNING_CONFIG_H",
        "#define TUNING_CONFIG_H",
        "",
        "/*",
        " * Generated by tools/follow_tuning_tool.py from tools/follow_tuning_profile.json.",
        " * Do not hand-edit unless the GUI tool is unavailable.",
        " */",
        "",
        "#define FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW 0",
        "#define FOLLOW_CONTROL_TEST_MODE_FIXED_TARGET 1",
        "#define FOLLOW_CONTROL_TEST_MODE_STEP_SEQUENCE 2",
        "#define FOLLOW_CONTROL_TEST_MODE_DIRECTION_PROBE 3",
        "",
        f"#define FOLLOW_CONTROL_TEST_MODE {TEST_MODE_MACROS[profile['test_mode']]}",
        "",
        f"#define FOLLOW_CONTROL_FIXED_TARGET_OFFSET_DEG10 {profile['fixed_target_offset_deg10']}",
        f"#define FOLLOW_CONTROL_STEP_AMPLITUDE_DEG10 {profile['step_amplitude_deg10']}",
        f"#define FOLLOW_CONTROL_STEP_DWELL_MS {profile['step_dwell_ms']}U",
        f"#define FOLLOW_CONTROL_STEP_START_POSITIVE {profile['step_start_positive']}",
        "",
        f"#define FOLLOW_CONTROL_HOLD_ENTER_DEG10 {profile['hold_enter_deg10']}",
        f"#define FOLLOW_CONTROL_HOLD_EXIT_DEG10 {profile['hold_exit_deg10']}",
        f"#define FOLLOW_CONTROL_SPEED_ZERO_WINDOW_RPM {profile['speed_zero_window_rpm']}",
        f"#define FOLLOW_CONTROL_SPEED_KP_RPM_PER_DEG {profile['speed_kp_rpm_per_deg']}",
        f"#define FOLLOW_CONTROL_POSITION_CURRENT_MA_NUM {profile['position_current_ma_num']}",
        f"#define FOLLOW_CONTROL_POSITION_CURRENT_MA_DEN {profile['position_current_ma_den']}",
        f"#define FOLLOW_CONTROL_DAMPING_CURRENT_MA_PER_RPM {profile['damping_current_ma_per_rpm']}",
        f"#define FOLLOW_CONTROL_BREAKAWAY_CURRENT_MA {profile['breakaway_current_ma']}",
        f"#define FOLLOW_CONTROL_BREAKAWAY_SPEED_WINDOW_RPM {profile['breakaway_speed_window_rpm']}",
        f"#define FOLLOW_CONTROL_SPEED_KI_DIVISOR {profile['speed_ki_divisor']}",
        f"#define FOLLOW_CONTROL_SPEED_KI_ENABLE {profile['speed_ki_enable']}",
        f"#define FOLLOW_CONTROL_SPEED_KD_RPM_PER_RPM {profile['speed_kd_rpm_per_rpm']}",
        f"#define FOLLOW_CONTROL_SPEED_TO_CURRENT_KP_MA_PER_RPM {profile['speed_to_current_kp_ma_per_rpm']}",
        f"#define FOLLOW_CONTROL_OUTER_ANGLE_FILTER_SHIFT {profile['outer_angle_filter_shift']}",
        f"#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MIN_DEG10_PER_TICK {profile['target_angle_slew_min_deg10_per_tick']}",
        f"#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MAX_DEG10_PER_TICK {profile['target_angle_slew_max_deg10_per_tick']}",
        f"#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DIVISOR {profile['target_angle_slew_divisor']}",
        f"#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_ACCEL_DEG10_PER_TICK {profile['target_angle_slew_accel_deg10_per_tick']}",
        f"#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DECEL_DEG10_PER_TICK {profile['target_angle_slew_decel_deg10_per_tick']}",
        f"#define FOLLOW_CONTROL_SETTLE_ZONE_DEG10 {profile['settle_zone_deg10']}",
        f"#define FOLLOW_CONTROL_SETTLE_MAX_RPM {profile['settle_max_rpm']}",
        f"#define FOLLOW_CONTROL_SETTLE_MAX_CURRENT_MA {profile['settle_max_current_ma']}",
        f"#define FOLLOW_CONTROL_SETTLE_MIN_CURRENT_MA {profile['settle_min_current_ma']}",
        f"#define FOLLOW_CONTROL_MAX_RPM {profile['max_rpm']}",
        f"#define FOLLOW_CONTROL_MIN_RPM {profile['min_rpm']}",
        f"#define FOLLOW_CONTROL_MAX_CURRENT_MA {profile['max_current_ma']}",
        f"#define FOLLOW_CONTROL_CURRENT_ZERO_WINDOW_MA {profile['current_zero_window_ma']}",
        f"#define FOLLOW_CONTROL_SOURCE_DIRECTION_SIGN {profile['source_direction_sign']}",
        f"#define FOLLOW_CONTROL_FOLLOWER_DIRECTION_SIGN {profile['follower_direction_sign']}",
        "",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_TARGET_ABS_RPM {profile['direction_probe_target_abs_rpm']}",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_MAX_CURRENT_MA {profile['direction_probe_max_current_ma']}",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_SPEED_TO_CURRENT_KP_MA_PER_RPM {profile['direction_probe_speed_to_current_kp_ma_per_rpm']}",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_OUTPUT_SIGN {profile['direction_probe_output_sign']}",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_DURATION_MS {profile['direction_probe_phase_duration_ms']}U",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_SWEEP_ENABLE {profile['direction_probe_sweep_enable']}",
        f"#define FOLLOW_CONTROL_DIRECTION_PROBE_FIXED_PHASE_INDEX {profile['direction_probe_fixed_phase_index']}U",
        "",
        f"#define USER_SENSORED_DIRECTION_SIGN {profile['user_sensored_direction_sign']}",
        f"#define USER_SENSORED_ALIGN_LOCK_ELEC_DEG10 {profile['user_sensored_align_lock_elec_deg10']}",
        f"#define USER_SENSORED_ELEC_TRIM_DEG10 {profile['user_sensored_elec_trim_deg10']}",
        f"#define USER_SENSORED_SPEED_ZERO_WINDOW_RPM {profile['user_sensored_speed_zero_window_rpm']}",
        f"#define USER_SENSORED_SPEED_LPF_SHIFT {profile['user_sensored_speed_lpf_shift']}",
        f"#define USER_SENSORED_ALIGN_CURRENT_A {profile['user_sensored_align_current_a']}",
        f"#define USER_SENSORED_ALIGN_DURATION_MS {profile['user_sensored_align_duration_ms']}U",
        "",
        "#endif /* TUNING_CONFIG_H */",
        "",
    ]
    return "\n".join(lines)


class FollowTuningTool(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Follow Tuning Tool")
        self.geometry("1180x860")
        self.minsize(640, 480)
        self.output_queue = queue.Queue()
        self.variables = {}
        self.preset_files = {}
        self.port_map = {}
        self.profile = load_profile()
        self.preset_var = tk.StringVar(value="")
        self.port_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Ready")
        self.live_client = None
        self.live_operation_lock = threading.Lock()
        self.live_poll_enabled = False
        self.live_poll_inflight = False
        self._build_scroll_container()
        self._build_ui()
        self._refresh_preset_list()
        self._refresh_port_list()
        self._load_initial_profile()
        self.after_idle(self._fit_scroll_content_to_canvas)
        self.after(100, self._poll_output_queue)

    def _build_scroll_container(self):
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        viewport = ttk.Frame(self)
        viewport.grid(row=0, column=0, sticky=tk.NSEW)
        viewport.rowconfigure(0, weight=1)
        viewport.columnconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(viewport, highlightthickness=0)
        self.vertical_scrollbar = AutoScrollbar(viewport, orient=tk.VERTICAL, command=self.scroll_canvas.yview)
        self.horizontal_scrollbar = AutoScrollbar(viewport, orient=tk.HORIZONTAL, command=self.scroll_canvas.xview)
        self.scroll_canvas.configure(
            xscrollcommand=self.horizontal_scrollbar.set,
            yscrollcommand=self.vertical_scrollbar.set,
        )

        self.scroll_canvas.grid(row=0, column=0, sticky=tk.NSEW)
        self.vertical_scrollbar.grid(row=0, column=1, sticky=tk.NS)
        self.horizontal_scrollbar.grid(row=1, column=0, sticky=tk.EW)

        self.scroll_content = ttk.Frame(self.scroll_canvas)
        self.scroll_window = self.scroll_canvas.create_window((0, 0), window=self.scroll_content, anchor=tk.NW)

        self.scroll_content.bind("<Configure>", self._on_scroll_content_configure)
        self.scroll_canvas.bind("<Configure>", self._on_scroll_canvas_configure)

    def _on_scroll_content_configure(self, _event):
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))
        self.after_idle(self._fit_scroll_content_to_canvas)

    def _on_scroll_canvas_configure(self, _event):
        self.after_idle(self._fit_scroll_content_to_canvas)

    def _fit_scroll_content_to_canvas(self):
        canvas_width = self.scroll_canvas.winfo_width()
        canvas_height = self.scroll_canvas.winfo_height()

        if (canvas_width <= 1) or (canvas_height <= 1):
            return

        content_req_height = self.scroll_content.winfo_reqheight()
        target_width = max(canvas_width, BASE_LAYOUT_WIDTH)
        target_height = max(canvas_height, BASE_LAYOUT_HEIGHT, content_req_height)

        self.scroll_canvas.itemconfigure(
            self.scroll_window,
            width=target_width,
            height=target_height,
        )
        self.scroll_canvas.configure(scrollregion=(0, 0, target_width, target_height))

    def _build_ui(self):
        toolbar = ttk.Frame(self.scroll_content, padding=8)
        toolbar.pack(fill=tk.X)

        toolbar.columnconfigure(2, weight=1)
        toolbar.columnconfigure(3, weight=1)

        ttk.Button(toolbar, text="Reload", command=self.reload_profile).grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        ttk.Label(toolbar, text="Preset").grid(row=0, column=1, sticky=tk.W, padx=(16, 4), pady=4)
        self.preset_combo = ttk.Combobox(
            toolbar,
            textvariable=self.preset_var,
            state="readonly",
            width=28,
        )
        self.preset_combo.grid(row=0, column=2, sticky=tk.EW, padx=4, pady=4)
        ttk.Button(toolbar, text="Load Preset", command=self.load_selected_preset).grid(row=0, column=3, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Refresh Presets", command=self._refresh_preset_list).grid(row=0, column=4, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Save Profile", command=self.save_profile).grid(row=0, column=5, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Save and Build Debug", command=self.save_and_build).grid(row=0, column=6, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Open Test Plan", command=self.open_test_plan).grid(row=0, column=7, sticky=tk.W, padx=4, pady=4)

        ttk.Label(toolbar, text="COM").grid(row=1, column=1, sticky=tk.W, padx=(16, 4), pady=4)
        self.port_combo = ttk.Combobox(toolbar, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.grid(row=1, column=2, sticky=tk.EW, padx=4, pady=4)
        ttk.Button(toolbar, text="Refresh COM", command=self._refresh_port_list).grid(row=1, column=3, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Connect", command=self.connect_live).grid(row=1, column=4, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Disconnect", command=self.disconnect_live).grid(row=1, column=5, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Read Live", command=self.read_live_state).grid(row=1, column=6, sticky=tk.W, padx=4, pady=4)
        ttk.Button(toolbar, text="Apply Live", command=self.apply_live_profile).grid(row=1, column=7, sticky=tk.W, padx=4, pady=4)

        main = ttk.Panedwindow(self.scroll_content, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(main)
        right = ttk.Frame(main)
        main.add(left, weight=3)
        main.add(right, weight=2)

        notebook = ttk.Notebook(left)
        notebook.pack(fill=tk.BOTH, expand=True)

        for section_name, fields in SECTIONS.items():
            frame = ttk.Frame(notebook, padding=10)
            notebook.add(frame, text=section_name)
            self._build_section(frame, fields)

        info_frame = ttk.LabelFrame(right, text="Suggested Test Order", padding=10)
        info_frame.pack(fill=tk.BOTH, expand=False)
        info_text = self._create_scrollable_text_widget(info_frame, wrap=tk.WORD, height=20)
        info_text.insert(
            tk.END,
            "1. direction_probe: only when inner FOC direction/trim is suspicious again.\n\n"
            "2. 01_fixed_target_hold: verify spring return around one target without knob input.\n\n"
            "3. 02_fixed_target_offset_150: verify static offset hold without knob noise.\n\n"
            "4. 03_step_small_120: 120 deg10 for near-target damping.\n\n"
            "5. 04_step_medium_360: 360 deg10 for large-angle braking.\n\n"
            "6. 05_normal_follow: only after step modes converge.\n\n"
            "Use the tool to load a preset, save/build, then press VS Code Debug to flash."
        )
        info_text.configure(state=tk.DISABLED)

        live_frame = ttk.LabelFrame(right, text="Live Snapshot", padding=10)
        live_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.live_text = self._create_scrollable_text_widget(live_frame, wrap=tk.WORD, height=18)
        self.live_text.insert(tk.END, "Disconnected\n")
        self.live_text.configure(state=tk.DISABLED)

        output_frame = ttk.LabelFrame(right, text="Build Output", padding=10)
        output_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        output_actions = ttk.Frame(output_frame)
        output_actions.pack(fill=tk.X, pady=(0, 6))
        self.output_clear_button = ttk.Button(output_actions, text="Clear Output", command=self.clear_output_text)
        self.output_clear_button.pack(side=tk.RIGHT)
        self.output_text = self._create_scrollable_text_widget(output_frame, wrap=tk.NONE, height=18, horizontal=True)

        status = ttk.Label(self.scroll_content, textvariable=self.status_var, anchor=tk.W)
        status.pack(fill=tk.X, padx=8, pady=(0, 8))

    def _create_scrollable_text_widget(self, parent, *, wrap, height, horizontal=False):
        container = ttk.Frame(parent)
        container.pack(fill=tk.BOTH, expand=True)
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        text_widget = tk.Text(container, wrap=wrap, height=height)
        vertical_scrollbar = ttk.Scrollbar(container, orient=tk.VERTICAL, command=text_widget.yview)
        text_widget.configure(yscrollcommand=vertical_scrollbar.set)

        text_widget.grid(row=0, column=0, sticky=tk.NSEW)
        vertical_scrollbar.grid(row=0, column=1, sticky=tk.NS)

        if horizontal:
            horizontal_scrollbar = ttk.Scrollbar(container, orient=tk.HORIZONTAL, command=text_widget.xview)
            text_widget.configure(xscrollcommand=horizontal_scrollbar.set)
            horizontal_scrollbar.grid(row=1, column=0, sticky=tk.EW)

        return text_widget

    def _build_section(self, parent, fields):
        for row, (key, label, field_type) in enumerate(fields):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 12), pady=4)
            if field_type == "enum":
                variable = tk.StringVar()
                widget = ttk.Combobox(
                    parent,
                    textvariable=variable,
                    values=list(TEST_MODE_MACROS.keys()),
                    state="readonly",
                    width=28,
                )
            else:
                variable = tk.StringVar()
                widget = ttk.Entry(parent, textvariable=variable, width=30)
            widget.grid(row=row, column=1, sticky=tk.EW, pady=4)
            self.variables[key] = variable

        parent.columnconfigure(1, weight=1)

    def _load_into_widgets(self, profile: dict):
        for key, variable in self.variables.items():
            variable.set(str(profile.get(key, "")))

    def _load_initial_profile(self):
        if RECOMMENDED_START_PRESET in self.preset_files:
            self.preset_var.set(RECOMMENDED_START_PRESET)
            self.load_selected_preset()
        else:
            self._load_into_widgets(self.profile)

    def _refresh_preset_list(self):
        preset_paths = list_preset_paths()
        self.preset_files = {path.stem: path for path in preset_paths}
        preset_names = list(self.preset_files.keys())
        self.preset_combo["values"] = preset_names
        if preset_names:
            preferred_preset = RECOMMENDED_START_PRESET if RECOMMENDED_START_PRESET in self.preset_files else preset_names[0]
            if self.preset_var.get() not in self.preset_files:
                self.preset_var.set(preferred_preset)
            self.status_var.set(f"Loaded preset list: {len(preset_names)} presets available")
        else:
            self.preset_var.set("")
            self.status_var.set("No preset files found")

    def _refresh_port_list(self):
        if list_ports is None:
            self.port_map = {}
            self.port_combo["values"] = []
            self.port_var.set("")
            self.status_var.set("pyserial not available: live COM features are disabled")
            return

        ports = list_serial_ports()
        self.port_map = {f"{port.device} - {port.description}": port.device for port in ports}
        labels = list(self.port_map.keys())
        self.port_combo["values"] = labels

        if labels:
            if self.port_var.get() not in self.port_map:
                preferred = next((label for label in labels if self.port_map[label] == "COM4"), labels[0])
                self.port_var.set(preferred)
            self.status_var.set(f"Detected {len(labels)} serial port(s)")
        else:
            self.port_var.set("")
            self.status_var.set("No serial ports detected")

    def _selected_port_device(self) -> str:
        selected = self.port_var.get()
        return self.port_map.get(selected, selected.strip())

    def _set_live_text(self, content: str):
        self.live_text.configure(state=tk.NORMAL)
        self.live_text.delete("1.0", tk.END)
        self.live_text.insert(tk.END, content)
        self.live_text.configure(state=tk.DISABLED)

    def clear_output_text(self):
        self.output_text.delete("1.0", tk.END)
        self.status_var.set("Cleared build output")

    def _start_live_worker(self, worker, busy_message: str = "Another live operation is still running", error_context: str = "operation"):
        thread = threading.Thread(target=self._run_live_worker, args=(worker, busy_message, error_context), daemon=True)
        thread.start()

    def _run_live_worker(self, worker, busy_message: str, error_context: str):
        if not self.live_operation_lock.acquire(blocking=False):
            if busy_message:
                self.output_queue.put(("status", busy_message))
            return
        try:
            worker()
        except Exception as exc:
            self.output_queue.put(("live-error", error_context, str(exc)))
        finally:
            self.live_operation_lock.release()

    def connect_live(self):
        if serial is None:
            messagebox.showerror("Live Connect Failed", "pyserial is not installed")
            return

        if self.live_client is not None:
            self.status_var.set("Live link is already connected")
            return

        port = self._selected_port_device()
        if not port:
            messagebox.showerror("Live Connect Failed", "Please choose a COM port")
            return

        self.status_var.set(f"Connecting to {port}...")
        self._set_live_text(f"Connecting to {port}...\n")

        def worker():
            client = AspepMcpClient(port)
            hello = client.connect()
            profile = client.read_profile()
            snapshot = client.read_snapshot()
            self.output_queue.put(("live-connected", {"client": client, "port": port, "hello": hello}))
            self.output_queue.put(("live-profile", profile))
            self.output_queue.put(("snapshot", snapshot))
            self.output_queue.put(("live-log", f"Connected to {port}; protocol v{hello['protocol_version']}"))
            self.output_queue.put(("status", f"Connected to {port}"))

        self._start_live_worker(worker, error_context="connect")

    def disconnect_live(self):
        self.live_poll_enabled = False
        client = self.live_client
        self.live_client = None
        self.live_poll_inflight = False
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        self._set_live_text("Disconnected\n")
        self.status_var.set("Disconnected from live COM link")

    def read_live_state(self):
        if self.live_client is None:
            self.status_var.set("Live link is not connected")
            return

        self.status_var.set("Reading live profile and snapshot...")

        def worker():
            profile = self.live_client.read_profile()
            snapshot = self.live_client.read_snapshot()
            self.output_queue.put(("live-profile", profile))
            self.output_queue.put(("snapshot", snapshot))
            self.output_queue.put(("status", "Live profile and snapshot refreshed"))

        self._start_live_worker(worker, error_context="read")

    def apply_live_profile(self):
        if self.live_client is None:
            self.status_var.set("Live link is not connected")
            return

        try:
            profile = self.collect_profile()
        except Exception as exc:
            messagebox.showerror("Apply Live Failed", str(exc))
            self.status_var.set(f"Live apply failed: {exc}")
            return

        self.status_var.set("Applying runtime profile over COM...")

        def worker():
            applied_profile = self.live_client.write_profile(profile)
            snapshot = self.live_client.read_snapshot()
            self.output_queue.put(("live-profile", applied_profile))
            self.output_queue.put(("snapshot", snapshot))
            self.output_queue.put(("live-log", "Applied runtime profile over MCP/ASPEP"))
            self.output_queue.put(("status", "Runtime profile applied"))

        self._start_live_worker(worker, error_context="apply")

    def _poll_live_snapshot_loop(self):
        if (self.live_client is None) or (not self.live_poll_enabled):
            self.live_poll_inflight = False
            return

        if not self.live_poll_inflight:
            self.live_poll_inflight = True

            def worker():
                if self.live_client is None:
                    self.output_queue.put(("live-poll-finished", None))
                    return
                try:
                    snapshot = self.live_client.read_snapshot()
                    self.output_queue.put(("snapshot", snapshot))
                except Exception as exc:
                    self.output_queue.put(("live-disconnected", str(exc)))
                finally:
                    self.output_queue.put(("live-poll-finished", None))

            self._start_live_worker(worker, busy_message="", error_context="poll")

        self.after(500, self._poll_live_snapshot_loop)

    def load_selected_preset(self):
        preset_name = self.preset_var.get()
        if preset_name not in self.preset_files:
            messagebox.showerror("Preset Load Failed", "Please choose a valid preset")
            return

        try:
            preset_profile = load_json(self.preset_files[preset_name])
            self._load_into_widgets(preset_profile)
            self.status_var.set(f"Loaded preset: {preset_name}")
        except Exception as exc:
            messagebox.showerror("Preset Load Failed", str(exc))
            self.status_var.set(f"Preset load failed: {exc}")

    def collect_profile(self) -> dict:
        profile = {}
        for section_fields in SECTIONS.values():
            for key, _label, field_type in section_fields:
                value = self.variables[key].get().strip()
                if field_type == "enum":
                    if value not in TEST_MODE_MACROS:
                        raise ValueError(f"Invalid test mode: {value}")
                    profile[key] = value
                elif field_type == "bool":
                    profile[key] = bool_to_int(value)
                else:
                    profile[key] = int(value)

        if profile["position_current_ma_den"] == 0:
            raise ValueError("Position current denominator cannot be zero")
        if profile["step_dwell_ms"] <= 0:
            raise ValueError("Step dwell must be positive")
        if profile["target_angle_slew_divisor"] <= 0:
            raise ValueError("Slew divisor must be positive")
        return profile

    def save_profile(self) -> bool:
        try:
            profile = self.collect_profile()
            PROFILE_PATH.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
            HEADER_PATH.write_text(generate_header(profile), encoding="utf-8")
            self.profile = profile
            self.status_var.set("Saved profile and generated tuning_config.h")
            return True
        except Exception as exc:
            messagebox.showerror("Save Failed", str(exc))
            self.status_var.set(f"Save failed: {exc}")
            return False

    def reload_profile(self):
        try:
            self.profile = load_profile()
            self._load_into_widgets(self.profile)
            self.status_var.set("Reloaded profile from disk")
        except Exception as exc:
            messagebox.showerror("Reload Failed", str(exc))
            self.status_var.set(f"Reload failed: {exc}")

    def open_test_plan(self):
        try:
            subprocess.Popen(["notepad.exe", str(PLAN_PATH)])
        except Exception as exc:
            messagebox.showerror("Open Test Plan Failed", str(exc))

    def save_and_build(self):
        if not self.save_profile():
            return

        self.output_text.delete("1.0", tk.END)
        self.status_var.set("Building Debug preset...")

        thread = threading.Thread(target=self._run_build, daemon=True)
        thread.start()

    def _run_build(self):
        command = [
            "powershell",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(BUILD_SCRIPT),
            "-Action",
            "build",
            "-Preset",
            "Debug",
        ]

        try:
            process = subprocess.Popen(
                command,
                cwd=ROOT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )

            assert process.stdout is not None
            for line in process.stdout:
                self.output_queue.put(line)
            return_code = process.wait()
            self.output_queue.put(f"\nBuild finished with exit code {return_code}.\n")
            self.output_queue.put(("status", f"Build finished with exit code {return_code}"))
        except Exception as exc:
            self.output_queue.put(f"\nBuild failed to start: {exc}\n")
            self.output_queue.put(("status", f"Build failed to start: {exc}"))

    def _poll_output_queue(self):
        while not self.output_queue.empty():
            item = self.output_queue.get()
            if isinstance(item, tuple) and item[0] == "status":
                self.status_var.set(item[1])
            elif isinstance(item, tuple) and item[0] == "live-profile":
                self._load_into_widgets(item[1])
                self.profile = item[1]
            elif isinstance(item, tuple) and item[0] == "snapshot":
                self._set_live_text(format_snapshot(item[1]))
            elif isinstance(item, tuple) and item[0] == "live-log":
                self.output_text.insert(tk.END, str(item[1]) + "\n")
                self.output_text.see(tk.END)
            elif isinstance(item, tuple) and item[0] == "live-connected":
                self.live_client = item[1]["client"]
                self.live_poll_enabled = True
                self.live_poll_inflight = False
                self.after(500, self._poll_live_snapshot_loop)
            elif isinstance(item, tuple) and item[0] == "live-disconnected":
                error_message = item[1]
                self.disconnect_live()
                self.output_text.insert(tk.END, f"Live link closed: {error_message}\n")
                self.output_text.see(tk.END)
                self.status_var.set(f"Live link closed: {error_message}")
            elif isinstance(item, tuple) and item[0] == "live-poll-finished":
                self.live_poll_inflight = False
            elif isinstance(item, tuple) and item[0] == "live-error":
                error_context = item[1]
                error_message = item[2]
                self.output_text.insert(tk.END, f"Live operation failed ({error_context}): {error_message}\n")
                self.output_text.see(tk.END)
                self.status_var.set(f"Live operation failed: {error_message}")
                if error_context == "connect":
                    self._set_live_text(f"Connect failed\n\n{error_message}\n")
                    messagebox.showerror("Live Connect Failed", error_message)
                elif error_context == "read":
                    messagebox.showerror("Read Live Failed", error_message)
                elif error_context == "apply":
                    messagebox.showerror("Apply Live Failed", error_message)
            else:
                self.output_text.insert(tk.END, str(item))
                self.output_text.see(tk.END)
        self.after(100, self._poll_output_queue)


if __name__ == "__main__":
    app = FollowTuningTool()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        try:
            app.destroy()
        except tk.TclError:
            pass