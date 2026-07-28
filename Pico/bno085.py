# BNO085 Micropython I2C Library
# Optimized into a unified OOP structure

from math import asin, atan2, degrees
from collections import namedtuple
from micropython import const
from ustruct import unpack_from, pack_into
from utime import ticks_ms, sleep_ms, ticks_diff

# Packet header tuple structure
PacketHeader = namedtuple(
    "PacketHeader",
    [
        "channel_number",
        "sequence_number",
        "data_length",
        "packet_byte_count",
    ],
)

class BNO085:
    # BNO08X SETUP
    DEFAULT_ADDRESS = (0x4A, 0x4B)
    DATA_BUFFER_SIZE = 4096

    # Channel Numbers
    CHANNEL_SHTP_COMMAND = const(0x00)
    CHANNEL_EXE = const(0x01)
    CHANNEL_CONTROL = const(0x02)
    CHANNEL_INPUT_SENSOR_REPORTS = const(0x03)
    CHANNEL_WAKE_INPUT_SENSOR_REPORTS = const(0x04)
    CHANNEL_GYRO_ROTATION_VECTOR = const(0x05)

    # Configuring Reports
    COMMAND_RESPONSE = const(0xF1)
    COMMAND_REQUEST = const(0xF2)
    FRS_READ_RESPONSE = const(0xF3)
    FRS_READ_REQUEST = const(0xF4)
    FRS_WRITE_RESPONSE = const(0xF5)
    FRS_WRITE_DATA = const(0xF6)
    FRS_WRITE_REQUEST = const(0xF7)
    SHTP_REPORT_ID_RESPONSE = const(0xF8)
    SHTP_REPORT_ID_REQUEST = const(0xF9)
    REBASE_TIMESTAMP = const(0xFA)
    BASE_TIMESTAMP = const(0xFB)
    GET_FEATURE_RESPONSE = const(0xFC)
    SET_FEATURE_COMMAND = const(0xFD)
    GET_FEATURE_REQUEST = const(0xFE)

    # DCD/ ME Commands
    ME_ERRORREPORT_CDE = const(0x01)
    ME_COUNTER_CDE = const(0x02)
    ME_TARE_CDE = const(0x03)
    ME_INIT_CDE = const(0x04)
    ME_SAVE_DCD_CDE = const(0x06)
    ME_CALIBRATION_CDE = const(0x07)
    ME_SAVE_DCD_PERIODIC_CDE = const(0x09)
    ME_OSCILLATOR_TYPE_CDE = const(0x0A)
    ME_RESET_DCD_CDE = const(0x0B)

    # DCD/ME Sub-commands
    ME_COUNTER_GETCOUNTS_CDE = const(0x00)
    ME_COUNTER_CLEARCOUNTS_CDE = const(0x01)
    ME_TARE_NOW_SUBCDE = const(0x00)
    ME_TARE_PERSIST_SUBCDE = const(0x01)
    ME_TARE_REORIENTATION_SUBCDE = const(0x02)
    ME_CALIBRATION_CONFIG_SUBCDE = const(0x00)
    ME_CALIBRATION_GETCAL_SUBCDE = const(0x01)

    # Reports Summary
    REPORT_ACCELEROMETER = const(0x01)
    REPORT_GYROSCOPE = const(0x02)
    REPORT_MAGNETOMETER = const(0x03)
    REPORT_LINEAR_ACCELERATION = const(0x04)
    REPORT_ROTATION_VECTOR = const(0x05)
    REPORT_GRAVITY = const(0x06)
    REPORT_UNCALIBRATED_GYROSCOPE = const(0x07)
    REPORT_GAME_ROTATION_VECTOR = const(0x08)
    REPORT_GEOMAGNETIC_ROTATION_VECTOR = const(0x09)
    REPORT_PRESSURE = const(0x0A)
    REPORT_AMBIENT_LIGHT = const(0x0B)
    REPORT_HUMIDITY = const(0x0C)
    REPORT_PROXIMITY = const(0x0D)
    REPORT_TEMPERATURE = const(0x0E)
    REPORT_UNCALIBRATED_MAGNETOMETER = const(0x0F)
    REPORT_STEP_COUNTER = const(0x11)
    REPORT_STABILITY_CLASSIFIER = const(0x13)
    REPORT_RAW_ACCELEROMETER = const(0x14)
    REPORT_RAW_GYROSCOPE = const(0x15)
    REPORT_RAW_MAGNETOMETER = const(0x16)
    REPORT_SHAKE_DETECTOR = const(0x19)
    REPORT_ACTIVITY_CLASSIFIER = const(0x1E)

    # Timeouts
    QUAT_READ_TIMEOUT = 500
    PACKET_READ_TIMEOUT = 2000
    FEATURE_ENABLE_TIMEOUT = 2000
    DEFAULT_TIMEOUT = 2000

    # Scales & Constants
    Q_POINT_14_SCALAR = 2 ** -14
    Q_POINT_12_SCALAR = 2 ** -12
    Q_POINT_9_SCALAR = 2 ** -9
    Q_POINT_8_SCALAR = 2 ** -8
    Q_POINT_4_SCALAR = 2 ** -4

    # Available sensor configurations: (scalar, count, expected_report_length)
    AVAIL_SENSOR_REPORTS = {
        REPORT_ACCELEROMETER: (Q_POINT_8_SCALAR, 3, 10),
        REPORT_GYROSCOPE: (Q_POINT_9_SCALAR, 3, 10),
        REPORT_MAGNETOMETER: (Q_POINT_4_SCALAR, 3, 10),
        REPORT_LINEAR_ACCELERATION: (Q_POINT_8_SCALAR, 3, 10),
        REPORT_ROTATION_VECTOR: (Q_POINT_14_SCALAR, 4, 14),
        REPORT_GRAVITY: (Q_POINT_8_SCALAR, 3, 10),
        REPORT_GAME_ROTATION_VECTOR: (Q_POINT_14_SCALAR, 4, 12),
        REPORT_GEOMAGNETIC_ROTATION_VECTOR: (Q_POINT_12_SCALAR, 4, 14),
        REPORT_PRESSURE: (1, 1, 8),
        REPORT_AMBIENT_LIGHT: (1, 1, 8),
        REPORT_HUMIDITY: (1, 1, 6),
        REPORT_PROXIMITY: (1, 1, 6),
        REPORT_TEMPERATURE: (1, 1, 6),
        REPORT_STEP_COUNTER: (1, 1, 12),
        REPORT_SHAKE_DETECTOR: (1, 1, 6),
        REPORT_STABILITY_CLASSIFIER: (1, 1, 6),
        REPORT_ACTIVITY_CLASSIFIER: (1, 1, 16),
        REPORT_RAW_ACCELEROMETER: (1, 3, 16),
        REPORT_RAW_GYROSCOPE: (1, 3, 16),
        REPORT_RAW_MAGNETOMETER: (1, 3, 16),
        REPORT_UNCALIBRATED_GYROSCOPE: (Q_POINT_9_SCALAR, 3, 10),
        REPORT_UNCALIBRATED_MAGNETOMETER: (Q_POINT_4_SCALAR, 3, 10),
    }

    REPORT_LENGTHS = {
        SHTP_REPORT_ID_RESPONSE: 16,
        GET_FEATURE_RESPONSE: 17,
        COMMAND_RESPONSE: 16,
        BASE_TIMESTAMP: 5,
        REBASE_TIMESTAMP: 5,
    }

    RAW_REPORTS = {
        REPORT_RAW_ACCELEROMETER: REPORT_ACCELEROMETER,
        REPORT_RAW_GYROSCOPE: REPORT_GYROSCOPE,
        REPORT_RAW_MAGNETOMETER: REPORT_MAGNETOMETER,
    }

    ACTIVITIES = ["Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still", "Tilting", "Walking", "Running", "OnStairs"]
    ENABLED_ACTIVITIES = 0x1FF

    def __init__(self, i2c, address=None, rst_pin=None, int_pin=None, int_handler=None, debug=False):
        self._debug = debug
        self._i2c = i2c
        self._rst_pin = rst_pin
        self._ready = False

        if address is None:
            devices = set(self._i2c.scan())
            mpus = devices.intersection(set(self.DEFAULT_ADDRESS))
            if len(mpus) == 0:
                raise ValueError("No BNO08x detected")
            elif len(mpus) == 1:
                self._bno_add = mpus.pop()
                self._dbg("BNO08x found at address", hex(self._bno_add))
                self._ready = True
            else:
                raise ValueError("Two BNO08x detected: must specify a device address")
        else:
            self._bno_add = address

        if int_pin is not None:
            self.int_pin = int_pin
            self.int_handler = int_handler
            self.int_locked = False
            int_pin.irq(trigger=int_pin.IRQ_FALLING | int_pin.IRQ_RISING, handler=self.int_handle)

        self._dbg("INITIALISATION...")
        self._buffer = bytearray(self.DATA_BUFFER_SIZE)
        self._buffer_mv = memoryview(self._buffer)
        self._cde_buffer = bytearray(12)
        self._packet_slices = []

        self._seq_nb = [0, 0, 0, 0, 0, 0]
        self._sr_seq_nb = {"send": {}, "receive": {}}
        self._dcd_saved_at = -1
        self._me_calibration_started_at = -1
        self._calibration_complete = False
        self._magnetometer_accuracy = 0
        self._id_read = False
        self._quaternion_euler_vector = self.REPORT_GAME_ROTATION_VECTOR
        self._readings = {}
        
        self.initialize()

    def initialize(self):
        for _ in range(3):
            if self._rst_pin is not None:
                self.hard_reset()
            else:
                self.soft_reset()
            try:
                if self._check_id():
                    break
            except Exception:
                sleep_ms(500)
        else:
            raise RuntimeError("Could not initialize")

    def int_handle(self, pin):
        if not pin.value() and not self.int_locked:
            self.int_locked = True
        elif pin.value() and self.int_locked:
            self.int_locked = False

    def soft_reset(self):
        self._dbg("SOFT RESETTING...")
        data = bytearray([1])
        self._send_packet(self.CHANNEL_EXE, data)
        sleep_ms(500)
        self._send_packet(self.CHANNEL_EXE, data)
        sleep_ms(500)

        for _ in range(3):
            try:
                self._read_packet()
            except Exception:
                sleep_ms(500)
        self._dbg("SOFT RESETTING... OK!")

    def hard_reset(self):
        self._dbg("HARD RESETTING...")
        if self._rst_pin is None:
            return
        from machine import Pin
        self._reset = Pin(self._rst_pin, Pin.OUT)
        self._reset.value(1)
        sleep_ms(10)
        self._reset.value(0)
        sleep_ms(10)
        self._reset.value(1)
        sleep_ms(120)

    def enable_feature(self, feature_id, freq=20):
        self._dbg("ENABLING FEATURE ID...", feature_id)
        set_feature_report = bytearray(17)
        set_feature_report[0] = self.SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id
        
        report_interval = int(1_000_000 / freq)
        pack_into("<I", set_feature_report, 5, report_interval)
        if feature_id == self.REPORT_ACTIVITY_CLASSIFIER:
            pack_into("<I", set_feature_report, 13, self.ENABLED_ACTIVITIES)

        feature_dependency = self.RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._readings:
            self.enable_feature(feature_dependency, freq)

        self._send_packet(self.CHANNEL_CONTROL, set_feature_report)

        start_time = ticks_ms()
        while ticks_diff(ticks_ms(), start_time) < self.FEATURE_ENABLE_TIMEOUT:
            self._process_available_packets(max_packets=10)
            if feature_id in self._readings:
                return
        raise RuntimeError("Was not able to enable feature", feature_id)

    def set_quaternion_euler_vector(self, feature_id):
        self._quaternion_euler_vector = feature_id

    # ================= Class Properties =================

    @property
    def ready(self):
        return self._ready

    @property
    def acc(self):
        self._process_available_packets()
        if self.REPORT_ACCELEROMETER in self._readings:
            return self._readings[self.REPORT_ACCELEROMETER]
        raise RuntimeError("No accel report found, is it enabled?")

    @property
    def acc_raw(self):
        self._process_available_packets()
        if self.REPORT_RAW_ACCELEROMETER in self._readings:
            return self._readings[self.REPORT_RAW_ACCELEROMETER]
        raise RuntimeError("No raw acceleration report found, is it enabled?")

    @property
    def acc_linear(self):
        self._process_available_packets()
        if self.REPORT_LINEAR_ACCELERATION in self._readings:
            return self._readings[self.REPORT_LINEAR_ACCELERATION]
        raise RuntimeError("No lin. accel report found, is it enabled?")

    @property
    def gyro(self):
        self._process_available_packets()
        if self.REPORT_GYROSCOPE in self._readings:
            return self._readings[self.REPORT_GYROSCOPE]
        raise RuntimeError("No gyro report found, is it enabled?")

    @property
    def gyro_raw(self):
        self._process_available_packets()
        if self.REPORT_RAW_GYROSCOPE in self._readings:
            return self._readings[self.REPORT_RAW_GYROSCOPE]
        raise RuntimeError("No raw gyro report found, is it enabled?")

    @property
    def mag(self):
        self._process_available_packets()
        if self.REPORT_MAGNETOMETER in self._readings:
            return self._readings[self.REPORT_MAGNETOMETER]
        raise RuntimeError("No magfield report found, is it enabled?")

    @property
    def mag_raw(self):
        self._process_available_packets()
        if self.REPORT_RAW_MAGNETOMETER in self._readings:
            return self._readings[self.REPORT_RAW_MAGNETOMETER]
        raise RuntimeError("No raw magnetic report found, is it enabled?")

    @property
    def quaternion(self):
        self._process_available_packets()
        if self._quaternion_euler_vector in self._readings:
            return self._readings[self._quaternion_euler_vector]
        raise RuntimeError("No quaternion report found, is it enabled?")

    @property
    def euler(self):
        self._process_available_packets()
        if self._quaternion_euler_vector not in self._readings:
            raise RuntimeError("No quaternion report found, is it enabled?")
        
        q = self._readings[self._quaternion_euler_vector]
        jsqr = q[1] * q[1]
        t0 = +2.0 * (q[3] * q[0] + q[1] * q[2])
        t1 = +1.0 - 2.0 * (q[0] * q[0] + jsqr)
        roll = degrees(atan2(t0, t1))

        t2 = +2.0 * (q[3] * q[1] - q[2] * q[0])
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        tilt = degrees(asin(t2))

        t3 = +2.0 * (q[3] * q[2] + q[0] * q[1])
        t4 = +1.0 - 2.0 * (jsqr + q[2] * q[2])
        pan = degrees(atan2(t3, t4))
        return roll, tilt, pan

    @property
    def steps(self):
        self._process_available_packets()
        if self.REPORT_STEP_COUNTER in self._readings:
            return self._readings[self.REPORT_STEP_COUNTER]
        raise RuntimeError("No steps report found, is it enabled?")

    @property
    def gravity(self):
        self._process_available_packets()
        if self.REPORT_GRAVITY in self._readings:
            return self._readings[self.REPORT_GRAVITY]
        raise RuntimeError("No gravity report found, is it enabled?")

    @property
    def shake(self):
        self._process_available_packets()
        if self.REPORT_SHAKE_DETECTOR in self._readings:
            shake_detected = self._readings[self.REPORT_SHAKE_DETECTOR]
            if shake_detected:
                self._readings[self.REPORT_SHAKE_DETECTOR] = False
            return shake_detected
        raise RuntimeError("No shake report found, is it enabled?")

    # ================= Calibration / Tare =================

    def tare(self, axis=7, outputs=2):
        self._dbg("MOTION ENGINE TARE BEING DONE...")
        self._send_ME_cde(self.ME_TARE_CDE, [0, axis, outputs, 0, 0, 0, 0, 0, 0])
        self._calibration_complete = True

    def calibration(self):
        self._dbg("MOTION ENGINE CALIBRATION BEING DONE...")
        self._send_ME_cde(self.ME_CALIBRATION_CDE, [1, 1, 1, self.ME_CALIBRATION_CONFIG_SUBCDE, 0, 0, 0, 0, 0])
        self._calibration_complete = True

    @property
    def calibration_status(self):
        self._send_ME_cde(self.ME_CALIBRATION_CDE, [0, 0, 0, self.ME_CALIBRATION_GETCAL_SUBCDE, 0, 0, 0, 0, 0])
        return self._magnetometer_accuracy

    def calibration_save(self):
        local_buffer = bytearray(12)
        self._insert_cde_request_report(self.ME_SAVE_DCD_CDE, local_buffer, self._sr_seq_nb.get(self.COMMAND_REQUEST, 0))
        self._send_packet(self.CHANNEL_CONTROL, local_buffer)
        self._sr_seq_nb[self.COMMAND_REQUEST] = (self._sr_seq_nb.get(self.COMMAND_REQUEST, 0) + 1) % 256

        start_time = ticks_ms()
        while ticks_diff(ticks_ms(), start_time) < self.DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    # ================= Internal Packet Management =================

    def _header_from_buffer(self, header_bytes):
        header_byte_count, channel_number, sequence_number = unpack_from("<HBB", header_bytes)
        header_byte_count &= ~0x8000
        data_length = max(0, header_byte_count - 4)
        return PacketHeader(channel_number, sequence_number, data_length, header_byte_count)

    def _send_ME_cde(self, me_cde, subcommand):
        self._insert_cde_request_report(me_cde, self._cde_buffer, self._sr_seq_nb.get(self.COMMAND_REQUEST, 0), subcommand)
        self._send_packet(self.CHANNEL_CONTROL, self._cde_buffer)
        self._sr_seq_nb[self.COMMAND_REQUEST] = (self._sr_seq_nb.get(self.COMMAND_REQUEST, 0) + 1) % 256

    def _process_available_packets(self, max_packets=None):
        processed_count = 0
        while self._data_ready:
            if max_packets and processed_count > max_packets:
                return
            try:
                self._handle_packet(self._read_packet())
            except Exception:
                continue
            processed_count += 1

    def _wait_for_packet_type(self, channel_number, report_id=None, timeout=10000):
        start_time = ticks_ms()
        while ticks_diff(ticks_ms(), start_time) < timeout:
            if not self._data_ready:
                continue
            new_packet_data = self._read_packet_bytes()
            header = self._header_from_buffer(new_packet_data[0:4])
            if header.channel_number == channel_number:
                if report_id is None or (header.data_length > 0 and new_packet_data[4] == report_id):
                    return header
        raise RuntimeError("Timed out waiting for a packet on channel", channel_number)

    def _handle_packet(self, packet_bytes):
        header = self._header_from_buffer(packet_bytes[0:4])
        data = packet_bytes[4:]
        next_byte_index = 0
        while next_byte_index < header.data_length:
            report_id = data[next_byte_index]
            if report_id < 0xF0:
                required_bytes = self.AVAIL_SENSOR_REPORTS[report_id][2]
            else:
                required_bytes = self.REPORT_LENGTHS[report_id]
            
            if (header.data_length - next_byte_index) < required_bytes:
                break
            
            report_slice = data[next_byte_index: next_byte_index + required_bytes]
            self._process_report(report_slice[0], report_slice)
            next_byte_index += required_bytes

    def _process_report(self, report_id, report_bytes):
        if report_id >= 0xF0:
            if report_id == self.GET_FEATURE_RESPONSE:
                _rep_id, feature_report_id, *_ = unpack_from("<BBBHIII", report_bytes)
                self._readings[feature_report_id] = (0.0, 0.0, 0.0)
            elif report_id == self.COMMAND_RESPONSE:
                _, _, command, _, _ = unpack_from("<BBBBB", report_bytes)
                command_status = unpack_from("<B", report_bytes, 5)[0]
                if command == self.ME_CALIBRATION_CDE and command_status == 0:
                    self._me_calibration_started_at = ticks_ms()
                elif command == self.ME_SAVE_DCD_CDE:
                    if command_status == 0:
                        self._dcd_saved_at = ticks_ms()
                    else:
                        raise RuntimeError("Unable to save calibration data")
            return

        if report_id == self.REPORT_STEP_COUNTER:
            self._readings[report_id] = unpack_from("<H", report_bytes, 8)[0]
            return

        if report_id == self.REPORT_SHAKE_DETECTOR:
            shake_bitfield = unpack_from("<H", report_bytes, 4)[0]
            if (shake_bitfield & 0x07) != 0:
                self._readings[self.REPORT_SHAKE_DETECTOR] = True
            return

        if report_id in [self.REPORT_RAW_ACCELEROMETER, self.REPORT_RAW_GYROSCOPE, self.REPORT_RAW_MAGNETOMETER]:
            count = self.AVAIL_SENSOR_REPORTS[report_id][1]
            results = []
            for i in range(count):
                results.append(unpack_from("<H", report_bytes, 4 + (i * 2))[0])
            if report_id == self.REPORT_RAW_GYROSCOPE:
                temp_int = unpack_from("<h", report_bytes, 10)[0]
                results.append((temp_int / 2.0) + 23.0)
            results.append(unpack_from("<I", report_bytes, 12)[0])
            self._readings[report_id] = tuple(results)
            return

        # General scalar sensor reports
        scalar, count, _ = self.AVAIL_SENSOR_REPORTS[report_id]
        format_str = "<H" if report_id in self.RAW_REPORTS else "<h"
        results = []
        accuracy = unpack_from("<B", report_bytes, 2)[0] & 0b11

        for i in range(count):
            raw_data = unpack_from(format_str, report_bytes, 4 + (i * 2))[0]
            results.append(raw_data * scalar)
            
        self._readings[report_id] = tuple(results)
        if report_id == self.REPORT_MAGNETOMETER:
            self._magnetometer_accuracy = accuracy

    def _check_id(self):
        if self._id_read:
            return True
        data = bytearray([self.SHTP_REPORT_ID_REQUEST, 0])
        self._send_packet(self.CHANNEL_CONTROL, data)
        self._wait_for_packet_type(self.CHANNEL_CONTROL, self.SHTP_REPORT_ID_RESPONSE)
        if self._buffer[4] == self.SHTP_REPORT_ID_RESPONSE:
            sw_major, sw_minor, sw_part_number, _, _ = unpack_from("<BBIIH", self._buffer, 6)
            self._id_read = True
            return True
        return False

    @property
    def _data_ready(self):
        self._i2c.readfrom_into(self._bno_add, self._buffer_mv[0:4])
        header = self._header_from_buffer(self._buffer[0:4])
        if header.packet_byte_count == 0x7FFF:
            return False
        return header.data_length > 0

    def _send_packet(self, channel, data):
        write_length = len(data) + 4
        pack_into("<H", self._buffer, 0, write_length)
        self._buffer[2] = channel
        self._buffer[3] = self._seq_nb[channel]
        for idx, send_byte in enumerate(data):
            self._buffer[4 + idx] = send_byte
            
        self._i2c.writeto(self._bno_add, self._buffer[0:write_length])
        self._seq_nb[channel] = (self._seq_nb[channel] + 1) % 256
        return self._seq_nb[channel]

    def _read_packet_bytes(self):
        self._i2c.readfrom_into(self._bno_add, self._buffer_mv[0:4])
        header = self._header_from_buffer(self._buffer[0:4])
        if header.packet_byte_count > 0:
            self._i2c.readfrom_into(self._bno_add, self._buffer_mv[0:header.packet_byte_count])
        return self._buffer[0:header.packet_byte_count]

    def _read_packet(self):
        packet_bytes = self._read_packet_bytes()
        header = self._header_from_buffer(packet_bytes[0:4])
        self._seq_nb[header.channel_number] = header.sequence_number
        return packet_bytes

    def _insert_cde_request_report(self, command, buffer, next_sequence_number, command_params=None):
        for i in range(12):
            buffer[i] = 0
        buffer[0] = self.COMMAND_REQUEST
        buffer[1] = next_sequence_number
        buffer[2] = command
        if command_params:
            for idx, param in enumerate(command_params):
                buffer[3 + idx] = param

    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG:\tBNO085:\t", *args, **kwargs)