from micropython import const
from ustruct import unpack as unp
import time

# Author David Stenwall (david at stenwall.io)
# Modified for Rocket Avionics Baseline, Precision Logging, and Apogee Detection

# Power Modes
BMP280_POWER_SLEEP = const(0)
BMP280_POWER_FORCED = const(1)
BMP280_POWER_NORMAL = const(3)

BMP280_SPI3W_ON = const(1)
BMP280_SPI3W_OFF = const(0)

BMP280_TEMP_OS_SKIP = const(0)
BMP280_TEMP_OS_1 = const(1)
BMP280_TEMP_OS_2 = const(2)
BMP280_TEMP_OS_4 = const(3)
BMP280_TEMP_OS_8 = const(4)
BMP280_TEMP_OS_16 = const(5)

BMP280_PRES_OS_SKIP = const(0)
BMP280_PRES_OS_1 = const(1)
BMP280_PRES_OS_2 = const(2)
BMP280_PRES_OS_4 = const(3)
BMP280_PRES_OS_8 = const(4)
BMP280_PRES_OS_16 = const(5)

# Standby settings in ms
BMP280_STANDBY_0_5 = const(0)
BMP280_STANDBY_62_5 = const(1)
BMP280_STANDBY_125 = const(2)
BMP280_STANDBY_250 = const(3)
BMP280_STANDBY_500 = const(4)
BMP280_STANDBY_1000 = const(5)
BMP280_STANDBY_2000 = const(6)
BMP280_STANDBY_4000 = const(7)

# IIR Filter setting
BMP280_IIR_FILTER_OFF = const(0)
BMP280_IIR_FILTER_2 = const(1)
BMP280_IIR_FILTER_4 = const(2)
BMP280_IIR_FILTER_8 = const(3)
BMP280_IIR_FILTER_16 = const(4)

# Oversampling setting
BMP280_OS_ULTRALOW = const(0)
BMP280_OS_LOW = const(1)
BMP280_OS_STANDARD = const(2)
BMP280_OS_HIGH = const(3)
BMP280_OS_ULTRAHIGH = const(4)

# Oversampling matrix
# (PRESS_OS, TEMP_OS, sample time in ms)
_BMP280_OS_MATRIX = [
    [BMP280_PRES_OS_1, BMP280_TEMP_OS_1, 7],
    [BMP280_PRES_OS_2, BMP280_TEMP_OS_1, 9],
    [BMP280_PRES_OS_4, BMP280_TEMP_OS_1, 14],
    [BMP280_PRES_OS_8, BMP280_TEMP_OS_1, 23],
    [BMP280_PRES_OS_16, BMP280_TEMP_OS_2, 44]
]

# Use cases
BMP280_CASE_HANDHELD_LOW = const(0)
BMP280_CASE_HANDHELD_DYN = const(1)
BMP280_CASE_WEATHER = const(2)
BMP280_CASE_FLOOR = const(3)
BMP280_CASE_DROP = const(4)
BMP280_CASE_INDOOR = const(5)

_BMP280_CASE_MATRIX = [
    [BMP280_POWER_NORMAL, BMP280_OS_ULTRAHIGH, BMP280_IIR_FILTER_4, BMP280_STANDBY_62_5],
    [BMP280_POWER_NORMAL, BMP280_OS_STANDARD, BMP280_IIR_FILTER_16, BMP280_STANDBY_0_5],
    [BMP280_POWER_FORCED, BMP280_OS_ULTRALOW, BMP280_IIR_FILTER_OFF, BMP280_STANDBY_0_5],
    [BMP280_POWER_NORMAL, BMP280_OS_STANDARD, BMP280_IIR_FILTER_4, BMP280_STANDBY_125],
    [BMP280_POWER_NORMAL, BMP280_OS_LOW, BMP280_IIR_FILTER_OFF, BMP280_STANDBY_0_5],
    [BMP280_POWER_NORMAL, BMP280_OS_ULTRAHIGH, BMP280_IIR_FILTER_16, BMP280_STANDBY_0_5]
]

_BMP280_REGISTER_ID = const(0xD0)
_BMP280_REGISTER_RESET = const(0xE0)
_BMP280_REGISTER_STATUS = const(0xF3)
_BMP280_REGISTER_CONTROL = const(0xF4)
_BMP280_REGISTER_CONFIG = const(0xF5)  # IIR filter config

_BMP280_REGISTER_DATA = const(0xF7)


class BMP280:
    def __init__(self, i2c_bus, addr=0x77, use_case=BMP280_CASE_HANDHELD_DYN, log_filename="flight_log.txt"):
        self._bmp_i2c = i2c_bus
        self._i2c_addr = addr
        self.log_filename = log_filename

        # Read calibration data
        self._T1 = unp('<H', self._read(0x88, 2))[0]
        self._T2 = unp('<h', self._read(0x8A, 2))[0]
        self._T3 = unp('<h', self._read(0x8C, 2))[0]
        self._P1 = unp('<H', self._read(0x8E, 2))[0]
        self._P2 = unp('<h', self._read(0x90, 2))[0]
        self._P3 = unp('<h', self._read(0x92, 2))[0]
        self._P4 = unp('<h', self._read(0x94, 2))[0]
        self._P5 = unp('<h', self._read(0x96, 2))[0]
        self._P6 = unp('<h', self._read(0x98, 2))[0]
        self._P7 = unp('<h', self._read(0x9A, 2))[0]
        self._P8 = unp('<h', self._read(0x9C, 2))[0]
        self._P9 = unp('<h', self._read(0x9E, 2))[0]

        self._t_raw = 0
        self._t_fine = 0
        self._t = 0
        self._p_raw = 0
        self._p = 0

        self.read_wait_ms = 0
        self._new_read_ms = 200
        self._last_read_ts = 0

        if use_case is not None:
            self.use_case(use_case)

        # Avionics Tracking Variables
        self.ground_altitude_ft = 0.0
        self.max_altitude_ft = 0.0
        self.apogee_detected = False
        
        # Apogee filtering (requires 5 consecutive declining readings)
        self._decline_count = 0
        self._apogee_threshold_readings = 5

        # Initialize the log file with updated headers
        with open(self.log_filename, "w") as f:
            f.write("time_seconds,pressure_hpa,temperature_c,relative_altitude_ft\n")

        # Automatically establish ground baseline calibration
        self.calibrate_ground()

    def _read(self, addr, size=1):
        return self._bmp_i2c.readfrom_mem(self._i2c_addr, addr, size)

    def _write(self, addr, b_arr):
        if not type(b_arr) is bytearray:
            b_arr = bytearray([b_arr])
        return self._bmp_i2c.writeto_mem(self._i2c_addr, addr, b_arr)

    def _gauge(self):
        d = self._read(_BMP280_REGISTER_DATA, 6)
        self._p_raw = (d[0] << 12) + (d[1] << 4) + (d[2] >> 4)
        self._t_raw = (d[3] << 12) + (d[4] << 4) + (d[5] >> 4)
        self._t_fine = 0
        self._t = 0
        self._p = 0

    def _calc_t_fine(self):
        self._gauge()
        if self._t_fine == 0:
            var1 = (((self._t_raw >> 3) - (self._T1 << 1)) * self._T2) >> 11
            var2 = (((((self._t_raw >> 4) - self._T1) * ((self._t_raw >> 4) - self._T1)) >> 12) * self._T3) >> 14
            self._t_fine = var1 + var2

    @property
    def temperature(self):
        self._calc_t_fine()
        if self._t == 0:
            self._t = ((self._t_fine * 5 + 128) >> 8) / 100.
        return self._t

    @property
    def pressure(self):
        self._calc_t_fine()
        if self._p == 0:
            var1 = self._t_fine - 128000
            var2 = var1 * var1 * self._P6
            var2 = var2 + ((var1 * self._P5) << 17)
            var2 = var2 + (self._P4 << 35)
            var1 = ((var1 * var1 * self._P3) >> 8) + ((var1 * self._P2) << 12)
            var1 = (((1 << 47) + var1) * self._P1) >> 33

            if var1 == 0:
                return 0

            p = 1048576 - self._p_raw
            p = int((((p << 31) - var2) * 3125) / var1)
            var1 = (self._P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self._P8 * p) >> 19

            p = ((p + var1 + var2) >> 8) + (self._P7 << 4)
            self._p = p / 256.0
        return self._p

    def _get_absolute_altitude_ft(self):
        """Calculates absolute altitude above sea level in feet."""
        p_hpa = self.pressure / 100.0
        if p_hpa == 0:
            return 0.0
        meters = 44330 * (1 - (p_hpa / 1013.25) ** 0.1903)
        return meters * 3.28084

    def calibrate_ground(self, samples=20):
        """Discards initial power-up errors, reads baseline data, and zeroes altitude."""
        print("Calibrating launchpad baseline... Do not move sensor.")
        for _ in range(3):
            self._get_absolute_altitude_ft()
            time.sleep(0.05)
            
        total = 0.0
        for _ in range(samples):
            total += self._get_absolute_altitude_ft()
            time.sleep(0.05)
            
        self.ground_altitude_ft = total / samples
        print("Launchpad Baseline Fixed at: {:.2f} ft ASL".format(self.ground_altitude_ft))

    @property
    def altitude(self):
        """Returns relative flight altitude in feet (Launchpad is 0.0)."""
        return self._get_absolute_altitude_ft() - self.ground_altitude_ft

    def log_data(self):
        """Appends active flight metrics into the text file with time in seconds."""
        # Convert milliseconds system timer to fractional seconds
        current_time_seconds = time.ticks_ms() / 1000.0
        p_hpa = self.pressure / 100.0
        temp = self.temperature
        rel_alt = self.altitude
        
        try:
            with open(self.log_filename, "a") as f:
                # {:.3f} enforces exactly 3 decimal places for your timestamp string
                f.write("{:.3f},{:.2f},{:.2f},{:.2f}\n".format(current_time_seconds, p_hpa, temp, rel_alt))
        except Exception as e:
            print("Logging error:", e)

    def check_apogee(self):
        """Monitors peak altitude. Returns True when a structural descent matches apogee."""
        if self.apogee_detected:
            return True

        current_alt = self.altitude

        if current_alt > self.max_altitude_ft:
            self.max_altitude_ft = current_alt
            self._decline_count = 0  
        else:
            if (self.max_altitude_ft - current_alt) > 1.5:
                self._decline_count += 1

        if self._decline_count >= self._apogee_threshold_readings:
            self.apogee_detected = True
            print("!!! APOGEE DETECTED AT {:.2f} FEET !!!".format(self.max_altitude_ft))
            return True

        return False

    def _write_bits(self, address, value, length, shift=0):
        d = self._read(address)[0]
        m = int('1' * length, 2) << shift
        d &= ~m
        d |= m & value << shift
        self._write(address, d)

    def _read_bits(self, address, length, shift=0):
        d = self._read(address)[0]
        return d >> shift & int('1' * length, 2)

    def use_case(self, uc):
        assert 0 <= uc <= 5
        pm, oss, iir, sb = _BMP280_CASE_MATRIX[uc]
        p_os, t_os, self.read_wait_ms = _BMP280_OS_MATRIX[oss]
        self._write(_BMP280_REGISTER_CONFIG, (iir << 2) + (sb << 5))
        self._write(_BMP280_REGISTER_CONTROL, pm + (p_os << 2) + (t_os << 5))