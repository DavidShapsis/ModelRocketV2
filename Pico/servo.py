from machine import Pin, PWM

class Servo:
    def __init__(self, pin_num, freq=50):
        """Initializes a standard 50Hz hobby servo on the specified GPIO pin."""
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(freq)
        
        # Standard calibration for 0 to 180 degrees using u16 duty cycles
        # 0 degrees   -> ~0.5ms pulse -> ~1638 duty
        # 180 degrees -> ~2.5ms pulse -> ~8192 duty
        self.min_duty = 1638
        self.max_duty = 8192

    def write_angle(self, degrees):
        """Moves the servo to a specific angle between 0 and 180 degrees."""
        # Constrain input to valid physical limits
        if degrees < 0: degrees = 0
        if degrees > 180: degrees = 180
            
        # Map degrees (0-180) to duty cycle range (min_duty to max_duty)
        duty = int(self.min_duty + (degrees / 180.0) * (self.max_duty - self.min_duty))
        self.pwm.duty_u16(duty)

    def deinit(self):
        """Turns off the PWM signal to stop the servo from drawing standby power."""
        self.pwm.deinit()