import RPi.GPIO as GPIO
import time
import board
import busio
import adafruit_tcs34725

# Initialize I2C and TCS34725 sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin Definitions
LED1_PIN = 17  # Orange pingpong LED
LED2_PIN = 27  # White pingpong LED
LED3_PIN = 22  # No pingpong LED
SERVO1_PIN = 12  # Gate1 servo
SERVO2_PIN = 13  # Gate2 servo

# Setup GPIO pins
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)
GPIO.setup(LED3_PIN, GPIO.OUT)
GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

# Setup PWM for servos
servo1 = GPIO.PWM(SERVO1_PIN, 50)  # 50Hz frequency
servo2 = GPIO.PWM(SERVO2_PIN, 50)
servo1.start(0)
servo2.start(0)

class PingpongSorter:
    def __init__(self):
        self.state = "Idle"
        self.counter = 0
        self.start_time = time.time()
    
    def angle_to_duty_cycle(self, angle):
        """
        Convert angle to duty cycle
        The formula is adjusted to account for the servo's orientation
        """
        return (angle / 18.0) + 7.0  # Adjusted formula for more accurate servo control
    
    def set_servo_angle(self, servo, angle):
        """Set servo angle"""
        duty = self.angle_to_duty_cycle(angle)
        servo.ChangeDutyCycle(duty)
        time.sleep(0.3)  # Allow time for servo to move
        
    def read_colors(self):
        """Read RGB values from sensor"""
        color = sensor.color_rgb_bytes
        r, g, b = color[0], color[1], color[2]
        print(f"Color values - R: {r}, G: {g}, B: {b}")  # Debug output
        return r, g, b
    
    def is_orange_pingpong(self, r, g, b):
        """Check if color values match orange pingpong"""
        return (r > 30 and r < 255)
    
    def is_white_pingpong(self, r, g, b):
        """Check if color values match white pingpong"""
        return (r < 30 and r > 10)

    def is_no_pingpong(self, r, g, b):
        """Check if no pingpong is detected"""
        orange_condition = (r > 30 and r < 255)
        white_condition = (r < 30 and r > 10)
        return not (orange_condition or white_condition)
    
    def update_counter(self):
        """Update time counter"""
        self.counter = int(time.time() - self.start_time)
    
    def reset_counter(self):
        """Reset time counter"""
        self.start_time = time.time()
        self.counter = 0
    
    def run(self):
        try:
            while True:
                self.update_counter()
                r, g, b = self.read_colors()

                if self.state == "Idle":
                    if self.is_no_pingpong(r, g, b):
                        GPIO.output(LED3_PIN, 1)  # LED3 ON (No pingpong)
                        self.set_servo_angle(servo1, -90)  # Close Gate1
                    elif self.is_orange_pingpong(r, g, b) or self.is_white_pingpong(r, g, b):
                        self.state = "Pingpong_found"

                elif self.state == "Pingpong_found":
                    if self.is_no_pingpong(r, g, b):
                        self.state = "Idle"
                    else:
                        if self.is_orange_pingpong(r, g, b):
                            GPIO.output(LED1_PIN, 1)  # LED1 ON (Orange)
                            self.set_servo_angle(servo2, -45)  # Counterclockwise for orange
                            GPIO.output(LED3_PIN, 0)  # LED3 OFF
                            self.state = "Pingpong_is_Orange"
                            self.reset_counter()
                        elif self.is_white_pingpong(r, g, b):
                            GPIO.output(LED2_PIN, 1)  # LED2 ON (White)
                            self.set_servo_angle(servo2, 45)  # Clockwise for white
                            GPIO.output(LED3_PIN, 0)  # LED3 OFF
                            self.state = "Pingpong_is_White"
                            self.reset_counter()

                elif self.state == "Pingpong_is_Orange" or self.state == "Pingpong_is_White":
                    if self.counter >= 3 and self.is_no_pingpong(r, g, b):
                        self.set_servo_angle(servo2, 0)  # Reset Gate2 to center
                        GPIO.output(LED1_PIN, 0)  # LED1 OFF
                        GPIO.output(LED2_PIN, 0)  # LED2 OFF
                        self.state = "Close_Gate2"
                        self.reset_counter()

                elif self.state == "Close_Gate2" and self.counter >= 1:
                    self.set_servo_angle(servo1, 90)  # Open Gate1            
                    self.state = "Open_Gate1"
                    self.reset_counter()

                elif self.state == "Open_Gate1" and self.counter >= 1:
                    self.set_servo_angle(servo1, -90)  # Close Gate1
                    self.state = "Idle"
                    self.reset_counter()

                time.sleep(0.1)  # Small delay to prevent CPU overload

        except KeyboardInterrupt:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup GPIO and PWM on exit"""
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    sorter = PingpongSorter()
    sorter.run()
