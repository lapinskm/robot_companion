"""microservice for robot control hosted on embedded rassberry pi zero processor"""
#rassbery Pi related libraries
import RPi.GPIO as GPIO
import flask
import picamzero

from time import sleep
import os
from uuid import uuid4
from flask import Flask, send_file

#FLASK constants
ROBOT_PORT = 8090
EXTERNAL_EXPOSE_HOST = "0.0.0.0"

#FILE paths constants
SRC_DIR = os.path.dirname(os.path.abspath(__file__))
FRONT_FILENAME = "front.html"
FRONT_PATH = os.path.join(SRC_DIR, FRONT_FILENAME)
CAM_IMAGE_PATH = os.path.join(SRC_DIR, "captured_frames")

# empirically derived PWM values
PW_FACTOR_CLOCKWISE = 42
PW_FACTOR_STOP = 0
PW_FACTOR_COUNTERCLOCKWISE = 15

MAX_DUTY = 100
MAX_ANGLE = 180

DUTY_CYCLE_DEGREE_RATIO = MAX_DUTY/MAX_ANGLE

class Led:
    """LED controller class"""
    def __init__(self, pin:int) -> None:
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        self.off()

    def on(self) -> None:
        """Turn LED on"""
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self) -> None:
        """Turn LED off"""
        GPIO.output(self.pin, GPIO.LOW)
  

class Servo:
    """Servo controller class"""
    SERVO_PWM_FREQ = 50
    def __init__(self, pwm_pin: int) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pwm_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(pwm_pin, self.SERVO_PWM_FREQ)
        self.servo_pwm.start(0)

    def scan_pwm(self) -> None:
        """Method allowing checking empirically servo behaviour depending on servo pwm duty cycle."""
        for factor in range(0, MAX_DUTY):
            self.servo_pwm.ChangeDutyCycle(factor)
            print(f"factor: {factor}")
            sleep(1.5)
              
class ContinuousServo(Servo):
    """Controller class for continuous work servos."""
    def __init__(self, pwm_pin: int, clockwise_forward: bool) -> None:
        super().__init__(pwm_pin)
        self.clockwise_forward = clockwise_forward

    def forward(self) -> None:
        """Drives servo to run full speed forward."""
        if self.clockwise_forward:
            self.servo_pwm.ChangeDutyCycle(PW_FACTOR_CLOCKWISE)
        else:
            self.servo_pwm.ChangeDutyCycle(PW_FACTOR_COUNTERCLOCKWISE)

    def backward(self) -> None:
        """Drives servo to run full speed backward."""
        if self.clockwise_forward:
            self.servo_pwm.ChangeDutyCycle(PW_FACTOR_COUNTERCLOCKWISE)
        else:
            self.servo_pwm.ChangeDutyCycle(PW_FACTOR_CLOCKWISE)

    def stop(self) -> None:
        """Stops servo."""
        self.servo_pwm.ChangeDutyCycle(PW_FACTOR_STOP)

class PositionServo(Servo):
    """Controller class for positional servos."""
    def __init__(self, pwm_pin: int, initial_positon: float) -> None:
        super().__init__(pwm_pin)
        self.set_posion(initial_positon)


    def set_posion(self, degree: float) -> None:
        """Sets servo to given position."""
        self.servo_pwm.ChangeDutyCycle(degree * DUTY_CYCLE_DEGREE_RATIO)
        self.position = degree


class Camera:
    """Pi Zerdo camera controller."""
    def __init__(self, image_dir: str) -> None:
        self.camera = picamzero.Camera()
        self.image_dir = image_dir

    def clear_image_dir(self) -> None:
        """Utility function to clear image directory."""
        for filename in os.listdir(self.image_dir):
            img_path = os.path.join(self.image_dir, filename)
            if os.path.isfile(img_path):
                os.remove(img_path)

    def capture_frame(self) -> str:
        """Triggers frame capture,and returns unique image filename."""
        self.clear_image_dir()
        filename = f"{str(uuid4())[:8]}.jpg"
        img_path = os.path.join(self.image_dir, filename)
        self.camera.take_photo(img_path)
        return filename

class TrackedRobot:
    """Robot controller class for tracked robot"""
    LIGHT_LED_PIN = 17
    READY_LED_PIN = 4
    LEFT_SERVO_PWM_PIN = 12
    RIGHT_SERVO_PWM_PIN = 18
    LEFT_CLOCKWISE_FORWARD = False
    RIGHT_CLOCKWISE_FORWARD = True
    CAM_YAW_SERVO_PWM_PIN = 13
    CAM_PITCH_SERVO_PWM_PIN = 19
    CAM_YAW_INITIAL_VALUE = 90
    CAM_PITCH_INITIAL_VALUE = 90
    
    def __init__(self) -> None:
        self.left_servo = ContinuousServo(self.LEFT_SERVO_PWM_PIN,
                                                     self.LEFT_CLOCKWISE_FORWARD)
        self.right_servo = ContinuousServo(self.RIGHT_SERVO_PWM_PIN,
                                                      self.RIGHT_CLOCKWISE_FORWARD)

        # Currently unused, will be used for camera positioning
        self.yaw_servo = PositionServo(self.CAM_YAW_SERVO_PWM_PIN,
                                                 self.CAM_YAW_INITIAL_VALUE)
        self.pitch_servo = PositionServo(self.CAM_PITCH_SERVO_PWM_PIN,
                                                    self.CAM_PITCH_INITIAL_VALUE)

        self.camera = Camera(CAM_IMAGE_PATH)
        self.camera_light = Led(self.LIGHT_LED_PIN)
        self.ready_indicator = Led(self.READY_LED_PIN)
        self.use_cam_light = True

    def scan_servos(self) -> None:
        """Start scanning servos for empirical assessment of duty cycles."""
        print("left")
        self.left_servo.scan_pwm()
        print("right")
        self.right_servo.scan_pwm()

    def indicate_ready(self) -> None:
        """Turn on readines indicator LED"""
        self.ready_indicator.on()

    def relax(self) -> None:
        """Turn off readines indicator LED"""
        self.stop()
        self.ready_indicator.off()

    def forward(self) -> None:
        """Drive track servos to run full speed forward."""
        print("forward")
        self.left_servo.forward()
        self.right_servo.forward()

    def backward(self) -> None:
        """Drive track servos to run full speed backward."""
        print("backward")
        self.left_servo.backward()
        self.right_servo.backward()

    def turn_left(self) -> None:
        """Drive track servos to start turning left."""
        print("left")
        self.left_servo.backward()
        self.right_servo.forward()

    def turn_right(self) -> None:
        """Drive track servos to start turning right."""
        print("right")
        self.left_servo.forward()
        self.right_servo.backward()


    def toggle_use_light(self) -> bool:
        """Change usage of camera light LEDs."""
        self.use_cam_light = not self.use_cam_light
        return self.use_cam_light

    def stop(self) -> None:
        """Stop all movements."""
        #print("stop")
        self.left_servo.stop()
        self.right_servo.stop()

    def capture_frame(self) -> str:
        """Triggers frame capture,and returns unique image filename."""
        use_light=self.use_cam_light
        if use_light:
              self.camera_light.on()
        filename = self.camera.capture_frame()
        if use_light:
              self.camera_light.off()
        print(f"frame saved as: {filename}")
        return filename
    
 
# initialize controller class
robot = TrackedRobot()

app = Flask(__name__)


@app.route("/")
def gui() -> str:
    """Return human-friendly HTML frontend for robot remote control."""
    with open(FRONT_PATH, encoding="utf8") as gui_file:
        return gui_file.read()

@app.route("/forward")
def forward():
    """Trigger step forward."""
    robot.forward()
    sleep(0.33)
    robot.stop()
    return ""

@app.route("/backward")
def backward():
    """Trigger step backward."""
    robot.backward()
    sleep(0.33)
    robot.stop()
    return ""

@app.route("/turn_left")
def turn_left():
    """Trigger step left."""
    robot.turn_left()
    sleep(0.33)
    robot.stop()
    return ""

@app.route("/turn_right")
def turn_right():
    """Trigger step right."""
    robot.turn_right()
    sleep(0.33)
    robot.stop()
    return ""

@app.route("/capture_frame")
def capture_frame() -> str:
    """Triggers frame capture and returns relative image download path."""
    return f"download/{robot.capture_frame()}"

@app.route("/download/<path:filename>")
def download(filename: str) -> flask.Response:
    """Process download of image path"""
    return send_file(os.path.join(CAM_IMAGE_PATH, filename))

@app.route("/terminate")
def terminate() -> None:
    """Terminate robot controller service."""
    robot.relax()
    exit(0)

@app.route("/get_use_light")
def get_use_light() -> str:
    """Return usage of camera light LEDs."""
    return str(robot.use_cam_light)

@app.route("/toggle_use_light")
def toggle_use_light() -> str:
    """Change usage of camera light LEDs."""
    return str(robot.toggle_use_light())

@app.route("/scan")
def scan() -> None:
    """Trigger servo scan of duty cycle."""
    robot.scan_servos()
    robot.relax()

robot.indicate_ready()

app.run(host=EXTERNAL_EXPOSE_HOST,port=ROBOT_PORT)
