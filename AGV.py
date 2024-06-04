import threading
import time
from datetime import timedelta
from datetime import datetime

import RPi.GPIO as GPIO
from pynput import keyboard
import Navi
from steering_control import ENGINE

MOTOR_RIGHT = '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00KNUU-if00-port0'
MOTOR_LEFT =  '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00KX3V-if00-port0'

RUN_CMD_INTERVAL = 0.05

START_BUTTON_PIN = 17
STOP_BUTTON_PIN = 27

class AGV:
    def __init__(self):
        self.drive_mode = "auto"
        self.navi = Navi.NAVI(0)
        self.engine = ENGINE(MOTOR_LEFT, MOTOR_RIGHT)
        self.pause_sensor_is_on = False
        self.aruco_mark_command = None
        self.direction = ""
        self.is_driving = False
        self.curve_delay = False
        self.line_finded = False
        self.player_move_sound = Navi.MP3Player('/home/pi/Desktop/AGV/forklift_beep.mp3')
        self.gyro_recorder = Navi.GyroRecorder(file_path='/home/pi/AGV_log/gyro_data.txt')
        self.left_rpm = 300
        self.right_rpm = 300
        self.continuous_lidar_proximity_check()

    def start_gyro_recorder(self):
        self.gyro_recorder.start_gyro_record()

    def stop_gyro_recorder(self):
        self.gyro_recorder.stop_gyro_record()

    def play_move_sound(self):
        self.player_move_sound.play()

    def stop_move_sound(self):
        self.player_move_sound.stop()
        
    def continuous_lidar_proximity_check(self):
        lc = threading.Thread(target=self.lidar_proximity_verification)
        lc.start()
    
    def lidar_proximity_verification(self):
        while True:
            try:
                self.pause_sensor_is_on = self.navi.lidar.lidar_detected_proximity
            except:
                pass
            time.sleep(0.1)

    def turn_on(self):
        self.navi.start_navi()
        self.engine.enable()

    def write_last_move(self):
        if self.engine.motors_is_alive():
            with open("/home/pi/AGV_log/last_move.txt", 'w') as arquivo:
                arquivo.write(str(datetime.now()))

    def do_curves(self, left_rpm, right_rpm):
        while True:
            while self.pause_sensor_is_on:
                self.engine.disable()
            self.engine.enable()
            self.engine.set_rpm(300, 300)
            left, right = self.navi.get_curve_sensor_status()
            print("CURVE SENSORS", left, right)
            if left or right:
                break
        while self.pause_sensor_is_on:
            self.engine.set_rpm(0, 0)    
        self.engine.set_rpm(left_rpm * 0.2, right_rpm * 0.2)
        self.curve_delay = False
        print("Do Curve.....")

    def stop_drive(self):
        self.engine.set_rpm(0, 0)
        self.is_driving = False

    def drive(self):
        self.pause_sensor_is_on = self.navi.lidar.lidar_detected_proximity
        if self.is_driving:
            check = 0
            while self.pause_sensor_is_on:
                self.engine.disable()
                check = check+1
                self.pause_sensor_is_on = self.navi.lidar.lidar_detected_proximity
            if check !=0:
                self.engine.enable()
                self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)
            # Sensores em espera por isso vou para a parte de determinar rota

            if self.drive_mode == "auto":
                if self.aruco_mark_command is not None:
                    # Executa os comandos do arucoMark
                    pass

                else:
                    # Determinar Direcao
                    direction, motor_speed_left, motor_speed_right = self.navi.get_route()
                    if (direction == "Front Slow" or direction == "Front Normal") and not self.curve_delay:
                        if not self.line_finded:
                            print("ROUTE FINDED")
                            self.line_finded = True
                            self.motor_speed_left = 300
                            self.motor_speed_right = 300
                            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)

                        else:
                            self.line_finded = True
                            self.motor_speed_left = motor_speed_left
                            self.motor_speed_right = motor_speed_right
                            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)
                            
                    elif (direction == "Left Slow" or direction == "Left") and not self.curve_delay and self.line_finded:
                        # self.engine.set_rpm(motor_speed_left, motor_speed_right)
                        print("Curve identifed")
                        self.line_finded = False
                        self.curve_delay = True
                        self.motor_speed_left = motor_speed_left
                        self.motor_speed_right = motor_speed_right
                        tt = threading.Thread(target=self.do_curves, args=(self.motor_speed_left, self.motor_speed_right))
                        tt.start()

                    elif (direction == "Right Slow" or direction == "Right") and not self.curve_delay and self.line_finded:
                        # self.engine.set_rpm(motor_speed_left, motor_speed_right)
                        print("Curve identifed")
                        self.line_finded = False
                        self.curve_delay = True
                        self.motor_speed_left = motor_speed_left
                        self.motor_speed_right = motor_speed_right
                        tt = threading.Thread(target=self.do_curves, args=(self.motor_speed_left, self.motor_speed_right))
                        tt.start()



        tj= threading.Thread(target = self.drive)
        tj.start()

    def go_front(self):
        print(self.pause_sensor_is_on)
        if self.drive_mode == "manual" and self.is_driving and not self.pause_sensor_is_on:
            self.motor_speed_left = 300
            self.motor_speed_right = 300
            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)

    def go_back(self):
        if self.drive_mode == "manual" and self.is_driving and not self.pause_sensor_is_on:
            self.motor_speed_left = -300
            self.motor_speed_right = -300
            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)

    def turn_left(self):
        if self.drive_mode == "manual" and self.is_driving and not self.pause_sensor_is_on:
            self.motor_speed_left = -300
            self.motor_speed_right = 300
            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)

    def turn_right(self):
        if self.drive_mode == "manual" and self.is_driving and not self.pause_sensor_is_on:
            self.motor_speed_left = 300
            self.motor_speed_right = -300
            self.engine.set_rpm(self.motor_speed_left, self.motor_speed_right)

def on_press(key):
    try:
        print(key)
        if key.char == 'a':
            automatic_move_callback(None)
        elif key.char == 'm':
            manual_move_callback(None)
        elif key.char == '8':
            manual_move_front_callback(None)
        elif key.char == '2':
            manual_move_back_callback(None)
        elif key.char == '4':
            manual_move_left_callback(None)
        elif key.char == '6':
            manual_move_right_callback(None)
        elif key.char == 'r':
            start_button_callback(None)
        elif key.char == 's':
            stop_button_callback(None)

    except AttributeError:
        pass


def automatic_move_callback(channel):
    global agv
    agv.drive_mode = "auto"


def manual_move_callback(channel):
    global agv
    agv.drive_mode = "manual"
    print("manual")


def manual_move_front_callback(channel):
    global agv
    agv.go_front()
    print("Front")


def manual_move_back_callback(channel):
    global agv
    agv.go_back()


def manual_move_left_callback(channel):
    global agv
    agv.turn_left()


def manual_move_right_callback(channel):
    global agv
    agv.turn_right()


def start_button_callback(channel):
    global agv
    if not agv.is_driving:
        print("START")
        agv.is_driving = True
        time.sleep(1)
        agv.drive()
        agv.play_move_sound()
        agv.start_gyro_recorder()


def stop_button_callback(channel):
    global agv
    print("Stop")
    agv.stop_drive()
    agv.stop_move_sound()
    agv.stop_gyro_recorder()


if __name__ == '__main__':
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    agv = AGV()
    agv.turn_on()
    time.sleep(5)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(STOP_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(START_BUTTON_PIN, GPIO.FALLING, callback=start_button_callback, bouncetime=300)
    GPIO.add_event_detect(STOP_BUTTON_PIN, GPIO.FALLING, callback=stop_button_callback, bouncetime=300)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
