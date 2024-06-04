#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mai 13 2024
@author: Ichikawa Eisei
"""
import time
from datetime import datetime, timedelta

import cv2
import numpy
import threading
import os

import pygame

import image_processing
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor, LED
import smbus2

import struct
import numpy as np
import serial
from serial.tools import list_ports
import threading
import time


import cv2
import struct
import numpy as np
import serial
from serial.tools import list_ports
import threading
import time

class LIDAR:
    def __init__(self, usbID="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"):
        self.all_frames = []  # Shared frame list
        self.is_running = True  # Variable indicating the running state of the threads
        self.usbID = usbID
        self.lidar_img = None
        self.lidar_ret = False
        self.data_thread = None
        self.visualization_thread = None
        self.lidar_detected_proximity  = False
        self.max_proximity = 1000
        self.create_lidar_background()

    def run(self):
        
        self.data_thread = threading.Thread(target=self.read_data_from_serial)
        self.data_thread.start()

        # Start the thread for visualizing data
        self.visualization_thread = threading.Thread(target=self.visualize_lidar_data_continuously)
        self.visualization_thread.start()

    def stop(self):
        self.is_running = False
        self.data_thread.join()
        self.visualization_thread.join()

    def get_lidar_frames_from_buffer(self, buffer):
        frames = []
        while len(buffer) >= 47:
            # If the header is not correct, search for the next header
            if buffer[0] != 0x54:
                if 0x54 in buffer:
                    buffer = buffer[buffer.index(0x54):]
                else:
                    break

            # Extract one frame
            frame_data = buffer[:47]
            buffer = buffer[47:]

            # Verify the frame data is correct
            if not self.check_lidar_frame_data(frame_data):
                continue

            frames.append(self.get_lidar_frame(frame_data))

        return frames, buffer

    def check_lidar_frame_data(self, data):
        return data[1] == 0x2C and len(data) == 47

    def calc_crc8(self, data):
        # Implement the CRC8 calculation logic (dummy function as actual logic is unknown)
        return sum(data) % 256

    def get_lidar_frame(self, data):
        frame = {}
        frame['header'] = data[0]
        frame['ver_len'] = data[1]
        frame['speed'] = struct.unpack("<H", bytes(data[2:4]))[0]
        frame['startAngle'] = struct.unpack("<H", bytes(data[4:6]))[0]

        points = []
        for i in range(12):
            start_index = 6 + 3 * i
            distance = struct.unpack("<H", bytes(data[start_index:start_index + 2]))[0]
            intensity = data[start_index + 2]
            points.append((distance, intensity))

        frame['points'] = points
        frame['endAngle'] = struct.unpack("<H", bytes(data[42:44]))[0]
        frame['timestamp'] = struct.unpack("<H", bytes(data[44:46]))[0]
        frame['crc8'] = data[46]

        return frame

    def create_lidar_background(self):
        img = np.zeros((500, 500, 3), dtype=np.uint8)
        self.center = (img.shape[0] // 2, img.shape[1] // 2)
        cv2.circle(img, self.center, 5, (255, 255, 255), -1)
        cv2.circle(img, self.center, 40, (255, 255, 255), 1)
        cv2.circle(img, self.center, 90, (255, 255, 255), 1)
        cv2.circle(img, self.center, 140, (255, 255, 255), 1)
        cv2.circle(img, self.center, 190, (255, 255, 255), 1)
        cv2.circle(img, self.center, 240, (255, 255, 255), 1)
        self.lidar_background = img.copy()

    def visualize_lidar_frames(self, frames):
        img = self.lidar_background.copy()
        center = self.center
        self.lidar_detected_proximity = False
        for frame in frames:
            self.lidar_img = img
            diffAngle = (frame['endAngle'] - frame['startAngle']) / 11.0 if frame['endAngle'] > frame[
                'startAngle'] else (frame['endAngle'] + 36000.0 - frame['startAngle']) / 11.0
            for i, (distance, intensity) in enumerate(frame['points']):
                angle = (frame['startAngle'] + i * diffAngle) * (np.pi / 18000.0)
                angle = angle % (2 * np.pi)
                x = center[0] + distance * 0.1 * np.cos(angle)
                y = center[1] + distance * 0.1 * np.sin(angle)
                #cv2.circle(img, (int(x), int(y)), 2, (0, intensity, 255 - intensity), -1)
                if distance >= 10 and distance <= self.max_proximity and not ((x>200 and x<300) and(y<250)):
                    cv2.circle(img, (int(x), int(y)), 5, (0,0,255), -1)
                    #cv2.putText(img,'X',(int(x), int(y)),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),1,cv2.LINE_AA)
                    self.lidar_detected_proximity = True
        self.lidar_img = img
        #if self.lidar_img is None:
        #cv2.imshow("Lidar Data", img)

    def read_data_from_serial(self):
        # List available serial ports
        available_ports = list(serial.tools.list_ports.comports())

        if not available_ports:
            print("No available serial ports found. Exiting the program.")
        else:
            # Select the first serial port
            selected_port = self.usbID
            print(f"Selected serial port: {selected_port}")

            # Connect to the serial port
            try:
                ser = serial.Serial(selected_port, 230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE)
                time.sleep(1)
                buffer = bytearray()
                while self.is_running:
                    while ser.in_waiting:
                        buffer.extend(ser.read(ser.in_waiting))
                    frames, buffer = self.get_lidar_frames_from_buffer(buffer)
                    if frames:
                        self.all_frames.extend(frames)
                        time.sleep(0.01)
            except serial.SerialException as e:
                print(f"Error while connecting to the serial port: {e}")
            finally:
                if ser.is_open:
                    ser.close()
                    print(f"Closed serial port {selected_port}")

    def visualize_lidar_data_continuously(self):
        
        while self.is_running:
            if self.all_frames:
                
                self.visualize_lidar_frames(self.all_frames)
                self.all_frames.clear()
            #time.sleep(0.1)
            #key = cv2.waitKey(1)

            #if key == ord('q') or key == 27:  # If 'q' key or ESC key is pressed
                #print("end!")
                #self.is_running = False


class GyroRecorder:
    def __init__(self, bus_number=1, device_address=0x68, file_path='gyro_data.txt'):
        self.bus = smbus2.SMBus(bus_number)
        self.device_address = device_address
        self.file_path = file_path
        self.recording = False
        self.data = []
        self.lock = threading.Lock()

        # Inicializa o MPU-6050
        self.bus.write_byte_data(self.device_address, 0x6B, 0)

    def read_gyro_data(self):
        # Leitura dos registros de giroscópio
        high_x = self.bus.read_byte_data(self.device_address, 0x43)
        low_x = self.bus.read_byte_data(self.device_address, 0x44)
        high_y = self.bus.read_byte_data(self.device_address, 0x45)
        low_y = self.bus.read_byte_data(self.device_address, 0x46)
        high_z = self.bus.read_byte_data(self.device_address, 0x47)
        low_z = self.bus.read_byte_data(self.device_address, 0x48)

        gyro_x = (high_x << 8) | low_x
        gyro_y = (high_y << 8) | low_y
        gyro_z = (high_z << 8) | low_z

        if gyro_x >= 0x8000:
            gyro_x = -((65535 - gyro_x) + 1)
        if gyro_y >= 0x8000:
            gyro_y = -((65535 - gyro_y) + 1)
        if gyro_z >= 0x8000:
            gyro_z = -((65535 - gyro_z) + 1)

        return gyro_x, gyro_y, gyro_z

    def record_gyro_data(self):
        while self.recording:
            gyro_x, gyro_y, gyro_z = self.read_gyro_data()
            timestamp = datetime.now()
            data_point = f"{timestamp.isoformat()} {gyro_x} {gyro_y} {gyro_z}\n"

            with self.lock:
                self.data.append(data_point)
                # Mantém apenas os últimos 10 minutos de dados
                ten_minutes_ago = timestamp - timedelta(minutes=10)
                self.data = [d for d in self.data if datetime.fromisoformat(d.split()[0]) > ten_minutes_ago]

                with open(self.file_path, 'w') as f:
                    f.writelines(self.data)

            time.sleep(1)  # Ajuste o intervalo conforme necessário

    def start_gyro_record(self):
        if not self.recording:
            self.recording = True
            self.recording_thread = threading.Thread(target=self.record_gyro_data)
            self.recording_thread.start()

    def stop_gyro_record(self):
        if self.recording:
            self.recording = False
            self.recording_thread.join()

class MP3Player:
    def __init__(self, file_path):
        pygame.mixer.init()
        self.file_path = file_path
        self.is_playing = False
        self.thread = None

    def play(self):
        if not self.is_playing:
            self.is_playing = True
            self.thread = threading.Thread(target=self._play_sound)
            self.thread.start()

    def _play_sound(self):
        pygame.mixer.music.load(self.file_path)
        pygame.mixer.music.play(-1)  # -1 para tocar em loop
        while self.is_playing:
            time.sleep(0.1)

    def stop(self):
        if self.is_playing:
            self.is_playing = False
            pygame.mixer.music.stop()
            if self.thread:
                self.thread.join()


class Threading_Capture:
    def __init__(self, cap, max_queue_size=1):
        self.video = cap
        self.stopped = False
        self.frame = None

    def start(self):
        # process = multiprocessing.Process(target=self.update, args=(queue_from_cam, ), daemon=True)
        thread = threading.Thread(target=self.update, daemon=True)
        # process.start()
        thread.start()
        return self

    def update(self):  # , queue_from_cam):
        while True:
            try:
                if self.stopped:
                    return

                ok, frame = self.video.read()
                self.frame = frame
                # queue_from_cam.put(frame)

                if not ok:
                    self.stop()
                    return
            except cv2.error:
                print("cv2.error")

            except KeyboardInterrupt:
                break

        self.video.release()

    def read(self):
        # from_queue = queue_from_cam.get()
        if self.frame is None:
            # if from_queue is None:
            return False, None
        else:
            return True, self.frame  # from_queue

    def stop(self):
        self.stopped = True

    def release(self):
        self.stopped = True
        self.video.release()

    def isOpened(self):
        return self.video.isOpened()

    def get(self, i):
        return self.video.get(i)


class Threading_CurveTrigger:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.left_sensor_GPIO = 23
        self.right_sensor_GPIO = 24
        self.left_sensor_status = False
        self.right_sensor_status = False
        GPIO.setup(self.left_sensor_GPIO, GPIO.IN)
        GPIO.setup(self.right_sensor_GPIO, GPIO.IN)

    def start(self):
        thread = threading.Thread(target=self.capture_sensor, daemon=True)
        thread.start()
        return self

    def capture_sensor(self):
        while True:
            try:
                
                self.left_sensor_status = bool(GPIO.input(23))
                self.right_sensor_status = bool(GPIO.input(24))
                #print("LEFT",self.left_sensor_status,"RIGHT",self.right_sensor_status)
                time.sleep(0.05)
            except:
                pass

    def read(self):
        return self.left_sensor_status, self.right_sensor_status

RUN_CMD_INTERVAL = 0.001


class NAVI:
    def __init__(self, camera_front_id):
        self.mainframe = None
        self.mainret = False
        self.route_frame = None
        self.temp_frame = None
        self.direction = ""
        self.tilt_command_time = datetime.now() - timedelta(seconds=60)
        self.curve_trigger = False

        self.motor_speed_left = 300
        self.motor_speed_right = 300

        self.current_arucoID = None
        self.navi_is_active = True
        

        self.current_direction = "FRONT"
        self.camera_front_id = camera_front_id
        self.camera_front = cv2.VideoCapture(self.camera_front_id)
        self.camera_front.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.capture_front = Threading_Capture(self.camera_front)
        self.capture_front.start()
        self.get_main_frame()

        self.curve_sensor = Threading_CurveTrigger()
        self.curve_sensor.start()

        self.lidar = LIDAR()
        self.lidar.run()

        self.resultframe_aruco = None
        t = threading.Timer(RUN_CMD_INTERVAL, self.visualizator)
        t.start()
        



    def visualizator(self):

        while True:
            if self.lidar.lidar_img is not None:
                cv2.imshow("Lidar",self.lidar.lidar_img)
            
            if self.route_frame is not None:
                cv2.imshow("Route", self.route_frame)
            if self.temp_frame is not None:
                cv2.imshow("temp_frame", self.temp_frame)
            cv2.waitKey(1)

    def start_navi(self):
        self.navi_is_active = True
        self.find_aruco_marks()
        self.find_route()
        

    def stop_navi(self):
        self.navi_is_active = False

    def set_direction(self, orientation):
        # Accept only "FRONT" or "REAR"
        self.current_direction = orientation

    def get_main_frame(self):
        ret, frame = self.capture_front.read()
        if ret:
            self.mainframe = frame.copy()
        self.mainret = ret
        t = threading.Timer(RUN_CMD_INTERVAL, self.get_main_frame)
        t.start()

    def get_curve_sensor_status(self):
        return self.curve_sensor.read()

    def find_aruco_marks(self):
        if not self.mainret:
            return
        aruco_frame = self.mainframe.copy()
        corners, ids, self.resultframe_aruco = image_processing.find_aruco(aruco_frame)
        if ids is not None:
            if self.current_arucoID is None:
                self.current_arucoID = ids[0, 0]

        else:
            self.current_arucoID = None
        if self.navi_is_active:
            t = threading.Timer(RUN_CMD_INTERVAL, self.find_aruco_marks)
            t.start()

    def find_route(self):

        if not self.mainret:
            if self.navi_is_active:
                tt = threading.Timer(RUN_CMD_INTERVAL, self.find_route)
                tt.start()
            return
        orientation = ""
        route_frame = self.mainframe.copy()
        route_frame_preview = route_frame.copy()
        height, width, channels = route_frame.shape
        largura, altura = 180, 120  # Largura e altura da ROI
        threshold = int((largura * altura) * 0.05)
        x_inicial, y_inicial = int((width / 2)) - int((largura / 2)), 10  # Coordenadas do canto superior central da ROI
        x_final, y_final = int((x_inicial + largura)), int((y_inicial + altura))
        find_line_top = image_processing.find_line(route_frame[y_inicial:y_final, x_inicial:x_final], threshold)
        route_frame[y_inicial:y_final, x_inicial:x_final] = cv2.cvtColor(find_line_top[3], cv2.COLOR_GRAY2BGR)
        if find_line_top[0]:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 255, 0), 2)
        else:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 0, 255), 2)

        x_inicial, y_inicial = int((width / 2)) - int((largura / 2)), int((height - 10 - altura))
        x_final, y_final = int((x_inicial + largura)), int((y_inicial + altura))
        find_line_bottom = image_processing.find_line(route_frame[y_inicial:y_final, x_inicial:x_final], threshold)
        route_frame[y_inicial:y_final, x_inicial:x_final] = cv2.cvtColor(find_line_bottom[3], cv2.COLOR_GRAY2BGR)
        if find_line_bottom[0]:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 255, 0), 2)
        else:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 0, 255), 2)

        x_inicial, y_inicial = 10, int(((height / 2) - (altura / 2)))
        x_final, y_final = int(x_inicial + largura), int(y_inicial + altura)
        find_line_left = image_processing.find_line(route_frame[y_inicial:y_final, x_inicial:x_final], threshold)
        route_frame[y_inicial:y_final, x_inicial:x_final] = cv2.cvtColor(find_line_left[3], cv2.COLOR_GRAY2BGR)
        if find_line_left[0]:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 255, 0), 2)
        else:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 0, 255), 2)

        x_inicial, y_inicial = int(width - 10 - largura), int((height / 2) - (altura / 2))
        x_final, y_final = int(x_inicial + largura), int(y_inicial + altura)
        find_line_right = image_processing.find_line(route_frame[y_inicial:y_final, x_inicial:x_final], threshold)
        route_frame[y_inicial:y_final, x_inicial:x_final] = cv2.cvtColor(find_line_right[3], cv2.COLOR_GRAY2BGR)
        if find_line_right[0]:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 255, 0), 2)
        else:
            cv2.rectangle(route_frame, (x_inicial, y_inicial), (x_final, y_final), (0, 0, 255), 2)
        top = find_line_top[0]
        bottom = find_line_bottom[0]
        left = find_line_left[0]
        right = find_line_right[0]

        if (top, bottom, left, right) == (True, False, False, False):
            orientation = find_line_top[5]
            x = find_line_top[1][0]
            self.motor_speed_left = 300
            self.motor_speed_right = 300
            if x <= 70:
                self.motor_speed_left = 100
            elif x >= 110:
                self.motor_speed_right = 100
            self.direction = "Front Slow"
        elif (top, bottom, left, right) == (True, True, False, False):
            orientation = find_line_top[5]
            x = find_line_top[1][0]
            self.motor_speed_left = 300
            self.motor_speed_right = 300
            if x <= 70:
                self.motor_speed_left = 100
            elif x >= 110:
                self.motor_speed_right = 100
            self.direction = "Front Normal"
        elif (top, bottom, left, right) == (False, True, False, False):
            orientation = find_line_bottom[5]
            x = find_line_top[1][0]
            self.motor_speed_left = 300
            self.motor_speed_right = 300
            if x <= 70:
                self.motor_speed_left = 100
            elif x >= 110:
                self.motor_speed_right = 100
            self.direction = "Front Slow END ROAD "
        elif (top, bottom, left, right) == (False, False, False, False):
            self.motor_speed_right = 50
            self.motor_speed_right = 50
            self.direction = "OffTrack"
        elif (top, bottom, left, right) == (True, True, True, False):
            self.motor_speed_left = -300
            self.motor_speed_right = 300
            self.direction = "Front or LEFT"
        elif (top, bottom, left, right) == (False, True, True, False):
            self.motor_speed_left = -300
            self.motor_speed_right = 300
            self.direction = "Left"
        elif (top, bottom, left, right) == (False, False, True, False):
            self.motor_speed_left = -100
            self.motor_speed_right = 100
            self.direction = "Left Slow"
        elif (top, bottom, left, right) == (True, True, False, True):
            self.motor_speed_left = 300
            self.motor_speed_right = -300
            self.direction = "Front or Right"
        elif (top, bottom, left, right) == (False, True, False, True):
            self.motor_speed_left = 300
            self.motor_speed_right = -300
            self.direction = "Right"
        elif (top, bottom, left, right) == (False, False, False, True):
            self.motor_speed_left = 100
            self.motor_speed_right = -100
            self.direction = "Right Slow"

        self.route_frame = route_frame.copy()
        text1 = str(orientation)
        cv2.putText(self.route_frame, text1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text2 = self.direction
        cv2.putText(self.route_frame, text2, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text3 = "Right Motor: " + str(self.motor_speed_right) + "rpm"
        cv2.putText(self.route_frame, text3, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text4 = "Left Motor: " + str(self.motor_speed_left) + "rpm"
        cv2.putText(self.route_frame, text4, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        if self.navi_is_active:
            tt = threading.Timer(RUN_CMD_INTERVAL, self.find_route)
            tt.start()

    def get_route(self):
        return self.direction, self.motor_speed_left, self.motor_speed_right

# # Criar uma instância da classe NAVI
# navi = NAVI(0)
#
# # Iniciar a navegação
# navi.start_navi()
#
# # Manter o script em execução (modifique conforme necessário)
# try:
#     while True:
#         time.sleep(1)
# except KeyboardInterrupt:
#     # Parar a navegação quando o script for interrompido
#     navi.stop_navi()
#     print("Navegação interrompida")
