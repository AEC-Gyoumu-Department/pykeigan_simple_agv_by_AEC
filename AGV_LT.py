#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Dec 14 2021
@author: Takashi Tokuda
Modified on Mar 01 2024
@Contributor: Ichikawa Eisei
"""

# import the necessary packages
import time
import threading  # タイマー用 SignalライブラリがOpenCVと一緒に使えないため
import cv2  
import numpy as np 
from enum import Enum 
import RPi.GPIO as GPIO  
import csv 
from pykeigan import usbcontroller
from pykeigan import utils
from threading_capture import threading_capture
from safety_device import Ultrasonic_sensor
from sound_controller import SoundPlayer
import tkinter as tk
from tkinter import filedialog
import datetime
import subprocess
# ボタン（赤黄緑）
BUTTON_RED_PIN = 13
BUTTON_RED_PIN_2 = 20  # ２つ目の赤ボタンを追加
BUTTON_YELLOW_PIN = 19
BUTTON_GREEN_PIN = 26

# エリアセンサー　PIN　(HC-SR04 sensor)
SENSOR1_TRIGGER_PIN = 22
SENSOR1_ECHO_PIN = 23

area_sensor = Ultrasonic_sensor(trig_pin=SENSOR1_TRIGGER_PIN, echo_pin=SENSOR1_ECHO_PIN)
thread = threading.Thread(target=area_sensor.run)
thread.start()
# USBカメラ
"""
以下のコマンドで使用できるUSBカメラのリストを取得
$ v4l2-ctl --list-devices
インストール必要
$ sudo apt-get install v4l-utils
"""
#
CAM_U1_FRONT_ID = 0  # USBcam1 /dev/video0
#CAM_U1_REAR_ID = 2   # USBcam2 /dev/video1
CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 30

# フレームレート計算用
tm = cv2.TickMeter()
tm.start()

count = 0
max_count = 10
fps = 0
#Set disable auto exposure time and set that to 400
subprocess.check_output("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1", shell=True)
subprocess.check_output("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=400", shell=True)

subprocess.check_output("v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0", shell=True)
subprocess.check_output("v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=5000", shell=True)

camera_front = cv2.VideoCapture(CAM_U1_FRONT_ID)

camera_front.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
camera_front.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
camera_front.set(cv2.CAP_PROP_FPS, CAM_FPS)
# camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'));
camera_front.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'));
camera_front.set(cv2.CAP_PROP_BUFFERSIZE, 1)
capture_front = threading_capture(camera_front)
capture_front.start()

#camera_rear = cv2.VideoCapture(CAM_U1_REAR_ID)
#camera_rear.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
#camera_rear.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
#camera_rear.set(cv2.CAP_PROP_FPS, CAM_FPS)
# camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'));
#camera_rear.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'));
##camera_rear.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#capture_rear = threading_capture(camera_rear)
#capture_rear.start()
time.sleep(0.5)  # カメラのウォームアップ時間
# ライントレーサー
"""
トラックバー（Mainウインドウ下のスライダー）により決定される、
HSV値の範囲の色をラインとして認識する
"""
# 領域分離を行った後、この面積を超えた領域のみ処理を行う
LINE_AREA_THRESHOLD = 3500 / 4  # ライン検知用の面積の閾値
LINE_CROSS_PASS_AREA_THRESHOLD = 20000 / 4  # ラインを横切った場合に前回のライン位置を採用するための面積の閾値
LINE_UPPER_AREA_THRESHOLD = 5500 / 4
STOP_MARKER_AREA_THRESHOLD = 20000 / 4  # 停止テープマーカーを検知するための面積の閾値（※テープ, arucoマーカーではない）

RUN_CMD_INTERVAL = 0.1  # 0.1秒ごとに処理を行う
RUN_BASE_RPM = 300
MOTOR_L_BASE_SPEED = 300
MOTOR_R_BASE_SPEED = -180
RUN_LOWER_RPM = 15
STOP_AFTER_RPM = 10
STOP_AFTER_RPM1 = 5

# 負荷有無で PIDコントローラゲインを変更するため（デフォルトは未使用）
hasPayload = False  # 負荷あり: True, 負荷なし: False

# マーカーなどで停止する場合に関する変数
isPausingLinetrace = False  # マーカー発見等で停止すべき場合 True
isResuming = False  # 停止→ライントレース動作再開までの判定状態
RESUME_THRESHOLD = 10  # resumeCounter がこの回数以上の場合、動作再開する（動作しても良い）
resumeCounter = 0  # 動作再開用のカウンタ
# ドッキング中であることを示す
isDocking = False  # ドッキング中なら True
dockingCounter = 0  # ドッキング中ロストカウンタ
DOCKING_THRESHOLD = 5  # dockingCounter がこの回数以上の場合、moveStraight でC箱を実際につかみにいく

# ラインロスト（OFFにしている）
is_lost = False  # Trueならば、ラインがロストしている
lost_count = 0  # ラインをロストしたカウント
LOST_THRESHOLD = 7  # ラインをロストしたとみなす判定の閾値
lost_total_count = 0  # ラインをロストした回数の合計
LOST_TOTAL_THRESHOLD = 5  # ラインをロストした回数の合計がこの値以上になると、AGVはアイドル状態に戻る

# PID limit
DELTA_MAX = 25
# PIDコントローラのゲイン値：負荷なし
steer_p = 0.03  # 0.05 比例
steer_i = 0.05  # 0.002 積分
steer_d = 0  # 微分
# PIDコントローラのゲイン値：負荷あり
steer_load_p = 0.80  # 比例
steer_load_i = 0.5  # 積分
steer_load_d = 0  # 微分
x = 0  # ライン位置
x_old = 0  # ラインの前回の位置を保存しておく
CHARGING_TIME_SEC = 10  # 充電ステーションでの待機時間

# run rpm variable
run_rpm = RUN_BASE_RPM

# player  object
player = SoundPlayer()

# ID to identify this AGV in traffics map
AGV_ID = "DreamySmurf"

AGV_direction = 1
line_color = "black"

# システムの状態を表す列挙子クラス
class State(Enum):
    """システムのステート（状態）を表す列挙子
     状態遷移を管理するため
    Attributes:
        State (Enum): システムのステート（状態）
    """
    STATE_IDLE = 0  # 待機状態
    STATE_LINE_TRACE = 1  # ライントレーサー
    STATE_DEBUG = 10  # デバッグ用
    STATE_MOTOR_REINITIALIZE = 20


# KeiganMotor 本体のボタンから、システムのステートをセットする
def set_state_by_button(event):
    # ■ 停止ボタンでアイドル状態へ（停止）
    # ▶ 再生ボタンでライントレース開始
    if event['event_type'] == 'button':
        if event['number'] == 2:
            set_state(State.STATE_IDLE)
        elif event['number'] == 3:
            set_state(State.STATE_LINE_TRACE)
            
def calculate_speed_right(max_speed,image_width, object_center):
    # Calculate the distance from the object center to the image center
    image_center_x = image_width / 2
    distance_from_center = object_center - image_center_x
    # If the object's center is outside the image, the speed is zero
    if object_center <= 0 or object_center >= image_width:
        return 0
    # Calculate the speed based on the distance from the image center
    # If the object moves to the left and crosses the image center, the speed is 300
    if distance_from_center < 0:
        return max_speed
    # Adjust the reduction factor as needed to achieve the desired reduction
    reduction_factor = max_speed / image_center_x
    speed = max_speed - (distance_from_center * reduction_factor)
    return speed

def calculate_speed_left(max_speed,image_width, object_center):
    # Calculate the distance from the object center to the image center
    image_center_x = image_width / 2
    distance_from_center =   image_center_x  - object_center
    # If the object's center is outside the image, the speed is zero
    if object_center <= 0 or object_center >= image_width:
        return 0
    # Calculate the speed based on the distance from the image center
    # If the object moves to the left and crosses the image center, the speed is 300
    if distance_from_center < 0:
        return max_speed
    # Adjust the reduction factor as needed to achieve the desired reduction
    reduction_factor = max_speed / image_center_x
    speed = max_speed - (distance_from_center * reduction_factor)
    return speed

def calculate_speed(max_speed,image_width, object_center):
    # Calculate the relative position of the object's center compared to the image center
    image_center_x = image_width / 2
    distance_from_center = object_center - image_center_x
    # If the object's center is outside the image, the speed is zero
    if object_center <= 0 or object_center >= image_width:
        return 0
    # Calculate the speed based on the object's center position
    # Adjust the reduction factor as needed to achieve the desired reduction
    reduction_factor = max_speed / image_center_x
    speed = max_speed - (abs(distance_from_center) * reduction_factor)
    return speed

def motor_event_cb(event):
    # print("event")
    set_state_by_button(event)

"""
ターミナルで $ls /dev/serial/by-id/ 
で表示されるデバイスアドレス（デバイスファイル）を記録する
usb-FTDI_FT230X_Basic_UART_DM00LSSA-if00-port0
"""

# KeiganMotor デバイスアドレス（上記参照）
port_left = '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00KNUU-if00-port0'
port_right = '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00KX3V-if00-port0'

motor_left = usbcontroller.USBController(port_left)
motor_right = usbcontroller.USBController(port_right)
motor_left.set_curve_type(0)
motor_right.set_curve_type(0)

cur_state = State.STATE_IDLE  # システムの現在の状態


def set_state(state: State):
    """システムのステートをセットする
    同時に、モーターのLEDをステートに応じて色変更する
    Args:
        state (State): ステート（状態）
    """
    global cur_state, actionFlag, motor_left, motor_right
    cur_state = state

    if state == State.STATE_IDLE:
        print("-> State.STATE_IDLE")
        actionFlag = "停止"
        motor_left.disable_action()
        motor_right.disable_action()
        motor_left.set_led(2, 255, 0, 0)
        motor_right.set_led(2, 255, 0, 0)
        player.set_no_sound()
    elif state == State.STATE_LINE_TRACE:
        print("-> State.STATE_LINE_TRACE")
        motor_left.enable_action()
        motor_right.enable_action()
        motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
        motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED )) 
        t = threading.Thread(target=scheduler)
        t.start()
        player.set_normal_move_sound()
        actionFlag = ""
        
# 赤黄緑ボタンを押したときのコールバック関数
def rear_callback(gpio_pin):
    global AGV_direction
    if GPIO.input(gpio_pin) == GPIO.LOW:
        AGV_direction = -1
        set_state(State.STATE_LINE_TRACE)
        print("green pushed")


def red_callback(gpio_pin):
    time.sleep(0.05)
    if GPIO.input(gpio_pin) == GPIO.LOW:
        set_state(State.STATE_IDLE)
        print("red pushed")


def yellow_callback(gpio_pin):
    time.sleep(0.05)
    print("yellow pushed: nothing")


def green_callback(gpio_pin):
    time.sleep(0.05)
    if GPIO.input(gpio_pin) == GPIO.LOW:
        set_state(State.STATE_LINE_TRACE)
        print("green pushed")


# 重心の検出
def get_moment(mask, threshold):
    # 面積・重心計算付きのラベリング処理を行う
    num_labels, label_image, stats, center = cv2.connectedComponentsWithStats(mask)
    # 最大のラベルは画面全体を覆う黒なので不要．データを削除
    num_labels = num_labels - 1
    stats = np.delete(stats, 0, 0)
    center = np.delete(center, 0, 0)
    isExist = False
    x1, y1 = 0, 0
    area = 0

    if num_labels > 0:
        # 面積最大のラベル
        max_label = np.argmax(stats[:, 4])
        area = stats[max_label][4]
        #print("get moment Area:",area)
        x1, y1 = int(center[max_label][0]), int(center[max_label][1])
        #print("X1",x1)
        if area > threshold:
            x1, y1 = int(center[max_label][0]), int(center[max_label][1])
            cv2.circle(mask, (x1, y1), 4, 100, 2, 4)
            isExist = True
    cv2.imshow("Trace", mask)
    return isExist, (x1, y1), area

# 青色の重心の検出
# 存在する場合 true と、重心座標と、面積を返す
def get_color_moment(roi_img):
    global line_color
    
    if line_color == "black":
        # Converter para escala de cinza
        imagem_cinza = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        # Limiarização
        _, imagem_binaria = cv2.threshold(imagem_cinza, 40, 255, cv2.THRESH_BINARY)#buffalo = 127
        # Encontrar contornos
        contornos, _ = cv2.findContours(imagem_binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Criar uma máscara em branco do mesmo tamanho da imagem original
        mask = np.zeros_like(imagem_cinza)
        # Desenhar os contornos na máscara
        cv2.drawContours(mask, contornos, -1, (255), -1)  # -1 para desenhar todos os contornos
        mask = cv2.bitwise_not(mask)
    else:
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("(Trace)_H_min", "Trace")
        h_max = cv2.getTrackbarPos("(Trace)_H_max", "Trace")
        s_min = cv2.getTrackbarPos("(Trace)_S_min", "Trace")
        v_min = cv2.getTrackbarPos("(Trace)_V_min", "Trace")
        hsv_min = np.array([h_min, s_min, v_min])
        hsv_max = np.array([h_max, 255, 100]) 
        #hsv_min = np.array([0, 0, 0])
        #hsv_max = np.array([180, 255, 50])
        # 青色領域のマスク
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
    #cv2.imshow("Trace", mask)
    return get_moment(mask, LINE_AREA_THRESHOLD)

# ラインの位置から左右のモーターに与える回転速度rpmを計算する
def pid_controller():
    global x,CAM_WIDTH,MOTOR_L_BASE_SPEED,MOTOR_R_BASE_SPEED
    motor_left_speed = calculate_speed_left(MOTOR_L_BASE_SPEED,CAM_WIDTH,x)
    motor_right_speed = calculate_speed_right(MOTOR_R_BASE_SPEED,CAM_WIDTH,x)

    rpm = (motor_left_speed, motor_right_speed)
    return rpm

def scheduler():
    global cur_state, isPausingLinetrace, isDocking, AGV_direction,motor_left,motor_right
    global x, x_old
    
    if cur_state == State.STATE_IDLE:
        return
    # タイマーの再生成
    t = threading.Timer(RUN_CMD_INTERVAL, scheduler)
    t.start()
    rpm = pid_controller()  # PIDコントローラに突っ込む
    if isPausingLinetrace:  # マーカー検知時など停止するべき場合
        return
    if isDocking:
        rpm = (rpm[0] * 0.5, rpm[1] * 0.5)  # ドッキング時は半分の速度にする

    if cur_state == State.STATE_LINE_TRACE:
        #print("LEFT_MOTOR_SPEED",rpm[0])
        #print("RIGHT_MOTOR_SPEED",rpm[1])
        motor_left.run_at_velocity(utils.rpm2rad_per_sec(rpm[0]))
        motor_right.run_at_velocity(utils.rpm2rad_per_sec(rpm[1]))
        

# トラックバーのコールバック関数は何もしない空の関数
def nothing(x):
    pass

# aruco マーカー
# aruco マーカーの辞書定義
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# arucoマーカーを検知する
def aruco_reader(roi_ar):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(roi_ar, dictionary)
    cv2.aruco.drawDetectedMarkers(roi_ar, corners, ids, (0, 255, 0))
    cv2.imshow('detectedMakers', roi_ar)
    return corners, ids


# フレームレートの計算
def calc_frame_rate():
    global count, max_count, tm, fps
    if count == max_count:
        tm.stop()
        fps = max_count / tm.getTimeSec()
        tm.reset()
        tm.start()
        count = 0
    count += 1
    return fps

if __name__ == '__main__':
    print("青木電器 AGV START!")

    # GPIOをBCM番号で呼ぶことを宣言
    GPIO.setmode(GPIO.BCM)
    # ボタン入力の設定
    GPIO.setup(BUTTON_RED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_RED_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_YELLOW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_GREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # ボタンを押したときのコールバックを登録
    GPIO.add_event_detect(BUTTON_RED_PIN, GPIO.FALLING, callback=red_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_RED_PIN_2, GPIO.FALLING, callback=red_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_YELLOW_PIN, GPIO.FALLING, callback=yellow_callback, bouncetime=50)
    GPIO.add_event_detect(BUTTON_GREEN_PIN, GPIO.FALLING, callback=green_callback, bouncetime=50)
    set_state(State.STATE_IDLE)  # アイドル状態でスタート

    print("キーボードの [s] + Enter または 赤ボタン: ストップ STATE_IDLE")
    print("キーボードの [t] + Enter または 緑ボタン: ライントレース STATE_LINE_TRACE")
    print("キーボードの [d] + Enter :デバッグ用 STATE_DEBUG")

    # 画像表示用ウィンドウの生成 なくても動くが、ウィンドウにフォーカスさせてキー入力を受け付けるため必要
    cv2.namedWindow("Main")
    # PIDコントローラのゲイン調整
    cv2.createTrackbar("Gain_P", "Main", 10, 100, nothing)
    cv2.createTrackbar("Gain_I", "Main", 5, 100, nothing)
    cv2.createTrackbar("Gain_Load_P", "Main", 15, 100, nothing)
    cv2.createTrackbar("Gain_Load_I", "Main", 7, 100, nothing)

    # 青ライントレース確認用ウィンドウ
    cv2.namedWindow("Trace")  # なくても動くが、ウィンドウにフォーカスさせてキー入力を受け付けるため必要
    # 青ラインのHSV抽出領域設定
    cv2.createTrackbar("(Trace)_H_min", "Trace", 0, 179, nothing) #90
    cv2.createTrackbar("(Trace)_H_max", "Trace", 179, 179, nothing) #130
    cv2.createTrackbar("(Trace)_S_min", "Trace", 27, 255, nothing) #64
    cv2.createTrackbar("(Trace)_V_min", "Trace", 0, 255, nothing) #20

    # 停止条件を検知したカウント数（＝片道周回数になる）
    stop_marker_count = 0
    actionFlag = "停止"  # 停止
    try:
        while True:
            # カメラからフレームをキャプチャする
            #if AGV_direction > 0:
            ret, frame = capture_front.read()  # capture
            #else:
            #    ret, frame = capture_rear.read()  # capture
            if not ret:
                break
            
            image = frame.copy()
            roi = image[int((CAM_HEIGHT/2)+1):CAM_HEIGHT-1, 0:630]  # [180:230, 0:320] #[95:145, 0:320]
            #roi_u = image[45:95, 0:630]  # [30:80, 0:320] # [45:95, 0:320]
            #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # HSV画像
            #roi_u = cv2.cvtColor(roi_u, cv2.COLOR_BGR2HSV)
            img = cv2.medianBlur(roi, 5)
            #img_u = cv2.medianBlur(roi_u, 5)

            # フレームレートを表示する (frame per seconds)
            tfont = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(roi, 'FPS: {:.2f}'.format(calc_frame_rate()),(10, 30), tfont, 1.0, (0, 255, 0), thickness=2)

            # ライントレース中の動作フラグ
            ## 停止条件は (a) 赤ラインマーカー or (b) arucoマーカー指定id いずれか

            # (a) Arucoマーカー検知で停止を行う場合
            roi_ar = image[80:240, 0:320]  # [80:240, 0:320]
            corners, ids = aruco_reader(image)  # ArUcoマーカー検知
            color = get_color_moment(img)
            #color_u = get_color_moment(img_u)
            if ids is not None:
                if 4 == ids[0, 0]:  # 停止マーク
                    actionFlag = "停止"
                elif 1 == ids[0, 0]:  # モーターを再インスタンスマーク
                    actionFlag = "再インスタンス"
                elif 2 == ids[0, 0]:  # 左折マーク
                    if AGV_direction >0:
                        actionFlag = "左折"
                    else:
                        actionFlag = "右折"
                elif 3 == ids[0, 0]:  # 右折マーク
                    if AGV_direction >0:
                        actionFlag = "右折"
                    else:
                        actionFlag = "左折"
                elif 90 == ids[0, 0]:  # 回転180度マーク
                    actionFlag = "回転180"
                elif 91 == ids[0, 0]:  # 回転270度マーク
                    actionFlag = "回転270"
                elif 5 == ids[0, 0]:
                    actionFlag = "30秒待ち逆動き"

            if area_sensor.get_distance() <= 30:
                pass
                #actionFlag = "一時停止"

            if cur_state == State.STATE_LINE_TRACE or cur_state == State.STATE_DEBUG:
                if actionFlag == "停止":  # 停止マーカーを検知したら、停止して処理
                    #reset_pid_params()
                    isPausingLinetrace = True  # ライントレース一時停止
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec((MOTOR_R_BASE_SPEED/2) *-1 ))
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED/2 ))
                    time.sleep(2.8)
                    set_state(State.STATE_IDLE)
                    AGV_direction = 1
                if actionFlag == "一時停止":  # 停止マーカーを検知したら、停止して処理
                    stop_marker_count += 1
                    print("Detected Stop Marker:", stop_marker_count)
                    #reset_pid_params()
                    isPausingLinetrace = True  # ライントレース一時停止
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(0))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(0))
                    actionFlag = "isResuming"
                    player.set_blocked_move_sound()
                    run_rpm = RUN_BASE_RPM  # 速度を元に戻す

                elif actionFlag == "isResuming":
                    isPausingLinetrace = False
                    isResuming = False
                    player.set_normal_move_sound()
                    actionFlag = ""

                elif actionFlag == "左折":
                    print("Detected Left Turn Marker")
                    isPausingLinetrace = True  # ライントレース停止

                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED ))
                    
                    time.sleep(1.7)
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec((MOTOR_L_BASE_SPEED/2) *-1 ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED/2 ))
 
                    time.sleep(1.3)
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED ))

                    actionFlag = ""
                    isPausingLinetrace = False

                elif actionFlag == "右折":
                    print("Detected Right Turn Marker")
                    isPausingLinetrace = True  # ライントレース停止
                    
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED ))

                    time.sleep(1.6)
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec((MOTOR_R_BASE_SPEED/2) *-1 ))
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec((MOTOR_L_BASE_SPEED/2) ))
 
                    time.sleep(1.3)
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED ))

                    actionFlag = ""
                    isPausingLinetrace = False

                elif actionFlag == "30秒待ち逆動き":
                    print("30秒待ち逆動き")
                    player.set_blocked_move_sound()
                    isPausingLinetrace = True
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(0))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(0))

                    time.sleep(10)
                    
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec((MOTOR_R_BASE_SPEED/2) *-1 ))
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED/2 ))
                    time.sleep(2.6)
                    
                    motor_left.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_L_BASE_SPEED ))
                    motor_right.run_at_velocity(utils.rpm2rad_per_sec(MOTOR_R_BASE_SPEED ))
                    
                    AGV_direction = AGV_direction * -1
                    isPausingLinetrace = False
                    actionFlag = ""

                elif actionFlag == "再インスタンス":
                    motor_left = usbcontroller.USBController(port_left)
                    motor_right = usbcontroller.USBController(port_right)
                else:  # Follow the Blue Brick Road (Line)
                    isLineExist = False  # ラインが存在する場合、True
                    isLineExist = color[0]
                    lineArea = color[2]
                    #lineArea_u = color_u[2]
                    if isLineExist:
                        isPausingLinetrace = False
                        x = color[1][0]
                    else:
                        print("Lost Line")
                        isPausingLinetrace = True
                        motor_left.run_at_velocity(utils.rpm2rad_per_sec(- MOTOR_L_BASE_SPEED *0.1 ))
                        motor_right.run_at_velocity(utils.rpm2rad_per_sec(- MOTOR_R_BASE_SPEED *0.1 ))
                        time.sleep(5)
                                                
            cv2.imshow("Main", roi)
            # cv2.imshow("Raw", image)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("s"):
                set_state(State.STATE_IDLE)
            elif key == ord("t"):
                set_state(State.STATE_LINE_TRACE)
            elif key == ord("d"):
                set_state(State.STATE_DEBUG)
            elif key == ord("w"):
                set_state(State.STATE_MOTOR_REINITIALIZE)
                #twd = TWD(port_left, port_right, wheel_d=100.6, tread=306.5, button_event_cb=motor_event_cb)
            elif key == ord("l"):
                set_state(State.STATE_IDLE)
                file_path = filedialog.askopenfilename()
            elif key == ord("h"):
                file_path = filedialog.asksaveasfilename(defaultextension=".yaml")
            elif key == ord("q"):
                break

    except KeyboardInterrupt:
        twd.disable()
