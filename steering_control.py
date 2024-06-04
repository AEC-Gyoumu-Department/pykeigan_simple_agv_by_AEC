#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Dec 14 2021
@author: Keigan
Modified on Mar 01 2024
@Contributor: Ichikawa Eisei
"""
import threading

from pykeigan import usbcontroller
from pykeigan import utils
import time

"""
デバイスアドレス（ポート）は固有IDで指定する
----------------------
モーターへの接続
----------------------
    モーターのデバイスファイル指定について
        "/dev/ttyUSB0"で表示されるデバイス名での接続は、複数のモーターを接続した場合に変わる可能性がある。
        複数のモーターがある場合で、モーターを特定して接続する場合は "$ls /dev/serial/by-id/" で表示されるデバイスを使用する。
            ex)/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00LSSA-if00-port0
"""

RUN_CMD_INTERVAL = 0.1


# 2輪駆動車のクラス Two Wheel Drive Car
class ENGINE:

    def __init__(self, port_left, port_right, safe_time=1, button_event_cb=None):
        self.MAX_RPM = 300
        self.MIN_RPM = 150
        self.left_motor = usbcontroller.USBController(port_left, False)
        self.right_motor = usbcontroller.USBController(port_right, False)
        self.left_rpm = 0
        self.right_rpm = 0
        self.set_rpm(0,0)
        self.safe_time = safe_time
        self.is_running = False

    def get_maxRPM(self):
        return self.MAX_RPM

    def get_minRPM(self):
        return self.MIN_RPM

    def enable(self):  # モーター動作の許可
        #self.safe_setting(True)  # 安全装置開始
        self.left_motor.enable_action()
        self.right_motor.enable_action()
        self.set_rpm(0,0)

    def disable(self):  # モーターの動作不許可
        self.left_motor.disable_action()
        self.right_motor.disable_action()

    def button_setting(self, button_event_cb):  # KeiganMotor のボタンを有効化する
        pass

    def led(self, state, r, g, b):  # LEDの色変更 state = 1 が点灯, 0 は消灯, 2 は点滅
        self.left_motor.set_led(state, r, g, b)
        self.right_motor.set_led(state, r, g, b)

    def motors_is_alive(self):
        if self.left_motor.serial.isOpen() and self.right_motor.serial.isOpen():
            return True
        return False

    def set_rpm(self, left_rpm, right_rpm):
        self.left_rpm = left_rpm
        self.right_rpm = right_rpm
        self.left_motor.run_at_velocity(utils.rpm2rad_per_sec(-self.left_rpm))
        self.right_motor.run_at_velocity(utils.rpm2rad_per_sec(self.right_rpm))

    def drive(self):  #左右のモーターを指定の速度rpmで動作させる
        self.left_motor.run_at_velocity(utils.rpm2rad_per_sec(-self.left_rpm))
        self.right_motor.run_at_velocity(utils.rpm2rad_per_sec(self.right_rpm))

    def move_on(self):
        self.is_running = True
        self.running()

    def running(self):
        if self.is_running:
            self.drive()
            t = threading.Timer(RUN_CMD_INTERVAL, self.running)
            t.start()

    def park(self, timeout=0):  # timeout[s] だけその場で停止する（トルクあり）
        self.safe_setting(False)  # 安全装置解除
        self.left_motor.stop_motor()
        self.right_motor.stop_motor()
        # timeout == 0 の場合、安全装置は解除されたまま、stop（rpm = 0 速度制御）を続ける
        if timeout > 0:
            time.sleep(timeout)
            self.safe_setting(True)

    def neutral(self, timeout=0):  # # timeout[s] だけモーターフリー状態（粘性トルクあり）
        self.safe_setting(False)  # 安全装置解除
        self.left_motor.free_motor()
        self.right_motor.free_motor()
        # timeout == 0 の場合、安全装置は解除されたまま、free を続ける   
        if timeout > 0:
            time.sleep(timeout)
            self.safe_setting(True)  # 安全装置再開

    def safe_park(self, isEnabled):  # モーションントロールの台形速度カーブを使わない(0)
        # 第1引数が True safe_time[s]以内に次の動作命令が来ないと、停止する 0:free,1:disable,2:stop, 3:position固定
        if isEnabled:
            pass
            self.left_motor.set_safe_run_settings(True, self.safe_time * 1000, 2)
            self.right_motor.set_safe_run_settings(True, self.safe_time * 1000, 2)
        else:
            self.left_motor.set_safe_run_settings(False, self.safe_time * 1000, 2)
            self.right_motor.set_safe_run_settings(False, self.safe_time * 1000, 2)

    def spin_left(self):
        left_rpm = self.left_rpm * -1
        right_rpm = self.right_rpm
        self.set_rpm(left_rpm, right_rpm)

    def spin_right(self):
        left_rpm = self.left_rpm
        right_rpm = self.right_rpm * -1
        self.set_rpm(left_rpm, right_rpm)

    def forward(self):
        left_rpm = abs(self.left_rpm)
        right_rpm = abs(self.right_rpm)
        self.set_rpm(left_rpm, right_rpm)

    def backward(self):
        left_rpm = -abs(self.left_rpm)
        right_rpm = -abs(self.right_rpm)
        self.set_rpm(left_rpm, right_rpm)