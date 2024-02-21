"""
Created on February 2024
@author: Ichikawa Eisei
"""
from gpiozero import DistanceSensor

class Ultrasonic_sensor:
    def __init__(self, trig_pin, echo_pin):
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4)  # assumindo max_distance de 4 metros

    def get_distance(self):
        return self.sensor.distance * 100  # convertendo de metros para centímetros


Class Emergency_Stop:
    def __init__(self, stop_btn_pin):
        self.emergency_btn = stop_btn_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.emergency_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    def emergency_button_is_pressed():
        return GPIO.input(botao_pin)
