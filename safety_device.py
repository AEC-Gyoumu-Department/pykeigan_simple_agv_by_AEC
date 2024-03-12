"""
Created on February 2024
@author: Ichikawa Eisei
"""
import threading
import time
from gpiozero import DistanceSensor

class Ultrasonic_sensor:
    def __init__(self, trig_pin, echo_pin):
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4)  # assumindo max_distance de 4 metros
        self.distance = None
        self._running = True

    def get_distance(self):
        return self.distance * 100  # convertendo de metros para centímetros

    def run(self):
        while self._running:
            self.distance = self.sensor.distance
            time.sleep(0.1)  # tempo de atualização em segundos

    def stop(self):
        self._running = False
        
#Forma de uso
#if __name__ == "__main__":
#    sensor = Ultrasonic_sensor(trig_pin=17, echo_pin=18)
#    thread = threading.Thread(target=sensor.run)
#    thread.start()
#
#    try:
#        while True:
#            # Seu código principal aqui
#            print("Distância:", sensor.get_distance(), "cm")
#            time.sleep(1)
#    except KeyboardInterrupt:
#        sensor.stop()
#        thread.join()

class Emergency_Stop:
    def __init__(self, stop_btn_pin):
        self.emergency_btn = stop_btn_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.emergency_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    def emergency_button_is_pressed():
        return GPIO.input(botao_pin)
