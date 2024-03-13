"""
Created on February 2024
@author: Ichikawa Eisei
"""

import os

import simpleaudio as sa
import numpy as np
import time

class BeepPlayer:
    def __init__(self):
        pass

    def beep(self, frequency, duration):
        # Cria um array numpy para representar o som de um beep
        t = np.linspace(0, duration, int(44100 * duration), False)
        beep = np.sin(frequency * t * 2 * np.pi)
        beep = np.int16(beep * 32767)

        # Reproduz o som
        play_obj = sa.play_buffer(beep, 1, 2, 44100)
        play_obj.wait_done()

    def play_two_beep_intercalated(self, frequency1, frequency2, duration, total_time):
        start_time = time.time()
        while time.time() - start_time < total_time:
            self.beep(frequency1, duration)
            time.sleep(duration)
            if time.time() - start_time + duration >= total_time:
                break
            self.beep(frequency2, duration)
            time.sleep(duration)
            
    def play_beep_intermittent(self, frequency, duration, interval, total_time):
        start_time = time.time()
        while time.time() - start_time < total_time:
            self.beep(frequency, duration)
            time.sleep(interval)
            if time.time() - start_time + interval >= total_time:
                break
            time.sleep(interval)

# Use sample
#beeper = BeepPlayer()
#beeper.play_beep_intercalated(1000, 2000, 0.5, 10)  # Alterna entre beeps de 1000 Hz e 2000 Hz por 0.5 segundos, por 10 segundos

