import threading
import simpleaudio as sa
import numpy as np
import time

class SoundPlayer(threading.Thread):
    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        wave_obj = sa.WaveObject.from_wave_file(self.file_path)
        play_obj = wave_obj.play()
        play_obj.wait_done()

        while not self._stop_event.is_set():
            play_obj = wave_obj.play()
            play_obj.wait_done()

    def play(self):
        wave_obj = sa.WaveObject.from_wave_file(self.file_path)
        play_obj = wave_obj.play()
        play_obj.wait_done()

    def play_loop(self):
        while not self._stop_event.is_set():
            self.play()
