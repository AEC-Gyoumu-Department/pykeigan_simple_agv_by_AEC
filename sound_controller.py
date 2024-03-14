import threading
import simpleaudio as sa
import numpy as np
import time

class SoundPlayer(threading.Thread):
    def __init__(self):
        super().__init__()
        self.file_path = r"/sound/normal_move.mp3" 
        self.normal_move_sound =  r"/sound/normal_move.mp3"
        self.blocked_move_sound = r"/sound/blocked_move.mp3"
        self.no_sound = ""
        self._stop_event = threading.Event()

    def set_no_sound(self):
        self.file_path = self.no_sound

    def set_normal_move_sound():
        self.file_path = self.normal_move_sound

    def set_blocked_move_sound():
         self.file_path =  self.blocked_move_sound
    
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
        if self.file_path != "":
            wave_obj = sa.WaveObject.from_wave_file(self.file_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()

    def play_loop(self):
        while not self._stop_event.is_set():
            self.play()
