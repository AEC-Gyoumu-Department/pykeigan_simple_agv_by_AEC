import threading
import simpleaudio as sa
import numpy as np
import time

class BeepPlayer:
    def __init__(self):
        self.playing = False
        self.thread = None
        self.current_beep = ""

    def beep(self, tmp_frequency, tmp_duration):
        
        frequency = tmp_frequency  # Our played note will be 440 Hz
        fs = 44100  # 44100 samples per second
        seconds = tmp_duration  # Note duration of 3 seconds
        
        # Generate array with seconds*sample_rate steps, ranging between 0 and seconds
        t = np.linspace(0, seconds, seconds * fs, False)
        
        # Generate a 440 Hz sine wave
        note = np.sin(frequency * t * 2 * np.pi)
        
        # Ensure that highest value is in 16-bit range
        audio = note * (2**15 - 1) / np.max(np.abs(note))
        # Convert to 16-bit data
        audio = audio.astype(np.int16)
        
        # Start playback
        play_obj = sa.play_buffer(audio, 1, 2, fs)
        
        # Wait for playback to finish before exiting
        play_obj.wait_done()

    def play_beep_intermittent(self, frequency, duration, interval):
        def play():
            if self.current_beep != "intermittent":
                self.current_beep = "intermittent"
                self.playing = True
                while self.playingand self.current_beep == "intermittent":
                    self.beep(frequency, duration)
                    time.sleep(interval)

        self.thread = threading.Thread(target=play)
        self.thread.start()

    def play_beep_intercalated(self, frequency1, frequency2, duration):
        def play():
            if self.current_beep != "intercalated":
                self.current_beep = "intercalated"
                self.playing = True
                while self.playing and self.current_beep == "intercalated" :
                    self.beep(frequency1, duration)
                    time.sleep(duration)
                    self.beep(frequency2, duration)
                    time.sleep(duration)
                
        self.thread = threading.Thread(target=play)
        self.thread.start()

    def stop_beep(self):
        self.playing = False
        self.current_beep = ""
        if self.thread is not None:
            self.thread.join()

# Exemplo de uso
#beeper = BeepPlayer()
#beeper.play_beep_intermittent(1000, 0.5, 0.5)  # Reproduz beeps de 1000 Hz por 0.5 segundos intercalados com 0.5 segundos de intervalo, indefinidamente

# Para parar a reprodução:
# beeper.stop_beep_intermittent()
