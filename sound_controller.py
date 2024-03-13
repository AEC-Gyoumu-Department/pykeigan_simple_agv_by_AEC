import threading
import simpleaudio as sa
import numpy as np
import time

class BeepPlayer:
    def __init__(self):
        self.playing = False
        self.thread = None

    def beep(self, frequency, duration):
        # Cria um array numpy para representar o som de um beep
        t = np.linspace(0, duration, int(44100 * duration), False)
        beep = np.sin(frequency * t * 2 * np.pi)
        beep = np.int16(beep * 32767)

        # Reproduz o som
        play_obj = sa.play_buffer(beep, 1, 2, 44100)
        play_obj.wait_done()

    def play_beep_intermittent(self, frequency, duration, interval):
        def play():
            self.playing = True
            while self.playing:
                self.beep(frequency, duration)
                time.sleep(interval)

        self.thread = threading.Thread(target=play)
        self.thread.start()

    def stop_beep_intermittent(self):
        self.playing = False
        if self.thread is not None:
            self.thread.join()

    def play_beep_intercalated(self, frequency1, frequency2, duration, total_time):
        def play():
            start_time = time.time()
            while self.playing:
                self.beep(frequency1, duration)
                time.sleep(duration)
                self.beep(frequency2, duration)
                time.sleep(duration)
                
        self.thread = threading.Thread(target=play)
        self.thread.start()

    def stop_beep_intercalated(self):
        self.playing = False
        if self.thread is not None:
            self.thread.join()

# Exemplo de uso
#beeper = BeepPlayer()
#beeper.play_beep_intermittent(1000, 0.5, 0.5)  # Reproduz beeps de 1000 Hz por 0.5 segundos intercalados com 0.5 segundos de intervalo, indefinidamente

# Para parar a reprodução:
# beeper.stop_beep_intermittent()
