"""
Created on February 2024
@author: Ichikawa Eisei
"""

import os

class SoundController:
    def __init__(self):
        # Adicione aqui os caminhos para os arquivos de áudio correspondentes
        self.sounds = {
            "reto": "path_to_reto_sound_file.wav",
            "direita": "path_to_direita_sound_file.wav",
            "esquerda": "path_to_esquerda_sound_file.wav",
            "parar": "path_to_parar_sound_file.wav"
        }

    def play_sound(self, sound_type):
        if sound_type in self.sounds:
            os.system("aplay " + self.sounds[sound_type])