import smbus2
import time
import threading
from datetime import datetime, timedelta
import os

class GyroRecorder:
    def __init__(self, bus_number=1, device_address=0x68, file_path='gyro_data.txt'):
        self.bus = smbus2.SMBus(bus_number)
        self.device_address = device_address
        self.file_path = file_path
        self.recording = False
        self.data = []
        self.lock = threading.Lock()
        
        # Inicializa o MPU-6050
        self.bus.write_byte_data(self.device_address, 0x6B, 0)

    def read_gyro_data(self):
        # Leitura dos registros de giroscópio
        high_x = self.bus.read_byte_data(self.device_address, 0x43)
        low_x = self.bus.read_byte_data(self.device_address, 0x44)
        high_y = self.bus.read_byte_data(self.device_address, 0x45)
        low_y = self.bus.read_byte_data(self.device_address, 0x46)
        high_z = self.bus.read_byte_data(self.device_address, 0x47)
        low_z = self.bus.read_byte_data(self.device_address, 0x48)
        
        gyro_x = (high_x << 8) | low_x
        gyro_y = (high_y << 8) | low_y
        gyro_z = (high_z << 8) | low_z
        
        if gyro_x >= 0x8000:
            gyro_x = -((65535 - gyro_x) + 1)
        if gyro_y >= 0x8000:
            gyro_y = -((65535 - gyro_y) + 1)
        if gyro_z >= 0x8000:
            gyro_z = -((65535 - gyro_z) + 1)

        return gyro_x, gyro_y, gyro_z

    def record_gyro_data(self):
        while self.recording:
            gyro_x, gyro_y, gyro_z = self.read_gyro_data()
            timestamp = datetime.now()
            data_point = f"{timestamp.isoformat()} {gyro_x} {gyro_y} {gyro_z}\n"
            
            with self.lock:
                self.data.append(data_point)
                # Mantém apenas os últimos 10 minutos de dados
                ten_minutes_ago = timestamp - timedelta(minutes=10)
                self.data = [d for d in self.data if datetime.fromisoformat(d.split()[0]) > ten_minutes_ago]
                print(self.data)
                with open(self.file_path, 'w') as f:
                    f.writelines(self.data)
            
            time.sleep(1)  # Ajuste o intervalo conforme necessário

    def start_gyro_record(self):
        if not self.recording:
            self.recording = True
            self.recording_thread = threading.Thread(target=self.record_gyro_data)
            self.recording_thread.start()

    def stop_gyro_record(self):
        if self.recording:
            self.recording = False
            self.recording_thread.join()

if __name__ == '__main__':
    recorder = GyroRecorder()
    try:
        recorder.start_gyro_record()
        print("Recording started. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recorder.stop_gyro_record()
        print("Recording stopped.")
