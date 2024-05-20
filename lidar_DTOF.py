


import cv2
import struct
import numpy as np
import serial
from serial.tools import list_ports
import threading
import time


all_frames = []  # 共有のフレームリスト
is_running = True  # スレッドの実行状態を示す変数

def get_lidar_frames_from_buffer(buffer):
    frames = []
    
    while len(buffer) >= 47:
        # ヘッダーが正しくない場合、次のヘッダーを探す
        if buffer[0] != 0x54:
            if 0x54 in buffer:
                buffer = buffer[buffer.index(0x54):]
            else:
                break
        
        # 一つのフレームを取り出す
        frame_data = buffer[:47]
        buffer = buffer[47:]
        
        # フレームのデータが正しいか確認
        if not check_lidar_frame_data(frame_data):
            continue
        
        frames.append(get_lidar_frame(frame_data))
    
    return frames, buffer

def check_lidar_frame_data(data):
    return data[1] == 0x2C and len(data) == 47

def calc_crc8(data):
    # CRC8の計算ロジックを実装（実際のロジックが不明のため、ダミー関数としています）
    return sum(data) % 256

def get_lidar_frame(data):
    frame = {}
    frame['header'] = data[0]
    frame['ver_len'] = data[1]
    frame['speed'] = struct.unpack("<H", bytes(data[2:4]))[0]
    frame['startAngle'] = struct.unpack("<H", bytes(data[4:6]))[0]
    
    points = []
    for i in range(12):
        start_index = 6 + 3 * i
        distance = struct.unpack("<H", bytes(data[start_index:start_index+2]))[0]
        intensity = data[start_index+2]
        points.append((distance, intensity))
    
    frame['points'] = points
    frame['endAngle'] = struct.unpack("<H", bytes(data[42:44]))[0]
    frame['timestamp'] = struct.unpack("<H", bytes(data[44:46]))[0]
    frame['crc8'] = data[46]
    
    return frame

def visualize_lidar_frames(frames):
    img = np.zeros((500, 500, 3), dtype=np.uint8)
    center = (img.shape[0] // 2, img.shape[1] // 2)
    
    for frame in frames:
        diffAngle = (frame['endAngle'] - frame['startAngle']) / 11.0 if frame['endAngle'] > frame['startAngle'] else (frame['endAngle'] + 36000.0 - frame['startAngle']) / 11.0
        for i, (distance, intensity) in enumerate(frame['points']):
            angle = (frame['startAngle'] + i * diffAngle) * (np.pi / 18000.0)
            angle = angle % (2 * np.pi)
            x = center[0] + distance * 0.1 * np.cos(angle)
            y = center[1] + distance * 0.1 * np.sin(angle)
            cv2.circle(img, (int(x), int(y)), 2, (0, intensity, 255-intensity), -1)
    
    cv2.imshow("Lidar Data", img)

def read_data_from_serial():
    # 利用可能なシリアルポートをリストアップ
    available_ports = list(serial.tools.list_ports.comports())

    if not available_ports:
        print("利用可能なシリアルポートが見つかりません。プログラムを終了します。")
    else:
        # 一番最初のシリアルポートを選択
        selected_port = available_ports[0].device
        print(f"選択したシリアルポート: {selected_port}")

        # シリアルポートに接続
        try:
            ser = serial.Serial(selected_port, 230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
            time.sleep(1)
            buffer = bytearray()
            global is_running
            while is_running:  # この条件を追加
                while ser.in_waiting:
                    buffer.extend(ser.read(ser.in_waiting))
                frames, buffer = get_lidar_frames_from_buffer(buffer)
                if frames:
                    all_frames.extend(frames)
        except serial.SerialException as e:
            print(f"シリアルポートの接続中にエラーが発生しました: {e}")
    
    
def visualize_lidar_data_continuously():
    global is_running
    while is_running:  # この条件を追加
        if all_frames:
            visualize_lidar_frames(all_frames)
            all_frames.clear()
        time.sleep(0.1)

        key = cv2.waitKey(1)
        
        if key == ord('q') or key == 27:  # qキーまたはESCキーが押されたら
            print("end!")
            is_running = False

def main():
    global is_running

    try:

        # データの読み取りを行うスレッドを開始
        data_thread = threading.Thread(target=read_data_from_serial)
        data_thread.start()

        # 可視化を行うスレッドを開始
        visualization_thread = threading.Thread(target=visualize_lidar_data_continuously)
        visualization_thread.start()

    except Exception as e:        
        print(f"Error occurred: {e}")

    data_thread.join()
    visualization_thread.join()
  
    cv2.destroyAllWindows()  # OpenCVのウィンドウをすべて閉じる
    print("All thread is ended.")

if __name__ == "__main__":
    main()
