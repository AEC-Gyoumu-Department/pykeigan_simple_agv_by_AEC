import cv2
import struct
import numpy as np
import serial
from serial.tools import list_ports
import threading
import time

all_frames = []  # Shared frame list
is_running = True  # Variable indicating the running state of the threads

def get_lidar_frames_from_buffer(buffer):
    frames = []
    
    while len(buffer) >= 47:
        # If the header is not correct, search for the next header
        if buffer[0] != 0x54:
            if 0x54 in buffer:
                buffer = buffer[buffer.index(0x54):]
            else:
                break
        
        # Extract one frame
        frame_data = buffer[:47]
        buffer = buffer[47:]
        
        # Verify the frame data is correct
        if not check_lidar_frame_data(frame_data):
            continue
        
        frames.append(get_lidar_frame(frame_data))
    
    return frames, buffer

def check_lidar_frame_data(data):
    return data[1] == 0x2C and len(data) == 47

def calc_crc8(data):
    # Implement the CRC8 calculation logic (dummy function as actual logic is unknown)
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
    # List available serial ports
    available_ports = list(serial.tools.list_ports.comports())

    if not available_ports:
        print("No available serial ports found. Exiting the program.")
    else:
        # Select the first serial port
        selected_port = available_ports[0].device
        print(f"Selected serial port: {selected_port}")

        # Connect to the serial port
        try:
            ser = serial.Serial(selected_port, 230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
            time.sleep(1)
            buffer = bytearray()
            global is_running
            while is_running:
                while ser.in_waiting:
                    buffer.extend(ser.read(ser.in_waiting))
                frames, buffer = get_lidar_frames_from_buffer(buffer)
                if frames:
                    all_frames.extend(frames)
        except serial.SerialException as e:
            print(f"Error while connecting to the serial port: {e}")
        finally:
            if ser.is_open:
                ser.close()
                print(f"Closed serial port {selected_port}")

def visualize_lidar_data_continuously():
    global is_running
    while is_running:
        if all_frames:
            visualize_lidar_frames(all_frames)
            all_frames.clear()
        time.sleep(0.1)

        key = cv2.waitKey(1)
        
        if key == ord('q') or key == 27:  # If 'q' key or ESC key is pressed
            print("end!")
            is_running = False

def main():
    global is_running

    try:
        # Start the thread for reading data
        data_thread = threading.Thread(target=read_data_from_serial)
        data_thread.start()

        # Start the thread for visualizing data
        visualization_thread = threading.Thread(target=visualize_lidar_data_continuously)
        visualization_thread.start()

    except Exception as e:
        print(f"Error occurred: {e}")

    data_thread.join()
    visualization_thread.join()
  
    cv2.destroyAllWindows()
    print("All threads have ended.")

if __name__ == "__main__":
    main()
