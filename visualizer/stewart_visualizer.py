#!/usr/bin/env python3

import sys
import time
import threading
import queue
import struct
import math
from collections import deque
from typing import Optional, Tuple, List

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

PACKET_START_MARKER = 0xAA
PACKET_END_MARKER = 0x55
PACKET_TELEMETRY = 0x02
PACKET_COMMAND = 0x01

class StewartGeometry:
    
    def __init__(self):
        self.base_radius = 150.0
        self.base_angle_offset = 0.0
        
        self.top_radius = 100.0
        self.top_angle_offset = math.pi / 6.0
        
        self.min_leg_length = 80.0
        self.max_leg_length = 200.0
        self.actuator_offset = 20.0
        
        self.base_points = self._calculate_attachment_points(
            self.base_radius, self.base_angle_offset
        )
        self.top_points = self._calculate_attachment_points(
            self.top_radius, self.top_angle_offset
        )
    
    def _calculate_attachment_points(self, radius: float, angle_offset: float) -> np.ndarray:
        points = []
        for i in range(6):
            angle = angle_offset + i * math.pi / 3.0
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 0.0
            points.append([x, y, z])
        return np.array(points)

class SerialProtocolParser:
    
    def __init__(self):
        self.rx_buffer = bytearray()
        self.rx_index = 0
        self.packet_complete = False
        self.sequence_number = 0
        
        self.packets_received = 0
        self.crc_errors = 0
        self.framing_errors = 0
    
    def compute_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def process_byte(self, byte: int) -> Optional[dict]:
        if self.packet_complete:
            return None
        
        if self.rx_index == 0 and byte != PACKET_START_MARKER:
            self.framing_errors += 1
            return None
        
        self.rx_buffer.append(byte)
        self.rx_index += 1
        
        if self.rx_index >= 10:
            packet_type = self.rx_buffer[1]
            payload_length = (self.rx_buffer[3] << 8) | self.rx_buffer[4]
            total_length = 9 + payload_length + 1
            
            if self.rx_index >= total_length:
                if self.rx_buffer[total_length - 1] != PACKET_END_MARKER:
                    self.framing_errors += 1
                    self._reset()
                    return None
                
                payload = bytes(self.rx_buffer[9:9+payload_length])
                computed_crc = self.compute_crc(payload)
                received_crc = (self.rx_buffer[7] << 8) | self.rx_buffer[8]
                
                if computed_crc != received_crc:
                    self.crc_errors += 1
                    self._reset()
                    return None
                
                packet = self._parse_packet(packet_type, payload)
                self.packets_received += 1
                self._reset()
                return packet
        
        return None
    
    def _parse_packet(self, packet_type: int, payload: bytes) -> Optional[dict]:
        if packet_type == PACKET_TELEMETRY:
            return self._parse_telemetry(payload)
        elif packet_type == PACKET_COMMAND:
            return self._parse_command(payload)
        else:
            return None
    
    def _parse_telemetry(self, payload: bytes) -> dict:
        if len(payload) < 128:
            return None
        
        def parse_double(offset):
            bytes_data = payload[offset:offset+8]
            return struct.unpack('>d', bytes_data)[0]
        
        telemetry = {
            'current_x': parse_double(0),
            'current_y': parse_double(8),
            'current_z': parse_double(16),
            'current_phi': parse_double(24),
            'current_theta': parse_double(32),
            'current_psi': parse_double(40),
            'actuator_lengths': [],
            'actuator_velocities': [],
            'control_loop_period_us': parse_double(96),
            'ik_computation_time_us': parse_double(104),
            'loop_counter': struct.unpack('>I', payload[112:116])[0],
            'error_flags': payload[116],
            'system_status': payload[117],
            'actuator_errors': payload[118],
            'timestamp': struct.unpack('>I', payload[120:124])[0]
        }
        
        for i in range(6):
            telemetry['actuator_lengths'].append(parse_double(48 + i*8))
            telemetry['actuator_velocities'].append(parse_double(96 + i*8))
        
        return telemetry
    
    def _parse_command(self, payload: bytes) -> dict:
        if len(payload) < 78:
            return None
        
        def parse_double(offset):
            bytes_data = payload[offset:offset+8]
            return struct.unpack('>d', bytes_data)[0]
        
        command = {
            'command_type': payload[0],
            'target_x': parse_double(1),
            'target_y': parse_double(9),
            'target_z': parse_double(17),
            'target_phi': parse_double(25),
            'target_theta': parse_double(33),
            'target_psi': parse_double(41),
            'velocity': parse_double(49),
            'acceleration': parse_double(57),
            'duration': parse_double(65),
            'timestamp': struct.unpack('>I', payload[73:77])[0],
            'flags': payload[77]
        }
        
        return command
    
    def _reset(self):
        self.rx_buffer = bytearray()
        self.rx_index = 0
        self.packet_complete = False

class SerialHandler:
    
    def __init__(self, port: str = None, baudrate: int = 12000000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.parser = SerialProtocolParser()
        self.telemetry_queue = queue.Queue(maxsize=100)
        self.running = False
        self.thread = None
    
    def connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            return False
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.running = True
            self.thread = threading.Thread(target=self._read_thread, daemon=True)
            self.thread.start()
            return True
        except Exception:
            return False
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.serial_conn:
            self.serial_conn.close()
    
    def _read_thread(self):
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    for byte in data:
                        packet = self.parser.process_byte(byte)
                        if packet and 'current_x' in packet:
                            try:
                                self.telemetry_queue.put_nowait(packet)
                            except queue.Full:
                                pass
            except Exception:
                break
            time.sleep(0.001)
    
    def get_telemetry(self) -> Optional[dict]:
        try:
            return self.telemetry_queue.get_nowait()
        except queue.Empty:
            return None
    
    def send_command(self, command: dict) -> bool:
        if not self.serial_conn:
            return False
        
        try:
            packet = bytearray()
            packet.append(PACKET_START_MARKER)
            packet.append(PACKET_COMMAND)
            packet.append(1)
            
            payload_length = 78
            packet.append((payload_length >> 8) & 0xFF)
            packet.append(payload_length & 0xFF)
            
            packet.append(0)
            packet.append(0)
            
            packet.append(0)
            packet.append(0)
            
            payload = bytearray()
            payload.append(command.get('command_type', 0))
            
            def add_double(value):
                payload.extend(struct.pack('>d', value))
            
            add_double(command.get('target_x', 0.0))
            add_double(command.get('target_y', 0.0))
            add_double(command.get('target_z', 0.0))
            add_double(command.get('target_phi', 0.0))
            add_double(command.get('target_theta', 0.0))
            add_double(command.get('target_psi', 0.0))
            add_double(command.get('velocity', 0.0))
            add_double(command.get('acceleration', 0.0))
            add_double(command.get('duration', 1.0))
            payload.extend(struct.pack('>I', command.get('timestamp', int(time.time() * 1000))))
            payload.append(command.get('flags', 0))
            
            while len(payload) < payload_length:
                payload.append(0)
            
            crc = self.parser.compute_crc(bytes(payload))
            packet[7] = (crc >> 8) & 0xFF
            packet[8] = crc & 0xFF
            
            packet.extend(payload)
            packet.append(PACKET_END_MARKER)
            
            self.serial_conn.write(packet)
            return True
        except Exception:
            return False

class StewartVisualizer:
    
    def __init__(self, serial_handler: SerialHandler = None):
        self.serial_handler = serial_handler
        self.geometry = StewartGeometry()
        
        self.current_pose = {
            'x': 0.0, 'y': 0.0, 'z': 150.0,
            'phi': 0.0, 'theta': 0.0, 'psi': 0.0
        }
        
        self.actuator_lengths = [150.0] * 6
        
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.trajectory_history = deque(maxlen=100)
        
        self.animation = FuncAnimation(
            self.fig, self.update, interval=50, blit=False
        )
        
        self._setup_plot()
        
        self.status_text = self.fig.text(0.02, 0.95, "", fontsize=10, 
                                         verticalalignment='top')
        
        self._setup_controls()
    
    def _setup_plot(self):
        self.ax.set_xlabel('X [mm]')
        self.ax.set_ylabel('Y [mm]')
        self.ax.set_zlabel('Z [mm]')
        self.ax.set_title('Stewart Platform 3D Visualizer')
        
        self.ax.set_box_aspect([1, 1, 1])
        
        limit = 200
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, 300)
        
        self.ax.grid(True)
    
    def _setup_controls(self):
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
    
    def _on_key_press(self, event):
        if not self.serial_handler:
            return
        
        key = event.key
        command = None
        
        if key == 'r':
            command = {
                'command_type': 2,
                'timestamp': int(time.time() * 1000)
            }
        elif key == 'h':
            command = {
                'command_type': 0,
                'target_x': 20.0, 'target_y': 0.0, 'target_z': 150.0,
                'target_phi': 0.0, 'target_theta': 0.0, 'target_psi': 0.0,
                'duration': 1.0, 'timestamp': int(time.time() * 1000)
            }
        elif key == 'f':
            command = {
                'command_type': 0,
                'target_x': -20.0, 'target_y': 0.0, 'target_z': 150.0,
                'target_phi': 0.0, 'target_theta': 0.0, 'target_psi': 0.0,
                'duration': 1.0, 'timestamp': int(time.time() * 1000)
            }
        elif key == 't':
            command = {
                'command_type': 0,
                'target_x': 0.0, 'target_y': 20.0, 'target_z': 150.0,
                'target_phi': 0.0, 'target_theta': 0.0, 'target_psi': 0.0,
                'duration': 1.0, 'timestamp': int(time.time() * 1000)
            }
        elif key == 'g':
            command = {
                'command_type': 0,
                'target_x': 0.0, 'target_y': -20.0, 'target_z': 150.0,
                'target_phi': 0.0, 'target_theta': 0.0, 'target_psi': 0.0,
                'duration': 1.0, 'timestamp': int(time.time() * 1000)
            }
        elif key == 'space':
            command = {
                'command_type': 3,
                'timestamp': int(time.time() * 1000)
            }
        
        if command:
            self.serial_handler.send_command(command)
    
    def compute_rotation_matrix(self, phi: float, theta: float, psi: float) -> np.ndarray:
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        
        R = np.array([
            [cos_theta * cos_psi, sin_phi * sin_theta * cos_psi - cos_phi * sin_psi, 
             cos_phi * sin_theta * cos_psi + sin_phi * sin_psi],
            [cos_theta * sin_psi, sin_phi * sin_theta * sin_psi + cos_phi * cos_psi,
             cos_phi * sin_theta * sin_psi - sin_phi * cos_psi],
            [-sin_theta, sin_phi * cos_theta, cos_phi * cos_theta]
        ])
        
        return R
    
    def transform_points(self, points: np.ndarray, pose: dict) -> np.ndarray:
        R = self.compute_rotation_matrix(
            pose['phi'], pose['theta'], pose['psi']
        )
        
        T = np.array([pose['x'], pose['y'], pose['z']])
        
        transformed = (R @ points.T).T + T
        
        return transformed
    
    def update(self, frame):
        if self.serial_handler:
            telemetry = self.serial_handler.get_telemetry()
            if telemetry:
                self.current_pose = {
                    'x': telemetry['current_x'],
                    'y': telemetry['current_y'],
                    'z': telemetry['current_z'],
                    'phi': telemetry['current_phi'],
                    'theta': telemetry['current_theta'],
                    'psi': telemetry['current_psi']
                }
                self.actuator_lengths = telemetry['actuator_lengths']
                
                status = f"Position: ({self.current_pose['x']:.1f}, {self.current_pose['y']:.1f}, {self.current_pose['z']:.1f}) mm\n"
                status += f"Orientation: ({math.degrees(self.current_pose['phi']):.1f}, {math.degrees(self.current_pose['theta']):.1f}, {math.degrees(self.current_pose['psi']):.1f})°\n"
                status += f"Loop Period: {telemetry['control_loop_period_us']:.1f} μs\n"
                status += f"IK Time: {telemetry['ik_computation_time_us']:.1f} μs\n"
                status += f"System Status: {telemetry['system_status']}\n"
                status += f"Errors: {telemetry['error_flags']}"
                self.status_text.set_text(status)
        
        self.ax.clear()
        self._setup_plot()
        
        top_points_global = self.transform_points(self.geometry.top_points, self.current_pose)
        
        base_x = np.append(self.geometry.base_points[:, 0], self.geometry.base_points[0, 0])
        base_y = np.append(self.geometry.base_points[:, 1], self.geometry.base_points[0, 1])
        base_z = np.append(self.geometry.base_points[:, 2], self.geometry.base_points[0, 2])
        self.ax.plot(base_x, base_y, base_z, 'b-', linewidth=2, label='Base Platform')
        
        top_x = np.append(top_points_global[:, 0], top_points_global[0, 0])
        top_y = np.append(top_points_global[:, 1], top_points_global[0, 1])
        top_z = np.append(top_points_global[:, 2], top_points_global[0, 2])
        self.ax.plot(top_x, top_y, top_z, 'r-', linewidth=2, label='Top Platform')
        
        for i in range(6):
            base_point = self.geometry.base_points[i]
            top_point = top_points_global[i]
            
            length = self.actuator_lengths[i]
            mid_length = (self.geometry.max_leg_length + self.geometry.min_leg_length) / 2
            if abs(length - mid_length) < 20:
                color = 'g'
            else:
                color = 'orange'
            
            self.ax.plot(
                [base_point[0], top_point[0]],
                [base_point[1], top_point[1]],
                [base_point[2], top_point[2]],
                color=color, linewidth=1.5, alpha=0.7
            )
        
        if len(self.trajectory_history) > 1:
            traj_array = np.array(self.trajectory_history)
            self.ax.plot(
                traj_array[:, 0], traj_array[:, 1], traj_array[:, 2],
                'm--', linewidth=1, alpha=0.5, label='Trajectory'
            )
        
        self.trajectory_history.append([
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['z']
        ])
        
        self.ax.legend(loc='upper right')
        
        self.ax.view_init(elev=20, azim=45)
    
    def show(self):
        plt.show()

def main():
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        if SERIAL_AVAILABLE:
            available_ports = serial.tools.list_ports.comports()
            if available_ports:
                port = available_ports[0].device
    
    serial_handler = None
    if port:
        serial_handler = SerialHandler(port=port, baudrate=12000000)
        if not serial_handler.connect():
            serial_handler = None
    
    visualizer = StewartVisualizer(serial_handler)
    
    try:
        visualizer.show()
    except KeyboardInterrupt:
        pass
    finally:
        if serial_handler:
            serial_handler.disconnect()

if __name__ == "__main__":
    main()
