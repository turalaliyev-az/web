"""
🤖 ROBOT KONTROL PANELİ - TAM ÇALIŞAN SİSTEM (OTOMATİK KAMERA)
"""

import cv2
import time
import numpy as np
from flask import Flask, render_template, Response, request, session, jsonify, redirect, url_for
from flask_socketio import SocketIO, emit
from datetime import datetime
import secrets
import threading
import queue
import math
import os
import json
from functools import wraps

# ==================== FLASK APP ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_control_secret_key_2024'
app.config['SESSION_COOKIE_HTTPONLY'] = True
app.config['SESSION_COOKIE_SAMESITE'] = 'Lax'

# ==================== SOCKETIO ====================
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
    ping_interval=5,
    ping_timeout=10,
    max_http_buffer_size=1e8,
    logger=False,
    engineio_logger=False
)

# ==================== AUTHENTICATION ====================
def login_required(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if not session.get('authenticated', False):
            return redirect(url_for('login'))
        return f(*args, **kwargs)
    return decorated_function

def generate_csrf_token():
    if 'csrf_token' not in session:
        session['csrf_token'] = secrets.token_hex(16)
    return session['csrf_token']

# ==================== KAMERA SİSTEMİ (OTOMATİK TARAMA) ====================
class CameraSystem:
    def __init__(self):
        self.camera = None
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False
        self.camera_thread = None
        self.target_fps = 25
        self.resolution = (640, 480)  # Daha yüksek çözünürlük
        self.last_frame_time = 0
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.camera_index = None  # Bulunan kamera indeksi
        self.camera_name = "Bulunamadı"
        
        self.initialize_camera()
    
    def find_camera(self):
        """0-9 arası tüm kamera indekslerini tarar"""
        print("Kamera aranıyor (0-9)...")
        available_cameras = []
        
        # 0'dan 9'a kadar tüm indeksleri tara
        for cam_index in range(10):
            try:
                cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
                if cap.isOpened():
                    # Test için birkaç frame oku
                    test_frames = 0
                    for _ in range(3):
                        ret, frame = cap.read()
                        if ret:
                            test_frames += 1
                    
                    if test_frames > 0:
                        # Kamera bilgilerini al
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        available_cameras.append({
                            'index': cam_index,
                            'width': width,
                            'height': height,
                            'frames_read': test_frames
                        })
                        print(f"  ✓ Kamera {cam_index}: {width}x{height} ({test_frames} frame okundu)")
                    cap.release()
                else:
                    print(f"  ✗ Kamera {cam_index}: Bağlantı yok")
            except Exception as e:
                print(f"  ✗ Kamera {cam_index} hatası: {e}")
        
        # Bulunan kameraları listele
        if available_cameras:
            print(f"\n✓ {len(available_cameras)} kamera bulundu:")
            for cam in available_cameras:
                print(f"  [{cam['index']}] {cam['width']}x{cam['height']}")
            
            # En yüksek çözünürlüklü kamerayı seç
            best_camera = max(available_cameras, key=lambda x: x['width'] * x['height'])
            return best_camera['index']
        
        print("✗ Hiçbir kamera bulunamadı")
        return None
    
    def initialize_camera(self):
        """Kamerayı otomatik bul ve başlat"""
        try:
            # Kamera bul
            self.camera_index = self.find_camera()
            
            if self.camera_index is not None:
                # Kamerayı başlat
                self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                
                if self.camera.isOpened():
                    # Maksimum desteklenen çözünürlüğü al
                    self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                    self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                    
                    # En iyi ayarları yap
                    self.camera.set(cv2.CAP_PROP_FPS, self.target_fps)
                    self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    
                    # 5 test frame'i oku
                    test_success = 0
                    for i in range(5):
                        ret, frame = self.camera.read()
                        if ret:
                            test_success += 1
                            # Gerçek çözünürlüğü al
                            if i == 0:
                                h, w = frame.shape[:2]
                                self.resolution = (w, h)
                                self.camera_name = f"Kamera {self.camera_index} ({w}x{h})"
                        
                        time.sleep(0.1)
                    
                    if test_success > 0:
                        print(f"✓ {self.camera_name} başarıyla başlatıldı")
                        self.start()
                        return True
                    else:
                        print(f"✗ Kamera {self.camera_index} test frame'leri okuyamadı")
                        self.camera.release()
                        self.camera = None
                else:
                    print(f"✗ Kamera {self.camera_index} açılamadı")
            else:
                print("⚠️ Kamera bulunamadı - Demo modu")
                return False
                
        except Exception as e:
            print(f"Kamera başlatma hatası: {e}")
            return False
        
        return False
    
    def start(self):
        """Kamera thread'ini başlat"""
        if self.running:
            return
        
        self.running = True
        self.camera_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.camera_thread.start()
        print("✓ Kamera thread başlatıldı")
    
    def capture_loop(self):
        """Frame yakalama döngüsü"""
        print(f"Kamera capture loop başladı: {self.camera_name}")
        
        while self.running:
            try:
                start_time = time.time()
                
                if self.camera and self.camera.isOpened():
                    ret, frame = self.camera.read()
                    
                    if ret:
                        # Boyutlandır (performans için)
                        if frame.shape[1] > 640:
                            frame = cv2.resize(frame, (640, 480))
                        
                        # FPS hesapla
                        self.frame_count += 1
                        current_time = time.time()
                        if current_time - self.last_fps_time >= 1.0:
                            self.fps = self.frame_count
                            self.frame_count = 0
                            self.last_fps_time = current_time
                        
                        # Queue'ya ekle
                        if self.frame_queue.full():
                            try:
                                self.frame_queue.get_nowait()
                            except queue.Empty:
                                pass
                        
                        self.frame_queue.put(frame)
                        self.last_frame_time = time.time()
                        
                    else:
                        # Kamera hatası
                        print(f"⚠️ Kamera {self.camera_index} frame okuyamadı")
                        demo_frame = self.create_demo_frame()
                        if not self.frame_queue.full():
                            self.frame_queue.put(demo_frame)
                else:
                    # Demo mod
                    demo_frame = self.create_demo_frame()
                    if not self.frame_queue.full():
                        self.frame_queue.put(demo_frame)
                
                # FPS kontrolü
                elapsed = time.time() - start_time
                target_delay = 1.0 / self.target_fps
                if elapsed < target_delay:
                    time.sleep(target_delay - elapsed)
                    
            except Exception as e:
                print(f"Kamera loop hatası: {e}")
                time.sleep(0.1)
    
    def create_demo_frame(self):
        """Demo frame oluştur"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Arkaplan gradient
        for i in range(480):
            color = int((i / 480) * 50)
            cv2.line(frame, (0, i), (640, i), (color, color, 50), 1)
        
        # Robot simgesi
        center_x, center_y = 320, 240
        cv2.circle(frame, (center_x, center_y), 60, (0, 200, 200), 3)
        cv2.arrowedLine(frame, (center_x, center_y), 
                       (center_x, center_y - 50), (0, 255, 0), 3)
        
        # Metinler
        cv2.putText(frame, "DEMO MODE", (220, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(frame, f"{self.fps} FPS", (540, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "Kamera bağlı değil", (200, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 1)
        
        # Sistem bilgisi
        current_time = datetime.now().strftime('%H:%M:%S')
        cv2.putText(frame, f"Time: {current_time}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        if self.camera_index is not None:
            cv2.putText(frame, f"Port: {self.camera_index}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        
        # Kontrol bilgileri
        cv2.putText(frame, "W/A/S/D - Hareket", (200, 350),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Space/Y - Dur", (200, 380),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Q/E/Z/C - Capraz", (200, 410),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def get_frame(self):
        """Frame al"""
        try:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get_nowait()
                self.frame_queue.task_done()
                
                # Timestamp ekle
                timestamp = datetime.now().strftime('%H:%M:%S')
                cv2.putText(frame, timestamp, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                # FPS ve kamera bilgisi
                cv2.putText(frame, f"{self.fps} FPS", (frame.shape[1] - 120, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                
                if self.camera_index is not None:
                    cv2.putText(frame, f"Cam: {self.camera_index}", (frame.shape[1] - 120, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
                
                return frame
        except queue.Empty:
            pass
        
        return self.create_demo_frame()
    
    def get_status(self):
        """Kamera durumu"""
        return {
            'connected': self.camera is not None and self.camera.isOpened(),
            'fps': self.fps,
            'resolution': self.resolution,
            'camera_index': self.camera_index,
            'camera_name': self.camera_name,
            'queue_size': self.frame_queue.qsize(),
            'running': self.running
        }
    
    def stop(self):
        """Kamerayı durdur"""
        self.running = False
        
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1)
        
        if self.camera and self.camera.isOpened():
            self.camera.release()
        
        print("✓ Kamera durduruldu")

# ==================== SERIAL SİSTEMİ ====================
class SerialSystem:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.port_name = None
        self.command_queue = queue.Queue(maxsize=100)
        self.response_queue = queue.Queue(maxsize=100)
        self.running = False
        self.worker_thread = None
        self.serial_module = None
        
        # Komut map
        self.command_map = {
            'forward': 'w',
            'backward': 's',
            'left': 'a',
            'right': 'd',
            'stop': 'y',
            'forward_left': 'q',
            'forward_right': 'e',
            'backward_left': 'z',
            'backward_right': 'c'
        }
        
        self.initialize_serial()
    
    def initialize_serial(self):
        """Serial bağlantısını başlat"""
        try:
            import serial
            import serial.tools.list_ports
            self.serial_module = serial
            self.serial_tools = serial.tools.list_ports
            print("✓ PySerial modülü yüklendi")
            
            # Otomatik bağlan
            threading.Thread(target=self.auto_connect, daemon=True).start()
            return True
            
        except ImportError:
            print("⚠️ PySerial modülü yüklenemedi - Demo modu")
            return False
    
    def auto_connect(self):
        """Otomatik bağlanma"""
        time.sleep(2)  # Sistemin başlamasını bekle
        
        if not self.serial_module:
            return
        
        ports = list(self.serial_tools.comports())
        print(f"Bulunan portlar: {[p.device for p in ports]}")
        
        # Öncelikli portlar
        preferred_ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', 'COM3', 'COM4']
        
        for port_name in preferred_ports:
            if self.connect(port_name):
                return
        
        # Diğer portları dene
        for port in ports:
            if self.connect(port.device):
                return
        
        print("✗ Serial port bulunamadı - Demo modu")
    
    def connect(self, port_name):
        """Belirtilen porta bağlan"""
        try:
            self.port_name = port_name
            self.serial_port = self.serial_module.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0.05,
                write_timeout=0.05
            )
            
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connected = True
            
            # Thread başlat
            self.start()
            
            print(f"✓ Serial bağlandı: {port_name}")
            
            socketio.emit('serial_connection', {
                'connected': True,
                'port': port_name,
                'message': f'Arduino bağlandı: {port_name}'
            })
            
            return True
            
        except Exception as e:
            print(f"✗ Serial bağlantı hatası ({port_name}): {e}")
            self.connected = False
            return False
    
    def start(self):
        """Worker thread başlat"""
        if self.running:
            return
        
        self.running = True
        self.worker_thread = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()
        print("✓ Serial worker başlatıldı")
    
    def worker_loop(self):
        """Komut işleme döngüsü"""
        while self.running:
            try:
                # Komut gönder
                if not self.command_queue.empty():
                    cmd_data = self.command_queue.get_nowait()
                    cmd_char = cmd_data.get('char', '?')
                    cmd_name = cmd_data.get('command', 'unknown')
                    
                    if self.connected and self.serial_port and self.serial_port.is_open:
                        try:
                            self.serial_port.write(cmd_char.encode('utf-8'))
                            self.serial_port.flush()
                            
                            socketio.emit('serial_status', {
                                'status': 'sent',
                                'command': cmd_name,
                                'char': cmd_char,
                                'message': f"'{cmd_char}' gönderildi"
                            })
                            
                        except Exception as e:
                            print(f"Komut gönderme hatası: {e}")
                            socketio.emit('serial_status', {
                                'status': 'error',
                                'message': f"Gönderme hatası: {e}"
                            })
                    
                    self.command_queue.task_done()
                
                # Serial'dan oku
                if self.connected and self.serial_port and self.serial_port.is_open:
                    try:
                        if self.serial_port.in_waiting > 0:
                            response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                            if response:
                                socketio.emit('serial_response', {
                                    'message': response,
                                    'source': 'arduino',
                                    'timestamp': time.time()
                                })
                    except Exception as e:
                        pass
                
                time.sleep(0.001)
                
            except queue.Empty:
                time.sleep(0.001)
            except Exception as e:
                print(f"Serial worker hatası: {e}")
                time.sleep(0.01)
    
    def send_command(self, command_name):
        """Komut gönder"""
        if command_name not in self.command_map:
            return False
        
        cmd_char = self.command_map[command_name]
        
        # Demo mod için
        if not self.connected or not self.serial_module:
            socketio.emit('serial_response', {
                'message': f"DEMO: {command_name.upper()} ('{cmd_char}') - Arduino bağlı değil",
                'source': 'demo',
                'timestamp': time.time()
            })
            return True
        
        # Kuyruğa ekle
        try:
            if not self.command_queue.full():
                self.command_queue.put({
                    'command': command_name,
                    'char': cmd_char,
                    'timestamp': time.time()
                })
                return True
            else:
                print("Komut kuyruğu dolu!")
                return False
                
        except Exception as e:
            print(f"Komut ekleme hatası: {e}")
            return False
    
    def get_status(self):
        """Durum bilgisi"""
        return {
            'connected': self.connected,
            'port': self.port_name,
            'queue_size': self.command_queue.qsize(),
            'running': self.running
        }
    
    def cleanup(self):
        """Temizlik"""
        self.running = False
        
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1)
        
        if self.serial_port and self.serial_port.is_open():
            try:
                self.serial_port.close()
            except:
                pass
        
        self.connected = False
        print("✓ Serial sistem temizlendi")

# ==================== MAVLINK SİSTEMİ ====================
class MAVLinkSystem:
    def __init__(self):
        self.device = "/dev/ttyUSB1"
        self.baud = 115200
        self.connected = False
        self.running = False
        self.mav_thread = None
        self.last_heartbeat = 0
        self.telemetry_cache = {
            'timestamp': 0,
            'gps': {'lat': 0, 'lon': 0, 'alt_m': 0, 'vel_m_s': 0, 'fix_type': 0, 'satellites_visible': 0},
            'attitude': {'roll_rad': 0, 'pitch_rad': 0, 'yaw_deg': 0, 'roll_deg': 0, 'pitch_deg': 0},
            'battery': {'voltage': 0, 'current': 0, 'remaining': 0},
            'system': {'load': 0, 'id': 0, 'status': 'DISCONNECTED'}
        }
        self.last_emit = 0
        
        # Başlatma thread'i
        threading.Thread(target=self.delayed_init, daemon=True).start()
    
    def delayed_init(self):
        """Gecikmeli başlatma"""
        time.sleep(3)
        self.initialize_mavlink()
    
    def initialize_mavlink(self):
        """MAVLink bağlantısını başlat"""
        try:
            from pymavlink import mavutil
            print("✓ pymavlink modülü yüklendi")
            
            # Thread başlat
            self.running = True
            self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)
            self.mav_thread.start()
            print("✓ MAVLink thread başlatıldı")
            
        except ImportError:
            print("⚠️ pymavlink kurulu değil - Telemetri simülasyonu")
            self.start_simulation()
            return False
    
    def start_simulation(self):
        """Telemetri simülasyonu başlat"""
        self.running = True
        self.mav_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.mav_thread.start()
        print("✓ Telemetri simülasyonu başlatıldı")
    
    def simulation_loop(self):
        """Simülasyon döngüsü"""
        import random
        
        while self.running:
            try:
                current_time = time.time()
                
                # GPS simülasyonu
                base_lat = 41.0082 + (random.random() - 0.5) * 0.001
                base_lon = 28.9784 + (random.random() - 0.5) * 0.001
                
                self.telemetry_cache.update({
                    'timestamp': current_time,
                    'gps': {
                        'lat': base_lat,
                        'lon': base_lon,
                        'alt_m': 50 + random.random() * 20,
                        'vel_m_s': 2 + random.random() * 3,
                        'fix_type': 3,
                        'satellites_visible': 8 + int(random.random() * 4)
                    },
                    'attitude': {
                        'roll_rad': (random.random() - 0.5) * 0.2,
                        'pitch_rad': (random.random() - 0.5) * 0.1,
                        'yaw_deg': random.random() * 360,
                        'roll_deg': (random.random() - 0.5) * 20,
                        'pitch_deg': (random.random() - 0.5) * 10
                    },
                    'battery': {
                        'voltage': 12.4 + random.random() * 0.4,
                        'current': 3 + random.random() * 2,
                        'remaining': 75 + random.random() * 20
                    },
                    'system': {
                        'load': 45 + random.random() * 10,
                        'id': 1,
                        'status': 'SIMULATION'
                    }
                })
                
                # Her 0.2 saniyede bir gönder (5Hz)
                if current_time - self.last_emit >= 0.2:
                    socketio.emit('mavlink_telemetry', self.telemetry_cache)
                    self.last_emit = current_time
                
                # Durum güncellemesi
                if not self.connected:
                    self.connected = True
                    socketio.emit('mavlink_status', {
                        'connected': True,
                        'message': 'Telemetri simülasyonu çalışıyor',
                        'device': 'SIMULATION',
                        'baud': 115200
                    })
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Simülasyon hatası: {e}")
                time.sleep(0.1)
    
    def mavlink_loop(self):
        """Gerçek MAVLink döngüsü"""
        try:
            from pymavlink import mavutil
            
            print(f"MAVLink bağlanıyor: {self.device}")
            
            if not os.path.exists(self.device):
                print(f"✗ Port bulunamadı: {self.device}")
                socketio.emit('mavlink_status', {
                    'connected': False,
                    'message': f'Port bulunamadı: {self.device}',
                    'device': self.device
                })
                self.start_simulation()
                return
            
            # Bağlan
            connection = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                autoreconnect=True,
                retries=5
            )
            
            # Heartbeat bekle
            start_time = time.time()
            while time.time() - start_time < 5:
                msg = connection.recv_match(type='HEARTBEAT', blocking=False, timeout=0.5)
                if msg:
                    self.connected = True
                    self.last_heartbeat = time.time()
                    
                    print(f"✓ MAVLink bağlandı! System ID: {msg.get_srcSystem()}")
                    
                    socketio.emit('mavlink_status', {
                        'connected': True,
                        'message': 'Pixhawk bağlandı',
                        'device': self.device,
                        'baud': self.baud,
                        'system': msg.get_srcSystem()
                    })
                    
                    break
            
            if not self.connected:
                print("✗ Heartbeat alınamadı - Simülasyon başlatılıyor")
                socketio.emit('mavlink_status', {
                    'connected': False,
                    'message': 'Heartbeat alınamadı - Simülasyon modu',
                    'device': self.device
                })
                self.start_simulation()
                return
            
            # Ana döngü
            while self.running:
                try:
                    current_time = time.time()
                    
                    # Heartbeat kontrolü
                    if current_time - self.last_heartbeat > 5:
                        print("⚠️ Heartbeat timeout!")
                        self.connected = False
                        socketio.emit('mavlink_status', {
                            'connected': False,
                            'message': 'Heartbeat timeout',
                            'device': self.device
                        })
                        break
                    
                    # Mesaj oku
                    msg = connection.recv_match(blocking=False, timeout=0.01)
                    
                    if msg:
                        self.process_message(msg)
                        
                        if msg.get_type() == 'HEARTBEAT':
                            self.last_heartbeat = current_time
                    
                    # Telemetri gönder
                    if current_time - self.last_emit >= 0.2:
                        socketio.emit('mavlink_telemetry', self.telemetry_cache)
                        self.last_emit = current_time
                    
                    time.sleep(0.005)
                    
                except Exception as e:
                    print(f"MAVLink loop hatası: {e}")
                    time.sleep(0.1)
            
            connection.close()
            
        except Exception as e:
            print(f"MAVLink bağlantı hatası: {e}")
            self.start_simulation()
    
    def process_message(self, msg):
        """MAVLink mesajını işle"""
        msg_type = msg.get_type()
        
        try:
            self.telemetry_cache['timestamp'] = time.time()
            
            if msg_type == 'GPS_RAW_INT':
                self.telemetry_cache['gps'].update({
                    'lat': msg.lat / 1e7 if msg.lat != 0 else 0,
                    'lon': msg.lon / 1e7 if msg.lon != 0 else 0,
                    'alt_m': msg.alt / 1000.0 if msg.alt != 0 else 0,
                    'vel_m_s': msg.vel / 100.0 if msg.vel != 0 else 0,
                    'fix_type': msg.fix_type,
                    'satellites_visible': msg.satellites_visible
                })
            
            elif msg_type == 'ATTITUDE':
                self.telemetry_cache['attitude'].update({
                    'roll_rad': msg.roll,
                    'pitch_rad': msg.pitch,
                    'yaw_deg': math.degrees(msg.yaw) % 360,
                    'roll_deg': math.degrees(msg.roll),
                    'pitch_deg': math.degrees(msg.pitch)
                })
            
            elif msg_type == 'SYS_STATUS':
                self.telemetry_cache['battery'].update({
                    'voltage': msg.voltage_battery / 1000.0 if msg.voltage_battery != 0 else 0,
                    'current': msg.current_battery / 100.0 if msg.current_battery != 0 else 0,
                    'remaining': msg.battery_remaining
                })
                self.telemetry_cache['system']['load'] = msg.load / 10.0
            
            elif msg_type == 'HEARTBEAT':
                self.telemetry_cache['system']['id'] = msg.get_srcSystem()
            
        except Exception as e:
            print(f"MAVLink mesaj işleme hatası ({msg_type}): {e}")
    
    def get_status(self):
        """Durum bilgisi"""
        return {
            'connected': self.connected,
            'device': self.device,
            'baud': self.baud,
            'status': self.telemetry_cache['system']['status']
        }
    
    def get_telemetry(self):
        """Telemetri cache'ini al"""
        return self.telemetry_cache.copy()
    
    def cleanup(self):
        """Temizlik"""
        self.running = False
        
        if self.mav_thread and self.mav_thread.is_alive():
            self.mav_thread.join(timeout=1)
        
        print("✓ MAVLink sistem temizlendi")

# ==================== SİSTEMLERİ BAŞLAT ====================
print("\n" + "="*60)
print("   🤖 ROBOT KONTROL PANELİ - TAM SİSTEM")
print("   🔍 0-9 KAMERA OTOMATİK TARAMA")
print("="*60)

camera_system = CameraSystem()
serial_system = SerialSystem()
mavlink_system = MAVLinkSystem()

cam_status = camera_system.get_status()
print(f"   Kamera: {cam_status['camera_name']}")
print(f"   Arduino: {'Bağlı' if serial_system.connected else 'Demo'}")
print(f"   Pixhawk: Telemetri aktif")
print("="*60)

# ==================== VIDEO STREAM ====================
def generate_frames():
    """Video frame'leri oluştur"""
    print("Video akışı başlatılıyor...")
    
    while True:
        try:
            frame = camera_system.get_frame()
            
            ret, buffer = cv2.imencode('.jpg', frame, [
                int(cv2.IMWRITE_JPEG_QUALITY), 
                70  # %70 kalite (performans için)
            ])
            
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + 
                       buffer.tobytes() + b'\r\n')
            
            # FPS kontrolü
            time.sleep(1.0 / camera_system.target_fps)
            
        except Exception as e:
            print(f"Frame generation error: {e}")
            time.sleep(0.1)

# ==================== ROUTES ====================
@app.route('/')
@login_required
def index():
    """Ana sayfa"""
    return render_template('index.html', csrf_token=generate_csrf_token())

@app.route('/login', methods=['GET', 'POST'])
def login():
    """Giriş sayfası"""
    if request.method == 'POST':
        session['authenticated'] = True
        session['username'] = 'operator'
        print("Kullanıcı giriş yaptı")
        return redirect(url_for('index'))
    
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Giriş</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                height: 100vh;
                display: flex;
                justify-content: center;
                align-items: center;
                margin: 0;
            }
            .login-container {
                background: white;
                padding: 40px;
                border-radius: 10px;
                box-shadow: 0 10px 25px rgba(0,0,0,0.2);
                width: 320px;
                text-align: center;
            }
            .login-title {
                color: #333;
                margin-bottom: 30px;
                font-size: 24px;
            }
            .login-input {
                width: 100%;
                padding: 12px;
                margin: 10px 0;
                border: 2px solid #ddd;
                border-radius: 5px;
                font-size: 16px;
                transition: border-color 0.3s;
            }
            .login-input:focus {
                border-color: #667eea;
                outline: none;
            }
            .login-button {
                width: 100%;
                padding: 12px;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                cursor: pointer;
                margin-top: 20px;
                transition: transform 0.2s;
            }
            .login-button:hover {
                transform: translateY(-2px);
            }
            .demo-note {
                font-size: 12px;
                color: #666;
                margin-top: 20px;
                font-style: italic;
            }
        </style>
    </head>
    <body>
        <div class="login-container">
            <div class="login-title">🤖 Robot Kontrol Paneli</div>
            <form method="post">
                <input type="text" class="login-input" name="username" placeholder="Kullanıcı adı" value="operator" required>
                <input type="password" class="login-input" name="password" placeholder="Şifre" value="1234" required>
                <button type="submit" class="login-button">Giriş Yap</button>
            </form>
            <div class="demo-note">Kullanıcı: operator | Şifre: 1234</div>
        </div>
    </body>
    </html>
    '''

@app.route('/logout')
def logout():
    """Çıkış"""
    session.pop('authenticated', None)
    session.pop('username', None)
    return redirect(url_for('login'))

@app.route('/video_feed')
@login_required
def video_feed():
    """Video akışı"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0'
        }
    )

# ==================== API ROUTES ====================
@app.route('/api/status')
@login_required
def api_status():
    """Sistem durumu"""
    return jsonify({
        'camera': camera_system.get_status(),
        'serial': serial_system.get_status(),
        'mavlink': mavlink_system.get_status(),
        'server_time': datetime.now().isoformat(),
        'status': 'running'
    })

@app.route('/api/health')
def health():
    """Sağlık kontrolü"""
    return jsonify({
        'status': 'healthy',
        'timestamp': datetime.now().isoformat()
    })

@app.route('/serial/status')
@login_required
def serial_status():
    """Serial durumu"""
    return jsonify(serial_system.get_status())

@app.route('/serial/connect/<port_name>')
@login_required
def serial_connect(port_name):
    """Serial bağlan"""
    if port_name == 'auto':
        serial_system.auto_connect()
        return jsonify({'success': True, 'message': 'Otomatik bağlanma başlatıldı'})
    
    if serial_system.connect(port_name):
        return jsonify({'success': True, 'message': f'{port_name} bağlandı'})
    else:
        return jsonify({'success': False, 'message': f'{port_name} bağlanamadı'})

@app.route('/serial/disconnect')
@login_required
def serial_disconnect():
    """Serial bağlantıyı kes"""
    serial_system.cleanup()
    return jsonify({'success': True, 'message': 'Serial bağlantısı kesildi'})

@app.route('/mavlink/status')
@login_required
def mavlink_status():
    """MAVLink durumu"""
    return jsonify(mavlink_system.get_status())

@app.route('/mavlink/connect')
@login_required
def mavlink_connect():
    """MAVLink bağlan"""
    return jsonify({
        'success': True,
        'message': 'MAVLink bağlantısı başlatıldı',
        'device': mavlink_system.device,
        'baud': mavlink_system.baud
    })

@app.route('/mavlink/disconnect')
@login_required
def mavlink_disconnect():
    """MAVLink bağlantıyı kes"""
    mavlink_system.cleanup()
    return jsonify({'success': True, 'message': 'MAVLink bağlantısı kesildi'})

# ==================== WEBSOCKET HANDLERS ====================
@socketio.on('connect')
def handle_connect():
    """Client bağlandığında"""
    print(f"Client bağlandı: {request.sid}")
    
    emit('connection_status', {
        'status': 'connected',
        'message': 'Bağlantı kuruldu!'
    })
    
    # Başlangıç durumlarını gönder
    emit('serial_connection', {
        'connected': serial_system.connected,
        'port': serial_system.port_name,
        'message': 'Arduino bağlantısı hazır' if serial_system.connected else 'Arduino bağlantısı yok'
    })
    
    mav_status = mavlink_system.get_status()
    emit('mavlink_status', {
        'connected': mav_status['connected'],
        'message': 'Pixhawk telemetri aktif' if mav_status['connected'] else 'Pixhawk bağlı değil',
        'device': mav_status['device'],
        'baud': mav_status['baud']
    })
    
    # Kamera bilgisini gönder
    cam_status = camera_system.get_status()
    emit('camera_status', {
        'connected': cam_status['connected'],
        'camera_index': cam_status['camera_index'],
        'camera_name': cam_status['camera_name'],
        'resolution': cam_status['resolution'],
        'fps': cam_status['fps']
    })

@socketio.on('disconnect')
def handle_disconnect():
    """Client bağlantısı kesildiğinde"""
    print(f"Client bağlantısı kesildi: {request.sid}")

@socketio.on('robot_command')
def handle_robot_command(data):
    """Robot komutu"""
    if not session.get('authenticated'):
        return
    
    cmd = data.get('command', 'unknown')
    csrf_token = data.get('csrf_token', '')
    
    if csrf_token != session.get('csrf_token'):
        emit('error', {'message': 'Güvenlik hatası!'})
        return
    
    valid_commands = ['forward', 'backward', 'left', 'right', 'stop', 
                     'forward_left', 'forward_right', 'backward_left', 'backward_right']
    
    if cmd not in valid_commands:
        emit('error', {'message': 'Geçersiz komut!'})
        return
    
    timestamp = datetime.now().strftime('%H:%M:%S')
    
    emit('status_update', {
        'message': cmd.upper(),
        'timestamp': timestamp,
        'char': serial_system.command_map.get(cmd, '?')
    }, broadcast=True)
    
    success = serial_system.send_command(cmd)
    
    if not success:
        emit('error', {'message': 'Komut gönderilemedi!'})

# ==================== TEMİZLİK ====================
import atexit

def cleanup():
    """Sistem temizliği"""
    print("\n🤖 Sistem durduruluyor...")
    
    camera_system.stop()
    serial_system.cleanup()
    mavlink_system.cleanup()
    
    print("✓ Tüm kaynaklar temizlendi")

atexit.register(cleanup)

# ==================== MAIN ====================
if __name__ == '__main__':
    print("\n📡 Sunucu başlatılıyor: http://localhost:5000")
    print("📡 Veya: http://<ip-adresiniz>:5000")
    print("\n🔧 Kontroller:")
    print("   - W/A/S/D veya ok tuşları = Robot hareketi")
    print("   - Space veya Y = Dur")
    print("   - Q/E/Z/C = Çapraz hareketler")
    print("="*60)
    
    try:
        socketio.run(
            app,
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=True
        )
    except KeyboardInterrupt:
        print("\n\n🛑 Sunucu durduruluyor...")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
    finally:
        cleanup()