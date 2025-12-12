"""
Robot Control Panel - Complete System with Serial Control & MAVLink Telemetry
"""

import cv2
import time
import numpy as np
from flask import Flask, render_template, Response, request, session, redirect, url_for
from flask_socketio import SocketIO, emit
from datetime import datetime
from functools import wraps
import secrets
import threading
import queue
import math
import os

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_control_complete_system_2024'
app.config['SESSION_COOKIE_HTTPONLY'] = True
app.config['SESSION_COOKIE_SAMESITE'] = 'Lax'

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
    ping_interval=5,
    ping_timeout=10,
    logger=False,
    engineio_logger=False
)

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

# ==================== SERIAL PORT YÖNETİCİSİ (Arduino) ====================
class SerialManager:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.running = False
        self.thread = None
        self.serial_module = None
        self.port_name = None
        
        try:
            import serial
            import serial.tools.list_ports
            self.serial_module = serial
            self.serial_tools = serial.tools.list_ports
            print("✓ PySerial modülü yüklendi")
            self.detect_and_connect()
        except ImportError:
            print("⚠️ PySerial modülü yüklenemedi. Serial desteği devre dışı.")
            self.serial_module = None
    
    def detect_and_connect(self):
        """Mevcut serial portları tespit et ve bağlan"""
        if not self.serial_module:
            return False
        
        try:
            ports = list(self.serial_tools.comports())
            print(f"Bulunan serial portlar: {[p.device for p in ports]}")
            
            # /dev/ttyUSB0 için öncelikli kontrol
            for port in ports:
                port_name = port.device
                if 'ttyUSB0' in port_name:
                    print(f"Arduino portu bulundu: {port_name}")
                    if self.connect(port_name):
                        return True
            
            # Diğer USB portlarını dene
            for port in ports:
                port_name = port.device
                if 'ttyUSB' in port_name or 'ttyACM' in port_name:
                    print(f"Potansiyel Arduino portu: {port_name}")
                    if self.connect(port_name):
                        return True
            
            if ports:
                port_name = ports[0].device
                print(f"İlk port deneniyor: {port_name}")
                return self.connect(port_name)
            
            print("Serial port bulunamadı")
            return False
            
        except Exception as e:
            print(f"Serial port tespit hatası: {e}")
            return False
    
    def connect(self, port_name):
        """Belirtilen porta bağlan"""
        try:
            self.port_name = port_name
            self.serial_port = self.serial_module.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0.1,
                write_timeout=0.5
            )
            
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connected = True
            print(f"✓ Serial port bağlandı: {port_name}")
            
            self.start()
            
            socketio.emit('serial_connection', {
                'connected': True,
                'port': port_name,
                'message': f'Arduino bağlandı: {port_name}'
            })
            
            return True
            
        except Exception as e:
            print(f"✗ Serial bağlantı hatası ({port_name}): {e}")
            self.connected = False
            
            socketio.emit('serial_connection', {
                'connected': False,
                'port': None,
                'message': f'Arduino bağlantı hatası: {str(e)[:50]}'
            })
            
            return False
    
    def start(self):
        """Serial worker thread'ini başlat"""
        if not self.connected:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.thread.start()
        print("✓ Serial worker thread başlatıldı")
    
    def serial_worker(self):
        """Serial komutlarını işleyen thread"""
        print("Serial worker başlatıldı")
        
        while self.running and self.connected:
            try:
                # Komut kuyruğunu işle
                if not self.command_queue.empty():
                    try:
                        cmd_data = self.command_queue.get_nowait()
                        cmd = cmd_data.get('command')
                        cmd_char = cmd_data.get('char', '?')
                        
                        if self.serial_port and self.serial_port.is_open:
                            command_str = cmd_char
                            self.serial_port.write(command_str.encode('utf-8'))
                            self.serial_port.flush()
                            
                            socketio.emit('serial_status', {
                                'status': 'sent',
                                'command': cmd,
                                'char': cmd_char,
                                'message': f"'{cmd_char}' Arduino'ya gönderildi"
                            })
                        
                        self.command_queue.task_done()
                        
                    except queue.Empty:
                        pass
                
                # Arduino'dan gelen verileri oku
                if self.serial_port and self.serial_port.is_open:
                    try:
                        if self.serial_port.in_waiting > 0:
                            response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                            if response and len(response) > 0:
                                socketio.emit('serial_response', {
                                    'message': response,
                                    'source': 'arduino'
                                })
                    except:
                        pass
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Serial worker hatası: {e}")
                time.sleep(0.01)
    
    def send_command(self, cmd, cmd_char):
        """Serial port'a komut gönder"""
        if not self.connected or not self.serial_port:
            socketio.emit('serial_response', {
                'message': f"Demo: {cmd.upper()} komutu ('{cmd_char}') - Arduino bağlı değil",
                'source': 'demo'
            })
            return True
        
        try:
            self.command_queue.put({
                'command': cmd,
                'char': cmd_char
            })
            return True
            
        except Exception as e:
            print(f"Komut kuyruğa ekleme hatası: {e}")
            socketio.emit('serial_status', {
                'status': 'error',
                'message': f"Komut gönderilemedi: {e}"
            })
            return False
    
    def cleanup(self):
        """Kaynakları temizle"""
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("✓ Serial port kapatıldı")
            except:
                pass
        
        self.connected = False
        print("Serial manager temizlendi")

# ==================== MAVLINK TELEMETRİ (Pixhawk) ====================
class MAVLinkTelemetry:
    def __init__(self):
        self.device = "/dev/ttyUSB1"
        self.baud = 115200  # 115200 baud
        self.emit_hz = 5
        self.heartbeat_timeout = 10
        
        self.connected = False
        self.mav_connection = None
        self.mav_thread = None
        self.running = False
        self.last_heartbeat_time = 0
        self.last_emit_time = 0
        
        self.telemetry = {
            'timestamp': 0,
            'connection_status': 'DISCONNECTED',
            'gps': {
                'lat': 0,
                'lon': 0,
                'alt_m': 0,
                'vel_m_s': 0,
                'cog_deg': 0,
                'fix_type': 0,
                'satellites_visible': 0,
                'hdop': 0
            },
            'attitude': {
                'roll_rad': 0,
                'pitch_rad': 0,
                'yaw_deg': 0,
                'roll_deg': 0,
                'pitch_deg': 0
            },
            'imu': {
                'xacc': 0,
                'yacc': 0,
                'zacc': 0,
                'xgyro': 0,
                'ygyro': 0,
                'zgyro': 0
            },
            'vfr_hud': {
                'airspeed': 0,
                'groundspeed': 0,
                'heading': 0,
                'throttle': 0,
                'alt': 0,
                'climb': 0
            },
            'battery': {
                'voltage': 0,
                'current': 0,
                'remaining': 0
            },
            'system': {
                'id': 0,
                'load': 0
            }
        }
        
        print(f"MAVLink Telemetri: {self.device} @ {self.baud} baud")
    
    def connect(self):
        """MAVLink bağlantısını kur"""
        try:
            from pymavlink import mavutil
            
            print(f"MAVLink bağlanıyor: {self.device} @ {self.baud} baud")
            
            if not os.path.exists(self.device):
                print(f"✗ Port bulunamadı: {self.device}")
                self.telemetry['connection_status'] = 'PORT_NOT_FOUND'
                socketio.emit('mavlink_status', {
                    'connected': False,
                    'message': f'Port bulunamadı: {self.device}',
                    'device': self.device,
                    'baud': self.baud
                })
                return False
            
            self.mav_connection = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                autoreconnect=True,
                retries=5,
                source_system=255,
                source_component=0,
                dialect='common',
                robust_parsing=True
            )
            
            print(f"MAVLink bağlantısı kuruldu, heartbeat bekleniyor...")
            
            max_attempts = 20
            for attempt in range(max_attempts):
                try:
                    msg = self.mav_connection.recv_match(
                        type='HEARTBEAT',
                        blocking=True,
                        timeout=1
                    )
                    
                    if msg:
                        self.connected = True
                        self.last_heartbeat_time = time.time()
                        self.telemetry['connection_status'] = 'CONNECTED'
                        self.telemetry['system']['id'] = msg.get_srcSystem()
                        
                        print(f"✓ MAVLink bağlandı!")
                        print(f"  System ID: {msg.get_srcSystem()}")
                        print(f"  Component ID: {msg.get_srcComponent()}")
                        
                        self.start()
                        
                        socketio.emit('mavlink_status', {
                            'connected': True,
                            'message': 'Pixhawk bağlandı',
                            'device': self.device,
                            'baud': self.baud,
                            'system': msg.get_srcSystem(),
                            'component': msg.get_srcComponent()
                        })
                        
                        return True
                        
                except Exception as e:
                    if attempt % 5 == 0:
                        print(f"  Heartbeat denemesi {attempt+1}/{max_attempts}...")
                    continue
            
            print(f"✗ Heartbeat alınamadı ({max_attempts} deneme)")
            self.telemetry['connection_status'] = 'NO_HEARTBEAT'
            socketio.emit('mavlink_status', {
                'connected': False,
                'message': f'Heartbeat alınamadı ({max_attempts} deneme)',
                'device': self.device
            })
            return False
            
        except ImportError:
            print("✗ pymavlink kurulu değil!")
            print("  pip install pymavlink")
            self.telemetry['connection_status'] = 'PYMAVLINK_NOT_INSTALLED'
            return False
        except Exception as e:
            print(f"✗ Bağlantı hatası: {e}")
            self.telemetry['connection_status'] = f'ERROR: {str(e)[:50]}'
            socketio.emit('mavlink_status', {
                'connected': False,
                'message': f'Bağlantı hatası: {str(e)[:50]}',
                'device': self.device
            })
            return False
    
    def start(self):
        """Thread başlat"""
        if not self.connected:
            return
        
        self.running = True
        self.mav_thread = threading.Thread(target=self.mavlink_worker, daemon=True)
        self.mav_thread.start()
        print("✓ MAVLink thread başlatıldı")
    
    def mavlink_worker(self):
        """MAVLink veri işleme thread'i"""
        print("MAVLink worker başladı")
        
        while self.running and self.connected:
            try:
                current_time = time.time()
                
                if current_time - self.last_heartbeat_time > self.heartbeat_timeout:
                    print(f"⚠️ Heartbeat timeout! ({self.heartbeat_timeout}s)")
                    self.connected = False
                    self.telemetry['connection_status'] = 'HEARTBEAT_TIMEOUT'
                    socketio.emit('mavlink_status', {
                        'connected': False,
                        'message': f'Heartbeat timeout ({self.heartbeat_timeout}s)',
                        'device': self.device
                    })
                    break
                
                try:
                    msg = self.mav_connection.recv_match(
                        blocking=False,
                        timeout=0.05
                    )
                    
                    if msg:
                        self.process_message(msg)
                        
                        if msg.get_type() == 'HEARTBEAT':
                            self.last_heartbeat_time = current_time
                            
                except Exception as e:
                    pass
                
                if current_time - self.last_emit_time >= 0.2:
                    self.emit_telemetry()
                    self.last_emit_time = current_time
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Worker hatası: {e}")
                time.sleep(0.5)
        
        print("MAVLink worker durdu")
    
    def process_message(self, msg):
        """MAVLink mesajını işle"""
        msg_type = msg.get_type()
        
        try:
            self.telemetry['timestamp'] = time.time()
            
            if msg_type == 'GPS_RAW_INT':
                self.telemetry['gps']['lat'] = msg.lat / 1e7 if msg.lat != 0 else 0
                self.telemetry['gps']['lon'] = msg.lon / 1e7 if msg.lon != 0 else 0
                self.telemetry['gps']['alt_m'] = msg.alt / 1000.0 if msg.alt != 0 else 0
                self.telemetry['gps']['vel_m_s'] = msg.vel / 100.0 if msg.vel != 0 else 0
                self.telemetry['gps']['cog_deg'] = msg.cog / 100.0 if msg.cog != 0 else 0
                self.telemetry['gps']['fix_type'] = msg.fix_type
                self.telemetry['gps']['satellites_visible'] = msg.satellites_visible
                if hasattr(msg, 'eph'):
                    self.telemetry['gps']['hdop'] = msg.eph / 100.0
            
            elif msg_type == 'ATTITUDE':
                self.telemetry['attitude']['roll_rad'] = msg.roll
                self.telemetry['attitude']['pitch_rad'] = msg.pitch
                self.telemetry['attitude']['yaw_deg'] = math.degrees(msg.yaw) % 360
                self.telemetry['attitude']['roll_deg'] = math.degrees(msg.roll)
                self.telemetry['attitude']['pitch_deg'] = math.degrees(msg.pitch)
            
            elif msg_type == 'RAW_IMU':
                self.telemetry['imu']['xacc'] = msg.xacc
                self.telemetry['imu']['yacc'] = msg.yacc
                self.telemetry['imu']['zacc'] = msg.zacc
                self.telemetry['imu']['xgyro'] = msg.xgyro
                self.telemetry['imu']['ygyro'] = msg.ygyro
                self.telemetry['imu']['zgyro'] = msg.zgyro
            
            elif msg_type == 'VFR_HUD':
                self.telemetry['vfr_hud']['airspeed'] = msg.airspeed
                self.telemetry['vfr_hud']['groundspeed'] = msg.groundspeed
                self.telemetry['vfr_hud']['heading'] = msg.heading
                self.telemetry['vfr_hud']['throttle'] = msg.throttle
                self.telemetry['vfr_hud']['alt'] = msg.alt
                self.telemetry['vfr_hud']['climb'] = msg.climb
            
            elif msg_type == 'SYS_STATUS':
                self.telemetry['system']['load'] = msg.load / 10.0
                self.telemetry['battery']['voltage'] = msg.voltage_battery / 1000.0 if msg.voltage_battery != 0 else 0
                self.telemetry['battery']['current'] = msg.current_battery / 100.0 if msg.current_battery != 0 else 0
                self.telemetry['battery']['remaining'] = msg.battery_remaining
            
            elif msg_type == 'HEARTBEAT' and time.time() % 10 < 0.1:
                print(f"❤️ Heartbeat: SYS={msg.get_srcSystem()}")
            
        except Exception as e:
            print(f"Mesaj işleme hatası ({msg_type}): {e}")
    
    def emit_telemetry(self):
        """Telemetriyi WebSocket'e gönder"""
        if not self.connected:
            return
        
        try:
            socketio.emit('mavlink_telemetry', self.telemetry)
        except Exception as e:
            print(f"Telemetri gönderme hatası: {e}")
    
    def get_status(self):
        """Durum bilgisini al"""
        return {
            'connected': self.connected,
            'device': self.device,
            'baud': self.baud,
            'connection_status': self.telemetry['connection_status'],
            'last_heartbeat': self.last_heartbeat_time
        }
    
    def cleanup(self):
        """Temizlik"""
        self.running = False
        self.connected = False
        self.telemetry['connection_status'] = 'DISCONNECTED'
        
        if self.mav_thread and self.mav_thread.is_alive():
            self.mav_thread.join(timeout=1)
        
        if self.mav_connection:
            try:
                self.mav_connection.close()
                print("✓ MAVLink bağlantısı kapatıldı")
            except:
                pass
        
        print("MAVLink temizlendi")

# Manager'ları başlat
serial_manager = SerialManager()
mavlink_telemetry = MAVLinkTelemetry()

# ==================== KAMERA BAŞLATMA ====================
def initialize_camera():
    print("Kamera aranıyor...")
    
    available_cameras = []
    for i in range(9):
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                available_cameras.append(i)
                cap.release()
        except:
            pass
    
    print(f"Mevcut kameralar: {available_cameras}")
    
    if not available_cameras:
        print("Kamera bulunamadı - demo modu")
        return None
    
    for cam_idx in available_cameras:
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                
                for _ in range(3):
                    ret, frame = cap.read()
                    if not ret:
                        break
                
                if ret:
                    print(f"✓ Kamera {cam_idx} başarıyla bağlandı")
                    return cap
                else:
                    cap.release()
        except Exception as e:
            print(f"Kamera {cam_idx} hatası: {e}")
            continue
    
    print("Kamera bağlanamadı - demo modu")
    return None

camera = initialize_camera()

# ==================== VİDEO AKIŞI ====================
def generate_frames():
    """Video frame'leri oluştur"""
    print("Video akışı başlatılıyor...")
    
    while True:
        try:
            start_time = time.time()
            
            if camera is None or not camera.isOpened():
                frame = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(frame, "ROBOT KONTROL", (50, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, "Kamera: DEMO", (80, 140),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            else:
                ret, frame = camera.read()
                if not ret:
                    time.sleep(0.01)
                    continue
            
            timestamp = datetime.now().strftime('%H:%M:%S')
            cv2.putText(frame, timestamp, (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            process_time = (time.time() - start_time) * 1000
            if process_time > 0:
                fps = 1000 / process_time
                cv2.putText(frame, f"{fps:.1f}fps", (250, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
            elapsed = time.time() - start_time
            if elapsed < 0.033:
                time.sleep(0.033 - elapsed)
            
        except Exception as e:
            print(f"Frame hatası: {e}")
            time.sleep(0.1)

@app.route('/video_feed')
@login_required
def video_feed():
    """Video akışı endpoint'i"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={
                        'Cache-Control': 'no-cache, no-store, must-revalidate',
                        'Pragma': 'no-cache',
                        'Expires': '0'
                    })

# ==================== ROUTES ====================
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
                <input type="text" class="login-input" name="username" placeholder="Kullanıcı adı" required>
                <input type="password" class="login-input" name="password" placeholder="Şifre" required>
                <button type="submit" class="login-button">Giriş Yap</button>
            </form>
            <div class="demo-note">Demo için herhangi bir kullanıcı/şifre kullanabilirsiniz</div>
        </div>
    </body>
    </html>
    '''

@app.route('/logout')
def logout():
    """Çıkış"""
    session.pop('authenticated', None)
    session.pop('username', None)
    print("Kullanıcı çıkış yaptı")
    return redirect(url_for('login'))

@app.route('/')
@login_required
def index():
    """Ana sayfa"""
    return render_template('index.html', csrf_token=generate_csrf_token())

# Serial routes
@app.route('/serial/connect/<port_name>')
@login_required
def serial_connect(port_name):
    """Manuel olarak serial port bağlantısı"""
    if serial_manager.serial_module:
        if serial_manager.connect(port_name):
            return {'success': True, 'message': f'{port_name} bağlandı'}
        else:
            return {'success': False, 'message': f'{port_name} bağlanamadı'}
    return {'success': False, 'message': 'Serial modülü yok'}

@app.route('/serial/disconnect')
@login_required
def serial_disconnect():
    """Serial bağlantısını kes"""
    serial_manager.cleanup()
    return {'success': True, 'message': 'Serial bağlantısı kesildi'}

@app.route('/serial/status')
@login_required
def serial_status():
    """Serial durumunu kontrol et"""
    return {
        'connected': serial_manager.connected,
        'port': serial_manager.port_name,
        'module_loaded': serial_manager.serial_module is not None
    }

# MAVLink routes
@app.route('/mavlink/connect')
@login_required
def mavlink_connect():
    """MAVLink bağlantısını kur"""
    if mavlink_telemetry.connect():
        return {
            'success': True, 
            'message': 'Pixhawk bağlandı',
            'device': mavlink_telemetry.device,
            'baud': mavlink_telemetry.baud
        }
    else:
        return {
            'success': False, 
            'message': 'Pixhawk bağlanamadı',
            'device': mavlink_telemetry.device,
            'status': mavlink_telemetry.telemetry['connection_status']
        }

@app.route('/mavlink/disconnect')
@login_required
def mavlink_disconnect():
    """MAVLink bağlantısını kes"""
    mavlink_telemetry.cleanup()
    return {'success': True, 'message': 'Pixhawk bağlantısı kesildi'}

@app.route('/mavlink/status')
@login_required
def mavlink_status():
    """MAVLink durumunu kontrol et"""
    status = mavlink_telemetry.get_status()
    return {
        'connected': status['connected'],
        'device': status['device'],
        'baud': status['baud'],
        'connection_status': status['connection_status'],
        'last_heartbeat': status['last_heartbeat']
    }

# ==================== SOCKET İŞLEMLERİ ====================
@socketio.on('connect')
def handle_connect():
    """Client bağlandığında"""
    print(f"Client bağlandı: {request.sid}")
    emit('connection_status', {'status': 'connected', 'message': 'Bağlantı kuruldu!'})
    
    emit('serial_connection', {
        'connected': serial_manager.connected,
        'port': serial_manager.port_name,
        'message': 'Arduino bağlantısı hazır' if serial_manager.connected else 'Arduino bağlantısı yok'
    })
    
    status = mavlink_telemetry.get_status()
    emit('mavlink_status', {
        'connected': status['connected'],
        'message': 'Pixhawk bağlı' if status['connected'] else 'Pixhawk bağlı değil',
        'device': status['device'],
        'baud': status['baud']
    })

@socketio.on('disconnect')
def handle_disconnect():
    """Client bağlantısı kesildiğinde"""
    print(f"Client bağlantısı kesildi: {request.sid}")

@socketio.on('robot_command')
def handle_robot_command(data):
    """Robot komutlarını işle"""
    try:
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

        command_map = {
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
        cmd_char = command_map.get(cmd, '?')
        
        emit('status_update', {
            'message': cmd.upper(),
            'timestamp': timestamp,
            'char': cmd_char
        }, broadcast=True)
        
        success = serial_manager.send_command(cmd, cmd_char)
        if not success:
            emit('error', {'message': 'Komut gönderilemedi!'})
        
    except Exception as e:
        print(f"Komut işleme hatası: {e}")
        emit('error', {'message': 'Komut işleme hatası!'})

# ==================== API ENDPOINTS ====================
@app.route('/api/status')
@login_required
def api_status():
    """Sistem durumu API"""
    mav_status = mavlink_telemetry.get_status()
    return {
        'camera': camera is not None and camera.isOpened(),
        'serial': serial_manager.connected,
        'serial_port': serial_manager.port_name,
        'mavlink': mav_status['connected'],
        'mavlink_device': mav_status['device'],
        'mavlink_baud': mav_status['baud'],
        'mavlink_status': mav_status['connection_status'],
        'server_time': datetime.now().isoformat(),
        'status': 'running'
    }

@app.route('/api/health')
def health():
    """Sağlık kontrolü"""
    return {'status': 'healthy', 'timestamp': datetime.now().isoformat()}

# ==================== TEMİZLİK ====================
def cleanup():
    """Kaynakları temizle"""
    print("\nKaynaklar temizleniyor...")
    
    serial_manager.cleanup()
    mavlink_telemetry.cleanup()
    
    if camera and camera.isOpened():
        camera.release()
        print("Kamera kapatıldı")
    
    print("Temizlik tamamlandı")

# ==================== ANA PROGRAM ====================
if __name__ == '__main__':
    print("\n" + "="*60)
    print("   🤖 ROBOT KONTROL PANELİ - TAM SİSTEM")
    print("   🚀 Başlatılıyor...")
    print("="*60)
    print(f"   Arduino Serial: {'Bağlı' if serial_manager.connected else 'Bağlı değil'}")
    if serial_manager.connected:
        print(f"   Arduino Port: {serial_manager.port_name}")
    print(f"   Pixhawk MAVLink: /dev/ttyUSB1 @ 115200 baud")
    print("="*60)
    
    try:
        # Pixhawk bağlantısını başlat (biraz gecikmeli)
        time.sleep(2)
        print("Pixhawk bağlanıyor...")
        mavlink_telemetry.connect()
        
        socketio.run(
            app,
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=True
        )
    except KeyboardInterrupt:
        print("\n\n🛑 Server durduruluyor...")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
    finally:
        cleanup()