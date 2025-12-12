"""
Robot Control Panel - With Serial Support
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

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gizli_anahtar_123'
app.config['SESSION_COOKIE_HTTPONLY'] = True
app.config['SESSION_COOKIE_SAMESITE'] = 'Lax'

# WebSocket bağlantısı için CORS ayarı
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
    ping_interval=5,
    ping_timeout=10,
    logger=False,
    engineio_logger=False
)

# Security decorator
def login_required(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if not session.get('authenticated', False):
            return redirect(url_for('login'))
        return f(*args, **kwargs)
    return decorated_function

# CSRF token
def generate_csrf_token():
    if 'csrf_token' not in session:
        session['csrf_token'] = secrets.token_hex(16)
    return session['csrf_token']

# ==================== SERIAL PORT YÖNETİCİSİ ====================
class SerialManager:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.running = False
        self.thread = None
        self.serial_module = None
        self.port_name = None
        
        # Serial port'u denemek için pyserial'ı import et
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
            # Mevcut portları listele
            ports = list(self.serial_tools.comports())
            print(f"Bulunan serial portlar: {[p.device for p in ports]}")
            
            # Arduino veya USB portlarını bul
            for port in ports:
                port_name = port.device
                # Tipik Arduino/Serial port isimleri
                if 'ttyUSB' in port_name or 'ttyACM' in port_name or 'COM' in port_name:
                    print(f"Potansiyel Arduino portu: {port_name}")
                    if self.connect(port_name):
                        return True
            
            # Herhangi bir port bulunamazsa ilk portu dene
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
            
            # Bağlantıyı test et
            time.sleep(2)  # Arduino'nun boot etmesi için bekle
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connected = True
            print(f"✓ Serial port bağlandı: {port_name}")
            
            # Worker thread'i başlat
            self.start()
            
            return True
            
        except Exception as e:
            print(f"✗ Serial bağlantı hatası ({port_name}): {e}")
            self.connected = False
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
                        
                        # Serial port'a komutu gönder
                        if self.serial_port and self.serial_port.is_open:
                            # Komutu Arduino'ya gönder (sadece karakter, newline olmadan)
                            command_str = cmd_char
                            self.serial_port.write(command_str.encode('utf-8'))
                            self.serial_port.flush()
                            
                            print(f"📤 Serial: {cmd} → '{cmd_char}' gönderildi")
                            
                            # Komut başarı mesajını gönder
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
                            if response:
                                print(f"📥 Arduino: {response}")
                                # Gelen veriyi tüm client'lara gönder
                                socketio.emit('serial_response', {
                                    'message': response,
                                    'source': 'arduino'
                                })
                    except:
                        pass
                
                time.sleep(0.005)  # Daha hızlı işleme için
                
            except Exception as e:
                print(f"Serial worker hatası: {e}")
                time.sleep(0.1)
    
    def send_command(self, cmd, cmd_char):
        """Serial port'a komut gönder"""
        if not self.connected or not self.serial_port:
            print(f"⚠️ Serial bağlı değil, komut gönderilemedi: {cmd}")
            socketio.emit('serial_status', {
                'status': 'error',
                'message': 'Serial bağlantısı yok'
            })
            return False
        
        try:
            # Komutu kuyruğa ekle
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

# Serial manager'ı başlat
serial_manager = SerialManager()

# ==================== KAMERA BAŞLATMA ====================
def initialize_camera():
    print("Kamera aranıyor...")
    
    # Önce mevcut kameraları kontrol et
    available_cameras = []
    for i in range(4):
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
    
    # İlk bulunan kamerayı kullan
    for cam_idx in available_cameras:
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if cap.isOpened():
                # Düşük gecikme ayarları
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                
                # Test için birkaç frame oku
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

# Kamera başlat
camera = initialize_camera()

# ==================== VİDEO AKIŞI ====================
def generate_frames():
    """Video frame'leri oluştur"""
    print("Video akışı başlatılıyor...")
    
    while True:
        try:
            start_time = time.time()
            
            if camera is None or not camera.isOpened():
                # Demo frame
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
            
            # Zaman damgası
            timestamp = datetime.now().strftime('%H:%M:%S')
            cv2.putText(frame, timestamp, (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # FPS göstergesi
            process_time = (time.time() - start_time) * 1000
            cv2.putText(frame, f"{process_time:.0f}ms", (250, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # JPEG encode
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
            # Performans için kısa bekleme
            time.sleep(0.001)
            
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

# ==================== SOCKET İŞLEMLERİ ====================
@socketio.on('connect')
def handle_connect():
    """Client bağlandığında"""
    print(f"Client bağlandı: {request.sid}")
    emit('connection_status', {'status': 'connected', 'message': 'Bağlantı kuruldu!'})
    
    # Serial durumunu gönder
    emit('serial_connection', {
        'connected': serial_manager.connected,
        'port': serial_manager.port_name,
        'message': 'Serial bağlantısı hazır' if serial_manager.connected else 'Serial bağlantısı yok'
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
        
        # CSRF kontrolü
        if csrf_token != session.get('csrf_token'):
            emit('error', {'message': 'Güvenlik hatası!'})
            return
        
        valid_commands = ['forward', 'backward', 'left', 'right', 'stop', 'forward_left', 'forward_right', 'backward_left', 'backward_right']
        if cmd not in valid_commands:
            emit('error', {'message': 'Geçersiz komut!'})
            return

        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f"[{timestamp}] Komut: {cmd.upper()}")

        # Komut haritalama (Arduino komutları)
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
        
        # Durum güncellemesini gönder
        emit('status_update', {
            'message': cmd.upper(),
            'timestamp': timestamp,
            'char': cmd_char
        }, broadcast=True)
        
        # Serial port'a komutu gönder
        if serial_manager.serial_module:
            success = serial_manager.send_command(cmd, cmd_char)
            if success:
                print(f"Serial komut gönderildi: {cmd} -> '{cmd_char}'")
            else:
                emit('error', {'message': 'Serial komut gönderilemedi!'})
        else:
            emit('serial_response', {
                'message': f"Demo: {cmd.upper()} komutu ('{cmd_char}')",
                'source': 'demo'
            })
        
    except Exception as e:
        print(f"Komut işleme hatası: {e}")
        emit('error', {'message': 'Komut işleme hatası!'})

# ==================== API ENDPOINTS ====================
@app.route('/api/status')
@login_required
def api_status():
    """Sistem durumu API"""
    return {
        'camera': camera is not None and camera.isOpened(),
        'serial': serial_manager.connected,
        'serial_port': serial_manager.port_name,
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
    
    # Serial manager'ı temizle
    serial_manager.cleanup()
    
    # Kamera'yı kapat
    if camera and camera.isOpened():
        camera.release()
        print("Kamera kapatıldı")
    
    print("Temizlik tamamlandı")

# ==================== ANA PROGRAM ====================
if __name__ == '__main__':
    print("\n" + "="*50)
    print("   🤖 ROBOT KONTROL PANELİ")
    print("   🚀 Başlatılıyor...")
    print("="*50)
    print(f"   Serial durumu: {'Bağlı' if serial_manager.connected else 'Bağlı değil'}")
    if serial_manager.connected:
        print(f"   Serial port: {serial_manager.port_name}")
    print("="*50)
    
    try:
        # HTTP ve WebSocket server'ı başlat
        socketio.run(
            app,
            host='0.0.0.0',
            port=7000,
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
