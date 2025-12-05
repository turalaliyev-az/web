import cv2
import time
import numpy as np
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gizli'
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='eventlet',
    ping_interval=0.5,
    ping_timeout=10
)

# ==================== KAMERA BAŞLATMA (UBUNTU 100% İŞLƏYİR) ====================
def initialize_camera():
    print("Ubuntu-da kamera axtarılır...")
    backends = [cv2.CAP_V4L2, cv2.CAP_ANY, cv2.CAP_GSTREAMER]
    
    for backend in backends:
        for i in range(5):
            cap = cv2.VideoCapture(i, backend)
            if cap.isOpened():
                print(f"KAMERA TAPILDI → /dev/video{i} | Backend: {backend}")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # Gecikməni öldürür
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Ən vacib!
                return cap
            cap.release()
    
    print("Kamera tapılmadı → demo kare göstəriləcək")
    return None

camera = initialize_camera()

# ==================== VİDEO YAYIMI (GÖRÜNTÜ MÜTLƏQ GƏLƏCƏK) ====================
def generate_frames():
    while True:
        if camera is None or not camera.isOpened():
            # Kamera yoxdursa sadə rəngli kare
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(frame, "KAMERA YOXDUR", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, "Demo mode", (70, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            success, frame = camera.read()
            if not success:
                # Nadir hallarda baş verir, gözləyirik
                time.sleep(0.05)
                continue

        # JPEG sıxılması – 55 ən optimal
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

# ==================== KOMUT ALMA ====================
@socketio.on('robot_command')
def handle_robot_command(data):
    cmd = data.get('command', 'bilinmir')
    print(f"[{time.strftime('%H:%M:%S')}] KOMUT → {cmd.upper()}")
    emit('status_update', {'message': cmd.upper()}, broadcast=True)

# ==================== BAŞLATMA ====================
if __name__ == '__main__':
    print("\n" + "="*50)
    print("   ROBOT KONTROL PANELİ BAŞLADILIR")
    print("   http://127.0.0.1:9966")
    print("   İnternet üçün → ngrok http 9966")
    print("="*50 + "\n")
    
    try:
        socketio.run(app, host='0.0.0.0', port=9966, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\nServer dayandırıldı.")
    finally:
        if camera and camera.isOpened():
            camera.release()
        print("Kamera bağlandı. Sağ ol!")
