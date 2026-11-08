import cv2
import numpy as np
import websocket
import serial
import time
import threading
from flask import Flask, Response, jsonify, render_template_string

# =============================================
# CẤU HÌNH SERIAL & WEBSOCKET
# =============================================
SERIAL_PORT = 'COM7'   # Đổi thành cổng COM thực tế của ESP32 Cánh tay
BAUD_RATE = 115200
WS_URL = "ws://192.168.4.1:81/"

CAM_W, CAM_H = 160, 120
FRAME_BYTES = CAM_W * CAM_H * 2
ROI_X1, ROI_Y1 = 64, 44
ROI_X2, ROI_Y2 = 96, 76

FOCUS_DELAY_SEC = 1.0
WIN_NAME = "OV7670 COLOR SORTING - IOT SYSTEM"

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    time.sleep(2)
    print(f"[SERIAL] Da ket noi thanh cong voi {SERIAL_PORT}")
except Exception as e:
    print(f"[SERIAL] Canh bao: Khong the mo cong {SERIAL_PORT} ({e}).")
    ser = None

# =============================================
# BIẾN TOÀN CỤC CHIA SẺ CHO WEB IOT DASHBOARD
# =============================================
iot_state = {
    "conveyor": "ĐANG CHẠY",
    "arm": "CHỜ Ở HOME",
    "color": "NONE",
    "hsv": "H: 0.0 | S: 0.0 | V: 0.0",
    "fps": 0.0,
    "count_red": 0,
    "count_green": 0,
    "count_blue": 0
}
latest_jpeg = None
frame_lock = threading.Lock()

# =============================================
# COLOR CLASS & XỬ LÝ ẢNH
# =============================================
class Color:
    GREEN = "GREEN"
    BLUE = "BLUE"
    RED = "RED"
    UNKNOWN = "UNKNOWN"

DRAW_COLORS = {
    Color.RED: (0, 0, 255),
    Color.GREEN: (0, 255, 0),
    Color.BLUE: (255, 120, 0),
    Color.UNKNOWN: (180, 180, 180)
}

def rgb565_to_bgr(raw):
    pixels = np.frombuffer(raw, dtype="<u2").reshape(CAM_H, CAM_W)
    r = ((pixels >> 11) & 0x1F) * 255 // 31
    g = ((pixels >> 5) & 0x3F) * 255 // 63
    b = (pixels & 0x1F) * 255 // 31
    return np.dstack((b, g, r)).astype(np.uint8)

def calculate_roi_hsv(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    S = float(np.mean(hsv[:, :, 1]))
    V = float(np.mean(hsv[:, :, 2]))
    H_channel = hsv[:, :, 0].astype(np.float32)

    theta = H_channel * (np.pi / 90.0)
    mean_x = np.mean(np.cos(theta))
    mean_y = np.mean(np.sin(theta))
    H_rad = np.arctan2(mean_y, mean_x)
    H = float(H_rad * (90.0 / np.pi))
    if H < 0:
        H += 180.0
    return H, S, V

def detect_color(H, S, V):
    if S < 50 or V < 40:
        return Color.UNKNOWN
    if 30 <= H <= 88:
        return Color.GREEN
    elif 89 <= H <= 140:
        return Color.BLUE
    elif H < 30 or H >= 145:
        return Color.RED
    return Color.UNKNOWN

def force_closest_color(H):
    if 30 <= H <= 88:
        return Color.GREEN
    elif 89 <= H <= 145:
        return Color.BLUE
    return Color.RED

def send_command_to_robot(color_name):
    cmd_map = {
        Color.RED: b'R\n',
        Color.GREEN: b'G\n',
        Color.BLUE: b'B\n'
    }
    if color_name in cmd_map:
        iot_state["color"] = color_name
        iot_state["arm"] = f"ĐANG HẠ TAY GẮP ({color_name})"
        if color_name == Color.RED:
            iot_state["count_red"] += 1
        elif color_name == Color.GREEN:
            iot_state["count_green"] += 1
        elif color_name == Color.BLUE:
            iot_state["count_blue"] += 1

        if ser and ser.is_open:
            ser.write(cmd_map[color_name])
            ser.flush()
            print(f"--> [ROBOT] Da gui lenh '{color_name}' xuong ESP32.")
        else:
            print(f"--> [MO PHONG] Gui lenh: {color_name}")

# =============================================
# GIAO DIỆN WEB IOT (FLASK SERVER)
# =============================================
app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <title>IoT Dashboard - Phân Loại Màu & Cánh Tay Robot</title>
    <style>
        body { font-family: 'Segoe UI', Arial, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; }
        h1 { text-align: center; color: #38bdf8; margin-bottom: 20px; font-size: 24px; }
        .container { display: flex; flex-wrap: wrap; gap: 20px; max-width: 1100px; margin: auto; }
        .card { background: #1e293b; border-radius: 12px; padding: 20px; box-shadow: 0 4px 15px rgba(0,0,0,0.4); flex: 1; min-width: 320px; }
        .cam-box { text-align: center; }
        .cam-box img { width: 100%; max-width: 520px; border-radius: 8px; border: 2px solid #334155; }
        .status-item { display: flex; justify-content: space-between; align-items: center; padding: 14px; margin: 10px 0; background: #0f172a; border-radius: 8px; }
        .label { font-size: 15px; color: #94a3b8; }
        .val { font-size: 16px; font-weight: bold; padding: 6px 14px; border-radius: 6px; }
        .running { background: #065f46; color: #34d399; }
        .stopped { background: #7f1d1d; color: #f87171; }
        
        /* Màu sắc động cho từng trạng thái của Cánh tay */
        .arm-wait { background: #334155; color: #e2e8f0; }
        .arm-scan { background: #854d0e; color: #fde047; }
        .arm-pick { background: #9a3412; color: #fdba74; }
        .arm-move { background: #1e3a8a; color: #93c5fd; }
        .arm-drop { background: #065f46; color: #6ee7b7; }

        .color-RED { background: #dc2626; color: white; }
        .color-GREEN { background: #16a34a; color: white; }
        .color-BLUE { background: #2563eb; color: white; }
        .color-NONE, .color-UNKNOWN { background: #475569; color: #cbd5e1; }
        .counters { display: flex; gap: 10px; margin-top: 15px; text-align: center; }
        .c-box { flex: 1; padding: 14px; border-radius: 8px; font-weight: bold; }
    </style>
</head>
<body>
    <h1>HỆ THỐNG GIÁM SÁT IOT - BĂNG CHUYỀN & CÁNH TAY PHÂN LOẠI MÀU</h1>
    <div class="container">
        <!-- Cột Camera -->
        <div class="card cam-box">
            <h3>Camera OV7670 Trực Tiếp</h3>
            <img src="/video_feed" alt="Camera Stream">
            <p id="hsv_text" style="color:#94a3b8; margin-top:10px;">H: 0.0 | S: 0.0 | V: 0.0</p>
        </div>

        <!-- Cột Trạng thái -->
        <div class="card">
            <h3>Trạng Thái Hệ Thống Real-time</h3>
            
            <div class="status-item">
                <span class="label">Trạng thái Băng chuyền:</span>
                <span id="conveyor_st" class="val running">ĐANG CHẠY</span>
            </div>

            <div class="status-item">
                <span class="label">Trạng thái Cánh tay:</span>
                <span id="arm_st" class="val arm-wait">CHỜ Ở HOME</span>
            </div>

            <div class="status-item">
                <span class="label">Màu vừa nhận diện:</span>
                <span id="color_st" class="val color-NONE">NONE</span>
            </div>

            <h3 style="margin-top: 25px;">Sản Lượng Đã Phân Loại</h3>
            <div class="counters">
                <div class="c-box color-RED">ĐỎ<br><span id="cnt_r" style="font-size:24px;">0</span></div>
                <div class="c-box color-GREEN">XANH LÁ<br><span id="cnt_g" style="font-size:24px;">0</span></div>
                <div class="c-box color-BLUE">XANH DƯƠNG<br><span id="cnt_b" style="font-size:24px;">0</span></div>
            </div>
        </div>
    </div>

    <script>
        function getArmClass(armText) {
            if (armText.includes("LẤY NÉT") || armText.includes("QUÉT")) return "val arm-scan";
            if (armText.includes("HẠ TAY") || armText.includes("KẸP GẮP")) return "val arm-pick";
            if (armText.includes("DI CHUYỂN") || armText.includes("QUAY VỀ")) return "val arm-move";
            if (armText.includes("THẢ VẬT")) return "val arm-drop";
            return "val arm-wait";
        }

        function updateDashboard() {
            fetch('/status')
                .then(res => res.json())
                .then(data => {
                    const convEl = document.getElementById('conveyor_st');
                    convEl.innerText = data.conveyor;
                    convEl.className = 'val ' + (data.conveyor.includes('DỪNG') ? 'stopped' : 'running');

                    const armEl = document.getElementById('arm_st');
                    armEl.innerText = data.arm;
                    armEl.className = getArmClass(data.arm);

                    const colEl = document.getElementById('color_st');
                    colEl.innerText = data.color;
                    colEl.className = 'val color-' + data.color;

                    document.getElementById('hsv_text').innerText = data.hsv + " | FPS: " + data.fps.toFixed(1);
                    document.getElementById('cnt_r').innerText = data.count_red;
                    document.getElementById('cnt_g').innerText = data.count_green;
                    document.getElementById('cnt_b').innerText = data.count_blue;
                });
        }
        setInterval(updateDashboard, 150); // Cập nhật nhạy hơn mỗi 150ms
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/status')
def get_status():
    return jsonify(iot_state)

def generate_mjpeg():
    while True:
        with frame_lock:
            jpg_bytes = latest_jpeg
        if jpg_bytes is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

# =============================================
# PHÂN TÍCH ĐẦY ĐỦ TRẠNG THÁI TỪ SERIAL ESP32
# =============================================
def parse_esp32_log(line):
    # 1. Trạng thái Băng chuyền
    if "DUNG" in line or "SCAN" in line:
        iot_state["conveyor"] = "ĐANG DỪNG (CÓ VẬT)"
    elif "CHAY" in line:
        iot_state["conveyor"] = "ĐANG CHẠY"
        iot_state["arm"] = "CHỜ Ở HOME"

    # 2. Đầy đủ 6 trạng thái của Cánh tay
    if "SCAN" in line:
        iot_state["arm"] = "ĐANG LẤY NÉT & QUÉT MÀU"
    elif "ARM_STATE:HA_TAY" in line or "Ha tay" in line:
        iot_state["arm"] = f"ĐANG HẠ TAY GẮP ({iot_state['color']})"
    elif "ARM_STATE:GAP_VAT" in line or "Dong kep" in line:
        iot_state["arm"] = f"ĐANG KẸP GẮP VẬT ({iot_state['color']})"
    elif "ARM_STATE:DI_CHUYEN" in line or "duong cong" in line:
        iot_state["arm"] = f"ĐANG DI CHUYỂN TỚI Ô {iot_state['color']}"
    elif "ARM_STATE:THA_VAT" in line or "tha vat" in line:
        iot_state["arm"] = f"ĐANG MỞ KẸP THẢ VẬT ({iot_state['color']})"
    elif "ARM_STATE:VE_HOME" in line or "Quay ve" in line:
        iot_state["arm"] = "ĐANG QUAY VỀ VỊ TRÍ HOME"
    elif "DONE" in line:
        iot_state["arm"] = "CHỜ Ở HOME"

# =============================================
# HIỂN THỊ KHUNG HÌNH & CẬP NHẬT STREAM WEB
# =============================================
def update_display_and_web(frame, H, S, V, detected, forced, fps):
    global latest_jpeg

    display = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
    color = DRAW_COLORS[detected]

    cv2.rectangle(display, (ROI_X1 * 4, ROI_Y1 * 4), (ROI_X2 * 4, ROI_Y2 * 4), color, 2)
    cv2.putText(display, f"FPS: {fps:.1f}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(display, f"H: {H:.1f} S: {S:.1f} V: {V:.1f}", (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(display, f"COLOR: {detected}", (15, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    if detected == Color.UNKNOWN:
        cv2.putText(display, f"FORCED: {forced}", (15, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.75, DRAW_COLORS[forced], 2)

    ret, buffer = cv2.imencode('.jpg', display)
    if ret:
        with frame_lock:
            latest_jpeg = buffer.tobytes()

    cv2.imshow(WIN_NAME, display)

def connect_camera():
    while True:
        try:
            print("[CAMERA] Dang ket noi toi ESP32 OV7670...")
            ws = websocket.create_connection(WS_URL, timeout=2, enable_multithread=True)
            print("[CAMERA] Da ket noi OV7670 thanh cong!")
            return ws
        except Exception as e:
            print("[CAMERA] Loi ket noi:", e)
            time.sleep(2)

# =============================================
# VÒNG LẶP CHÍNH (CHỈ CHẠY TỰ ĐỘNG)
# =============================================
def main():
    web_thread = threading.Thread(target=run_flask, daemon=True)
    web_thread.start()
    print("==========================================================")
    print(" WEB IOT DASHBOARD DA SAN SANG TAI: http://localhost:5000 ")
    print("==========================================================")

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)

    ws = None
    frame_count = 0
    fps = 0.0
    fps_timer = time.perf_counter()

    waiting_for_focus = False
    scan_trigger_time = 0.0

    try:
        while True:
            # 1. Đọc HẾT mọi tín hiệu Serial đang chờ từ ESP32 để không bị sót trạng thái
            if ser and ser.is_open:
                try:
                    while ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"[ESP32-ARM]: {line}")
                            parse_esp32_log(line)
                            if "SCAN" in line and not waiting_for_focus:
                                waiting_for_focus = True
                                scan_trigger_time = time.perf_counter()
                            elif "DONE" in line:
                                waiting_for_focus = False
                except Exception:
                    pass

            # 2. Kết nối Camera OV7670
            if ws is None:
                ws = connect_camera()
                frame_count = 0
                fps_timer = time.perf_counter()

            # 3. Nhận ảnh từ WebSocket
            try:
                raw = ws.recv()
                if not isinstance(raw, bytes) or len(raw) != FRAME_BYTES:
                    continue
            except websocket.WebSocketTimeoutException:
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
                continue
            except Exception:
                ws = None
                continue

            # 4. Giải mã và tính HSV
            frame = rgb565_to_bgr(raw)
            roi = frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
            H, S, V = calculate_roi_hsv(roi)
            detected = detect_color(H, S, V)
            forced = force_closest_color(H)

            iot_state["hsv"] = f"H: {H:.1f} | S: {S:.1f} | V: {V:.1f}"

            # 5. Tính FPS
            frame_count += 1
            now = time.perf_counter()
            if now - fps_timer >= 1.0:
                fps = frame_count / (now - fps_timer)
                iot_state["fps"] = fps
                frame_count = 0
                fps_timer = now

            # 6. Tự động chốt màu và gắp sau khi dừng băng chuyền đủ 1 giây
            if waiting_for_focus and (now - scan_trigger_time >= FOCUS_DELAY_SEC):
                final_color = detected if detected != Color.UNKNOWN else forced
                print(f"[AUTO-SCAN] Chot mau: {final_color}")
                send_command_to_robot(final_color)
                waiting_for_focus = False

            # 7. Hiển thị OpenCV & Phát lên Web
            update_display_and_web(frame, H, S, V, detected, forced, fps)

            # Nhấn 'q' hoặc ESC để đóng chương trình
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        if ws:
            try: ws.close()
            except: pass
        if ser and ser.is_open:
            ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()