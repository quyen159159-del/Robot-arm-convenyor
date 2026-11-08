#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================= CẤU HÌNH BĂNG CHUYỀN & CẢM BIẾN =================
const int IN1 = 25;
const int IN2 = 26;
const int ENA = 27;
const int SENSOR = 33;                 // HIGH = có vật, LOW = không có vật

const int CONVEYOR_SPEED = 200;        // Tốc độ (0 - 255)
const unsigned long STOP_DELAY_MS   = 20;   // Thấy vật liên tục 20ms -> dừng
const unsigned long RESUME_DELAY_MS = 300;  // Hết vật liên tục 300ms -> chạy lại

bool conveyorRunning = false;
bool lastRaw = false;
unsigned long lastChange = 0;
unsigned long lastScanRequest = 0;

// ================= CẤU HÌNH PCA9685 & SERVO =================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22
const float PWM_FREQ = 50.0;

const int servoPins[6]  = {0, 1, 2, 3, 4, 5};
const int servoMinUs[6] = {560, 600, 550, 550, 500, 500};
const int servoMaxUs[6] = {2460, 2400, 2450, 2450, 2300, 2300};

// Tọa độ Home khi chưa có vật và tọa độ Điểm Gắp
const float posHome[5] = {90, 88, 170, 90, 20};
const float posPick[5] = {90, 75, 175, 90, 10};

const int NUM_WAYPOINTS = 6;

// 1. Quỹ đạo cong VÒM CAO từ Điểm Gắp -> ĐỎ {0, 65, 160, 90, 5}
const float waypointsRed[NUM_WAYPOINTS][5] = {
  {90,  75, 175, 90, 10}, // Điểm 0: Vị trí Gắp (Giữ nguyên)
  {75,  96, 178, 90, 20}, // Điểm 1: Nhấc vọt lên cao
  {54, 110, 180, 90, 25}, // Điểm 2: Đỉnh đường cong (Cao nhất)
  {31, 105, 178, 90, 20}, // Điểm 3: Giữ độ cao khi xoay đế
  {13,  88, 170, 90, 12}, // Điểm 4: Hạ dần xuống vùng thả
  {0,   65, 160, 90, 5}   // Điểm 5: Vị trí Thả ĐỎ (Giữ nguyên)
};

// 2. Quỹ đạo cong VÒM CAO từ Điểm Gắp -> XANH DƯƠNG {0, 25, 90, 90, 0}
const float waypointsBlue[NUM_WAYPOINTS][5] = {
  {90,  75, 175, 90, 10}, // Điểm 0: Vị trí Gắp (Giữ nguyên)
  {75,  95, 176, 90, 20}, // Điểm 1: Nhấc vọt lên cao
  {54, 105, 172, 90, 25}, // Điểm 2: Đỉnh đường cong (Cao nhất)
  {31,  92, 158, 90, 18}, // Điểm 3: Giữ độ cao khi xoay đế
  {13,  62, 128, 90, 8},  // Điểm 4: Hạ dần xuống vùng thả
  {0,   25,  90, 90, 0}   // Điểm 5: Vị trí Thả XANH DƯƠNG (Giữ nguyên)
};

// 3. Quỹ đạo cong VÒM CAO từ Điểm Gắp -> XANH LÁ {0, 45, 135, 90, 0}
const float waypointsGreen[NUM_WAYPOINTS][5] = {
  {90,  75, 175, 90, 10}, // Điểm 0: Vị trí Gắp (Giữ nguyên)
  {75,  96, 178, 90, 20}, // Điểm 1: Nhấc vọt lên cao
  {54, 108, 176, 90, 25}, // Điểm 2: Đỉnh đường cong (Cao nhất)
  {31,  98, 166, 90, 18}, // Điểm 3: Giữ độ cao khi xoay đế
  {13,  74, 152, 90, 8},  // Điểm 4: Hạ dần xuống vùng thả
  {0,   45, 135, 90, 0}   // Điểm 5: Vị trí Thả XANH LÁ (Giữ nguyên)
};

const float (*activeWaypoints)[5] = waypointsRed;

// Khởi tạo tay máy ở vị trí HOME {90, 90, 90, 90, 90}, kẹp mở 90 độ
float currentAngle[6] = {90.0, 88.0, 170.0, 90.0, 20.0, 90.0};

// ================= BIẾN ĐIỀU KHIỂN QUỸ ĐẠO =================
const int UPDATE_INTERVAL = 10; // 10ms (100Hz)
unsigned long lastUpdateMillis = 0;

int sequenceStep = -1;
unsigned long actionStartTime = 0;

const unsigned long APPROACH_DURATION = 800;  // 0.8s đi từ Home xuống Điểm Gắp
const unsigned long CURVE_DURATION    = 1500; // 1.5s cho đường cong
const unsigned long GRIP_DURATION     = 400;  // 0.4s đóng/mở kẹp
const unsigned long PAUSE_TIME        = 400;  // 0.4s nghỉ ổn định

// Khai báo nguyên mẫu hàm
void runConveyor();
void stopConveyor();
void checkSerialCommand();
void applyServoAngle(int servoIndex, float angle);
void moveLinearSmooth(const float startPos[5], const float targetPos[5], float progress, float gripAngle);
void moveAlongCurve(float progress, bool isForward, float gripAngle);
void moveGripperSmooth(float startGrip, float targetGrip, float progress);

void runConveyor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, CONVEYOR_SPEED);
  conveyorRunning = true;
}

void stopConveyor() {
  if (conveyorRunning) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, 255);
    delay(50);
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
  conveyorRunning = false;
}

void checkSerialCommand() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      stopConveyor();
      activeWaypoints = waypointsRed;
      Serial.println("--> Nhan lenh DO (R): Ha tay tu HOME xuong diem gap!");
      actionStartTime = millis();
      sequenceStep = 1;
      return;
    } 
    else if (cmd == 'B' || cmd == 'b') {
      stopConveyor();
      activeWaypoints = waypointsBlue;
      Serial.println("--> Nhan lenh XANH DUONG (B): Ha tay tu HOME xuong diem gap!");
      actionStartTime = millis();
      sequenceStep = 1;
      return;
    } 
    else if (cmd == 'G' || cmd == 'g') {
      stopConveyor();
      activeWaypoints = waypointsGreen;
      Serial.println("--> Nhan lenh XANH LA (G): Ha tay tu HOME xuong diem gap!");
      actionStartTime = millis();
      sequenceStep = 1;
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);
  ledcAttach(ENA, 1000, 8);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pca.begin();
  pca.setOscillatorFrequency(29000000);
  pca.setPWMFreq(PWM_FREQ);

  // Đưa cánh tay về vị trí HOME {90, 90, 90, 90, 90}
  for (int i = 0; i < 6; i++) {
    applyServoAngle(i, currentAngle[i]);
  }
  delay(1000);

  bool hasObject = (digitalRead(SENSOR) == HIGH);
  lastRaw = hasObject;
  lastChange = millis();

  if (hasObject) {
    stopConveyor();
    Serial.println("Khoi dong: Dang co vat -> DUNG");
    Serial.println("SCAN");
    lastScanRequest = millis();
    sequenceStep = 0;
  } else {
    runConveyor();
    Serial.println("Khoi dong: Khong co vat -> CHAY (Tay may o HOME)");
    sequenceStep = -1;
  }
}

void loop() {
  // TRẠNG THÁI -1: BĂNG CHUYỀN CHẠY ĐỢI VẬT (ĐÃ BỎ NHẬN LỆNH THỦ CÔNG)
  if (sequenceStep == -1) {
    bool raw = (digitalRead(SENSOR) == HIGH);

    if (raw != lastRaw) {
      lastRaw = raw;
      lastChange = millis();
    }

    if (raw && (millis() - lastChange >= STOP_DELAY_MS)) {
      stopConveyor();
      Serial.println("CONVEYOR:DUNG");
      while (Serial.available() > 0) Serial.read(); // Xóa bộ đệm cũ
      Serial.println("SCAN"); // Báo Python bắt đầu đếm 1s lấy nét rồi quét
      lastScanRequest = millis();
      sequenceStep = 0;
    } 
    else if (!raw && !conveyorRunning && (millis() - lastChange >= RESUME_DELAY_MS)) {
      runConveyor();
      Serial.println("CONVEYOR:CHAY");
    }
    return;
  }

  // TRẠNG THÁI 0: ĐỢI PYTHON LẤY NÉT 1S VÀ TRẢ MÃ MÀU R/G/B
  if (sequenceStep == 0) {
    checkSerialCommand();
    if (sequenceStep != 0) {
      // Ngay khi nhận được R, G, hoặc B -> Báo trạng thái bắt đầu hạ tay
      Serial.println("ARM_STATE:HA_TAY");
      return;
    }

    // Nhắc lại lệnh SCAN mỗi 1.8 giây nếu Python chưa phản hồi
    if (millis() - lastScanRequest >= 1800) {
      Serial.println("SCAN");
      lastScanRequest = millis();
    }
    return;
  }

  // TRẠNG THÁI 1 -> 5: CHU TRÌNH CÁNH TAY (CÓ GỬI TRẠNG THÁI TỪNG BƯỚC LÊN WEB)
  if (millis() - lastUpdateMillis < UPDATE_INTERVAL) return;
  lastUpdateMillis = millis();

  unsigned long elapsed = millis() - actionStartTime;

  switch (sequenceStep) {
    case 1: // BƯỚC 1: HOME -> ĐIỂM GẮP
      if (elapsed <= APPROACH_DURATION) {
        float p = (float)elapsed / APPROACH_DURATION;
        moveLinearSmooth(posHome, posPick, p, 90.0);
      } else if (elapsed > APPROACH_DURATION + 200) {
        Serial.println("ARM_STATE:GAP_VAT"); // Báo sang bước đóng kẹp gắp vật
        actionStartTime = millis();
        sequenceStep = 2;
      }
      break;

    case 2: // BƯỚC 2: ĐÓNG KẸP GẮP VẬT (90 -> 0)
      if (elapsed <= GRIP_DURATION) {
        float p = (float)elapsed / GRIP_DURATION;
        moveGripperSmooth(90.0, 0.0, p);
      } else if (elapsed > GRIP_DURATION + PAUSE_TIME) {
        Serial.println("ARM_STATE:DI_CHUYEN"); // Báo sang bước đi đường cong tới ô màu
        actionStartTime = millis();
        sequenceStep = 3;
      }
      break;

    case 3: // BƯỚC 3: ĐI ĐƯỜNG CONG TỚI ĐIỂM THẢ
      if (elapsed <= CURVE_DURATION) {
        float p = (float)elapsed / CURVE_DURATION;
        float smoothP = (1.0 - cos(p * PI)) / 2.0;
        moveAlongCurve(smoothP, true, 0.0);
      } else if (elapsed > CURVE_DURATION + PAUSE_TIME) {
        Serial.println("ARM_STATE:THA_VAT"); // Báo sang bước mở kẹp thả vật
        actionStartTime = millis();
        sequenceStep = 4;
      }
      break;

    case 4: // BƯỚC 4: MỞ KẸP THẢ VẬT (0 -> 90)
      if (elapsed <= GRIP_DURATION) {
        float p = (float)elapsed / GRIP_DURATION;
        moveGripperSmooth(0.0, 90.0, p);
      } else if (elapsed > GRIP_DURATION + PAUSE_TIME) {
        Serial.println("ARM_STATE:VE_HOME"); // Báo sang bước quay về Home
        actionStartTime = millis();
        sequenceStep = 5;
      }
      break;

    case 5: // BƯỚC 5: QUAY VỀ HOME THEO ĐƯỜNG CONG
      if (elapsed <= CURVE_DURATION) {
        float p = (float)elapsed / CURVE_DURATION;
        float smoothP = (1.0 - cos(p * PI)) / 2.0;
        moveAlongCurve(smoothP, false, 90.0);
      } else if (elapsed > CURVE_DURATION + PAUSE_TIME) {
        while (Serial.available() > 0) Serial.read();
        lastRaw = (digitalRead(SENSOR) == HIGH);
        lastChange = millis();
        Serial.println("DONE"); // Báo đã về tới Home
        sequenceStep = -1;
      }
      break;
  }
}

void moveLinearSmooth(const float startPos[5], const float targetPos[5], float progress, float gripAngle) {
  float smoothP = (1.0 - cos(progress * PI)) / 2.0;
  for (int i = 0; i < 5; i++) {
    currentAngle[i] = startPos[i] + (targetPos[i] - startPos[i]) * smoothP;
    applyServoAngle(i, currentAngle[i]);
  }
  currentAngle[5] = gripAngle;
  applyServoAngle(5, currentAngle[5]);
}

void moveAlongCurve(float progress, bool isForward, float gripAngle) {
  float effectiveProgress = isForward ? progress : (1.0 - progress);
  float totalSegments = NUM_WAYPOINTS - 1;
  float exactPos = effectiveProgress * totalSegments;

  int indexA = (int)exactPos;
  if (indexA >= NUM_WAYPOINTS - 1) indexA = NUM_WAYPOINTS - 2;
  int indexB = indexA + 1;
  float localProgress = exactPos - indexA;

  for (int i = 0; i < 5; i++) {
    float startA = (!isForward && indexA == 0) ? posHome[i] : activeWaypoints[indexA][i];
    float endB   = activeWaypoints[indexB][i];
    currentAngle[i] = startA + (endB - startA) * localProgress;
    applyServoAngle(i, currentAngle[i]);
  }

  currentAngle[5] = gripAngle;
  applyServoAngle(5, currentAngle[5]);
}

void moveGripperSmooth(float startGrip, float targetGrip, float progress) {
  float smoothP = (1.0 - cos(progress * PI)) / 2.0;
  currentAngle[5] = startGrip + (targetGrip - startGrip) * smoothP;
  applyServoAngle(5, currentAngle[5]);
}

void applyServoAngle(int servoIndex, float angle) {
  uint16_t pulseUs = servoMinUs[servoIndex] + (angle / 180.0) * (servoMaxUs[servoIndex] - servoMinUs[servoIndex]);
  pca.writeMicroseconds(servoPins[servoIndex], pulseUs);
}