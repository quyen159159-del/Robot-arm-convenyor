
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <OV7670.h>

// =============================================
// WIFI ACCESS POINT
// =============================================

const char* AP_SSID = "ESP32_OV7670_CAM";
const char* AP_PASS = "12345678";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// =============================================
// CAMERA PINS
// =============================================

#define SIOD_PIN  21
#define SIOC_PIN  22

#define XCLK_PIN  32
#define PCLK_PIN  27

#define VSYNC_PIN 25
#define HREF_PIN  26

#define D0_PIN 4
#define D1_PIN 5
#define D2_PIN 18
#define D3_PIN 19
#define D4_PIN 36
#define D5_PIN 39
#define D6_PIN 34
#define D7_PIN 35

// =============================================
// CAMERA
// =============================================

OV7670* camera = nullptr;

const uint16_t CAM_W = 160;
const uint16_t CAM_H = 120;

const size_t FRAME_SIZE = CAM_W * CAM_H * 2;

// =============================================
// WEBSOCKET
// =============================================

WebSocketsServer webSocket(81);

bool cameraReady = false;

uint32_t frameCount = 0;
uint32_t fpsTimer = 0;

// =============================================
// WEBSOCKET EVENT
// =============================================

void webSocketEvent(
    uint8_t clientNum,
    WStype_t type,
    uint8_t* payload,
    size_t length
) {
    switch (type) {

        case WStype_CONNECTED:
            Serial.printf(
                "Client %u connected\n",
                clientNum
            );
            break;

        case WStype_DISCONNECTED:
            Serial.printf(
                "Client %u disconnected\n",
                clientNum
            );
            break;

        case WStype_ERROR:
            Serial.println("WebSocket error");
            break;

        default:
            break;
    }
}

// =============================================
// WIFI SETUP
// =============================================

void setupWiFi() {

    WiFi.mode(WIFI_AP);

    WiFi.softAPConfig(
        localIP,
        gateway,
        subnet
    );

    WiFi.softAP(
        AP_SSID,
        AP_PASS,
        1,
        0,
        1
    );

    Serial.println("WiFi AP started");

    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
}

// =============================================
// CAMERA SETUP
// =============================================

bool setupCamera() {

    Serial.println("Initializing OV7670...");

    camera = new OV7670(
        OV7670::Mode::QQVGA_RGB565,

        SIOD_PIN,
        SIOC_PIN,

        VSYNC_PIN,
        HREF_PIN,

        XCLK_PIN,
        PCLK_PIN,

        D0_PIN,
        D1_PIN,
        D2_PIN,
        D3_PIN,
        D4_PIN,
        D5_PIN,
        D6_PIN,
        D7_PIN
    );

    if (camera == nullptr) {
        Serial.println("Camera allocation failed");
        return false;
    }

    if (camera->xres != CAM_W ||
        camera->yres != CAM_H) {

        Serial.printf(
            "Wrong resolution: %d x %d\n",
            camera->xres,
            camera->yres
        );

        return false;
    }

    Serial.printf(
        "Camera ready: %d x %d\n",
        camera->xres,
        camera->yres
    );

    return true;
}

// =============================================
// SETUP
// =============================================

void setup() {

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("============================");
    Serial.println(" ESP32 OV7670 COLOR CAMERA");
    Serial.println("============================");

    setupWiFi();

    webSocket.begin();

    webSocket.onEvent(webSocketEvent);

    Serial.println("WebSocket server started");
    Serial.println("Port: 81");

    cameraReady = setupCamera();

    if (!cameraReady) {
        Serial.println("Camera initialization failed");
        return;
    }

    Serial.println("System ready!");
}

// =============================================
// MAIN LOOP
// =============================================

void loop() {

    webSocket.loop();

    if (!cameraReady) {
        delay(100);
        return;
    }

    // Chỉ lấy ảnh khi có client kết nối
    if (webSocket.connectedClients() == 0) {
        delay(5);
        return;
    }

    // Capture one RGB565 frame
    camera->oneFrame();

    if (camera->frame == nullptr) {
        Serial.println("Frame buffer is null");
        delay(10);
        return;
    }

    // Kiểm tra kích thước
    size_t actualSize =
        camera->xres * camera->yres * 2;

    if (actualSize != FRAME_SIZE) {
        Serial.println("Invalid frame size");
        delay(10);
        return;
    }

    // Truyền một frame hoàn chỉnh qua WebSocket
    webSocket.broadcastBIN(
        (uint8_t*)camera->frame,
        FRAME_SIZE
    );

    frameCount++;

    // FPS trên ESP32
    uint32_t now = millis();

    if (now - fpsTimer >= 1000) {

        Serial.printf(
            "Camera FPS: %lu | Clients: %u\n",
            (unsigned long)frameCount,
            webSocket.connectedClients()
        );

        frameCount = 0;
        fpsTimer = now;
    }

    // Xử lý sự kiện WebSocket
    webSocket.loop();
}
