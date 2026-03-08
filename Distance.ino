#include "esp_camera.h"
#include <esp_now.h>
#include <WiFi.h>

// ─── Camera pin map (AI-Thinker) ───────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── 🎯 ระบุ MAC Address ของ ESP32 ตัวรับ ───────────────────
uint8_t receiverAddress[] = {0x3C, 0x8A, 0x1F, 0x9D, 0xB6, 0x94};

typedef struct struct_message {
  float currentDistance; // เก็บค่าระยะทางเป็นเซนติเมตร
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// ─── ค่า Calibration ที่หามาได้ ──────────────────────────────
const float X_MIN = 48.0;   // พิกเซลซ้ายสุด (0 cm)
const float X_MAX = 260.0;  // พิกเซลขวาสุด (30 cm)
const float DIST_MAX = 30.0;// ความยาวราง (cm)

void setup() {
  Serial.begin(115200);

  // 1. เปิดกล้อง
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;   
  config.jpeg_quality = 10;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera Init Failed");
    while (1) delay(1000);
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0); 

  // 2. เปิด Wi-Fi และ ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("--- กล้องพร้อม! ล็อคเป้าหมายไปที่ ESP32 ตัวหลักแล้ว ---");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  uint16_t *buf = (uint16_t *)fb->buf;
  int w = fb->width, h = fb->height;

  long sumX = 0, count = 0;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint16_t pixel = buf[y * w + x];
      uint8_t r = (pixel >> 11) & 0x1F;  r = (r << 3) | (r >> 2);
      uint8_t g = (pixel >>  5) & 0x3F;  g = (g << 2) | (g >> 4);
      uint8_t b = (pixel >>  0) & 0x1F;  b = (b << 3) | (b >> 2);

      uint8_t gray = (r * 3 + g * 6 + b) / 10;

      if (gray >= 250) { 
        sumX += x;
        count++;
      }
    }
  }
  esp_camera_fb_return(fb);

  if (count >= 50 && count <= 500) {
    float centroidX = (float)sumX / count;
    
    // แปลงพิกเซลเป็น CM
    float distanceCm = ((centroidX - X_MIN) / (X_MAX - X_MIN)) * DIST_MAX;
    
    // ล็อคค่าไม่ให้หลุดกรอบ 0 - 30 
    if (distanceCm < 0.0) distanceCm = 0.0;
    if (distanceCm > 30.0) distanceCm = 30.0;

    // ส่งค่าผ่าน ESP-NOW
    myData.currentDistance = distanceCm;
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.print("✅ ส่งสำเร็จ! | 📏 ระยะทาง: "); 
    } else {
      Serial.print("❌ ส่งล้มเหลว | 📏 ระยะทาง: "); 
    }
    Serial.print(distanceCm); Serial.println(" cm");
    
  } else {
    Serial.println("❌ มองไม่เห็นรถ");
  }

  delay(50); // อัปเดตข้อมูลทุกๆ 50ms ให้ทันใจมอเตอร์
}