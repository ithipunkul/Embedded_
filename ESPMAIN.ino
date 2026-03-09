#include <esp_now.h>
#include <WiFi.h>

// ─── สร้างโครงสร้างข้อมูลให้ตรงกับฝั่งส่ง ───
typedef struct struct_message {
  float currentDistance; // รับค่าระยะทางเป็นเซนติเมตร
} struct_message;

struct_message myData;

// ─── ฟังก์ชันที่จะทำงานอัตโนมัติเมื่อข้อมูลบินมาถึง ───
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.print("📥 ตำแหน่งรถตอนนี้: ");
  Serial.print(myData.currentDistance);
  Serial.println(" cm");
  
  // 💡 เดี๋ยวเราจะเอาตัวแปร myData.currentDistance ไปใส่ในสมการ PID ควบคุมมอเตอร์ตรงนี้แหละครับ
}

void setup() {
  Serial.begin(115200);
  
  // เปิด Wi-Fi ในโหมด Station 
  WiFi.mode(WIFI_STA);

  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // ลงทะเบียนฟังก์ชันรอรับข้อมูล
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  Serial.println("--- 🟢 ESP32 ตัวหลักพร้อมรับข้อมูลจากกล้องแล้ว! ---");
}

void loop() {
  // ฝั่งนี้ปล่อยว่างไว้ได้เลยครับ ESP-NOW ทำงานอยู่เบื้องหลังให้เอง
  // เดี๋ยวเราค่อยมาเขียนโค้ดอ่านเซนเซอร์/คุมมอเตอร์ในนี้ทีหลัง
}