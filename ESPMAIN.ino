#include <Arduino.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <esp_now.h>

#define STEP_PIN 2
#define DIR_PIN 5
#define MS1_PIN 19
#define MS2_PIN 18
#define TRIG_PIN 17
#define ECHO_PIN 16

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

float min_dist = 2.5;
float max_dist = 26.5;

double Setpoint = 1.0;
double Input, Output;

float filtered_distance = Setpoint;

// PID constants
double Kp = 600.0;
double Ki = 0.0;
double Kd = 150.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

long max_steps = 600;
long min_steps = -600;

volatile long target_step_sync = 0;

typedef struct struct_message {
  float currentDistance;
} struct_message;

struct_message incomingData;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  memcpy(&incomingData, data, sizeof(incomingData));

  // รับค่าจาก ESP-CAM
  Setpoint = incomingData.currentDistance;

  // จำกัดช่วง Setpoint ให้ตรงกับช่วง Ultrasonic
  if (Setpoint < min_dist) Setpoint = min_dist;
  if (Setpoint > max_dist) Setpoint = max_dist;

  Serial.print("New Setpoint: ");
  Serial.println(Setpoint);
}

// Task ควบคุมมอเตอร์ให้หมุนต่อเนื่อง
void MotorTask(void *pvParameters) {

  long current_target = 0;

  for (;;) {

    if (target_step_sync != current_target) {
      current_target = target_step_sync;
      stepper.moveTo(current_target);
    }

    stepper.run();

    vTaskDelay(1);
  }
}


// อ่านค่า Ultrasonic พร้อม Filter
float getFilteredDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 10000);

  // ถ้า duration = 0 หรือสั้นผิดปกติ แสดงว่าชนเซนเซอร์
  if (duration == 0 || duration < 150) {
    target_step_sync = min_steps;
    Serial.println("Hit Ultrasonic -> Move MIN");
    return filtered_distance;
  }

  float raw_cm = (duration / 2.0) * 0.0343;

  // ถ้าไกลเกินช่วงเซนเซอร์
  if (raw_cm > max_dist) {
    target_step_sync = max_steps;
    Serial.println("Max Distance -> Move MAX");
    return filtered_distance;
  }

  // จำกัดช่วงอ่านค่าปกติ
  if (raw_cm < min_dist) raw_cm = min_dist;

  filtered_distance = (0.7 * raw_cm) + (0.3 * filtered_distance);

  return filtered_distance;
}


void setup() {

  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);

  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  stepper.setMaxSpeed(30000.0);
  stepper.setAcceleration(20000.0);
  stepper.setCurrentPosition(min_steps);
  target_step_sync = min_steps;
  stepper.moveTo(min_steps);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-350, 350);
  myPID.SetSampleTime(20);

  xTaskCreatePinnedToCore(
    MotorTask,
    "MotorTask",
    4096,
    NULL,
    3,
    NULL,
    1);
}


// ... (ส่วนประกาศตัวแปรด้านบนเหมือนเดิม) ...

// เพิ่มค่าความเร็วในการขยับ (Increment Speed)
// ยิ่งค่ามาก มอเตอร์ยิ่งเลื่อนไปหาเป้าหมายเร็วขึ้น
void loop() {

  Input = getFilteredDistance();

  // ถ้าอยู่ใกล้ Setpoint ให้หยุด
  if (abs(Input - Setpoint) < 0.4) {
    target_step_sync = stepper.currentPosition();
  } else {

    // แปลงระยะเป็นตำแหน่ง step
    long targetStep = (Input - Setpoint) * 100;

    // จำกัดช่วง step
    if (targetStep > max_steps) targetStep = max_steps;
    if (targetStep < min_steps) targetStep = min_steps;

    target_step_sync = targetStep;
  }

  Serial.print("Distance: ");
  Serial.print(Input);
  Serial.print("  Step: ");
  Serial.println(target_step_sync);

  delay(1);
}