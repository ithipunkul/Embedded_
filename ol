#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <AccelStepper.h>
#include <PID_v1.h> 

#define WIFI_SSID       "JooYai"
#define WIFI_PASSWORD   "12345678"
#define DATABASE_URL    "https://car-and-beam-default-rtdb.asia-southeast1.firebasedatabase.app"

NoAuth no_auth;
FirebaseApp app;
WiFiClientSecure ssl_client;
DefaultNetwork network;
AsyncClientClass aClient(ssl_client, getNetwork(network));
RealtimeDatabase Database;

#define STEP_PIN  2
#define DIR_PIN   5
#define MS1_PIN   19
#define MS2_PIN   18
#define TRIG_PIN  17
#define ECHO_PIN  16

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

float min_dist = 2.5;  
float max_dist = 26.5; 

double Setpoint = 13.5; 
double Input, Output;
float filtered_distance = Setpoint; 

// ใช้ค่า PID ที่นุ่มนวลที่สุด
double Kp = 15.0; 
double Ki = 0.0;  
double Kd = 60.0; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

long max_steps = 600;  
long min_steps = -600; 
volatile long target_step_sync = 0;

// *** 1. แก้ไข MotorTask ให้หมุนสมูทขึ้น ***
void MotorTask(void * pvParameters) {
    long current_target = 0; 
    for(;;) {
        if (target_step_sync != current_target) {
            current_target = target_step_sync;
            stepper.moveTo(current_target);
        }
        stepper.run();
        vTaskDelay(1);
    }
 }
float getFilteredDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 10000); 
    if (duration == 0) return filtered_distance; 
    float raw_cm = (duration / 2.0) * 0.0343;
    if (raw_cm > max_dist) raw_cm = max_dist;
    if (raw_cm < min_dist) raw_cm = min_dist;
    filtered_distance = (0.85 * raw_cm) + (0.15 * filtered_distance); 
    return filtered_distance;
}

// *** เพิ่มฟังก์ชันนี้เพื่อรับคำสั่งสไลด์เดอร์จากหน้าเว็บ ***
void getSetpointCb(AsyncResult &aResult) {
    if (aResult.available()) {
        RealtimeDatabaseResult &rtb = aResult.to<RealtimeDatabaseResult>();
        float new_set = rtb.to<float>();
        // ป้องกันค่าขยะ ถ้าระยะปลอดภัยให้ตั้งเป็นเป้าหมายใหม่เลย
        if (new_set >= 2.5 && new_set <= 26.5) {
            Setpoint = new_set;
        }
    }
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

    // *** 2. ปรับสปีดลงนิดนึงเพื่อความเนียน ***
    stepper.setMaxSpeed(2500.0);
    stepper.setAcceleration(2000.0);

    myPID.SetMode(AUTOMATIC);           
    myPID.SetOutputLimits(min_steps, max_steps); 
    
    // *** 3. เพิ่มเวลาให้มอเตอร์ได้หายใจ ***
    myPID.SetSampleTime(40);            

    xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL, 3, NULL, 1);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected!");

    ssl_client.setInsecure();
    initializeApp(aClient, app, getAuth(no_auth));
    app.getApp<RealtimeDatabase>(Database);
    Database.url(DATABASE_URL);
}

void loop() {
    app.loop();

    Input = getFilteredDistance();
    if (myPID.Compute()) {
        long target_step = (long)Output;
        double error = Setpoint - Input;
        
        // ขยาย Deadband เป็น 2.5 เพื่อสู้กับอาการล้อฝืด (Slip-Stick)
        if (abs(error) < 2.5) {
            target_step = 0;  
        }
        
        target_step = -target_step; 
        if (target_step > max_steps) target_step = max_steps;
        if (target_step < min_steps) target_step = min_steps;
        target_step_sync = target_step;
    }

    static unsigned long lastSendMillis = 0;
    if (app.ready() && millis() - lastSendMillis >= 500) {
        lastSendMillis = millis();
        
        // 1. ดึงค่า Setpoint ใหม่จากเว็บ
        Database.get(aClient, "/car_beam/target_setpoint", getSetpointCb);
        
        // 2. ส่งค่ากลับไปโชว์กราฟ
        double currentError = Setpoint - Input;
        Database.set<float>(aClient, "/car_beam/position", Input);
        Database.set<float>(aClient, "/car_beam/setpoint", Setpoint);
        Database.set<float>(aClient, "/car_beam/error", currentError);
        
        Serial.printf("Setpoint:%.2f, Pos:%.2f\n", Setpoint, Input);
    }
}
