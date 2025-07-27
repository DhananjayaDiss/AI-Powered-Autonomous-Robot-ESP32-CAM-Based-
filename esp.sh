#include "esp_camera.h"
#include "WiFi.h"
#include "WebSocketsServer.h"
#include "ArduinoJson.h"
#include "Wire.h"
#include "MPU6050.h"
#include "NewPing.h"

// Hardware configuration
#define MOTOR_L1 12
#define MOTOR_L2 13
#define MOTOR_R1 14
#define MOTOR_R2 15
#define TRIG_PIN 2
#define ECHO_PIN 4
#define BUZZER_PIN 16

// Communication
WebSocketsServer webSocket = WebSocketsServer(81);
MPU6050 mpu;
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

struct SensorData {
    float distance;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    unsigned long timestamp;
};

struct MotorCommand {
    int leftSpeed;   // -255 to 255
    int rightSpeed;  // -255 to 255
    int duration;    // milliseconds
};

void setup() {
    Serial.begin(115200);
    
    // Initialize camera
    setupCamera();
    
    // Initialize WiFi
    WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    // Initialize WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    // Initialize sensors
    Wire.begin();
    mpu.initialize();
    
    // Initialize motors
    setupMotors();
    
    Serial.println("ESP32-CAM Robot Ready!");
    Serial.println("IP: " + WiFi.localIP().toString());
}

void loop() {
    webSocket.loop();
    
    // Collect and send sensor data every 100ms
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead > 100) {
        SensorData data = collectSensorData();
        sendSensorData(data);
        lastSensorRead = millis();
    }
    
    delay(10);
}

void setupCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

SensorData collectSensorData() {
    SensorData data;
    
    // Ultrasonic sensor
    data.distance = sonar.ping_cm();
    
    // IMU data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    
    data.accelX = ax / 16384.0;
    data.accelY = ay / 16384.0;
    data.accelZ = az / 16384.0;
    data.gyroX = gx / 131.0;
    data.gyroY = gy / 131.0;
    data.gyroZ = gz / 131.0;
    
    data.timestamp = millis();
    
    return data;
}

void sendSensorData(SensorData data) {
    DynamicJsonDocument doc(512);
    doc["type"] = "sensor_data";
    doc["distance"] = data.distance;
    doc["accel"]["x"] = data.accelX;
    doc["accel"]["y"] = data.accelY;
    doc["accel"]["z"] = data.accelZ;
    doc["gyro"]["x"] = data.gyroX;
    doc["gyro"]["y"] = data.gyroY;
    doc["gyro"]["z"] = data.gyroZ;
    doc["timestamp"] = data.timestamp;
    
    String message;
    serializeJson(doc, message);
    webSocket.broadcastTXT(message);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED:
            Serial.printf("Client %u connected\n", num);
            break;
            
        case WStype_TEXT:
            handleCommand(payload);
            break;
            
        case WStype_DISCONNECTED:
            Serial.printf("Client %u disconnected\n", num);
            break;
    }
}

void handleCommand(uint8_t * payload) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, payload);
    
    String command = doc["command"];
    
    if (command == "move") {
        int leftSpeed = doc["left_speed"];
        int rightSpeed = doc["right_speed"];
        int duration = doc["duration"];
        
        executeMotorCommand(leftSpeed, rightSpeed, duration);
    }
    else if (command == "get_frame") {
        sendCameraFrame();
    }
    else if (command == "beep") {
        int frequency = doc["frequency"];
        int duration = doc["duration"];
        playTone(frequency, duration);
    }
}

void executeMotorCommand(int leftSpeed, int rightSpeed, int duration) {
    // Control left motor
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_L1, HIGH);
        digitalWrite(MOTOR_L2, LOW);
    } else if (leftSpeed < 0) {
        digitalWrite(MOTOR_L1, LOW);
        digitalWrite(MOTOR_L2, HIGH);
    } else {
        digitalWrite(MOTOR_L1, LOW);
        digitalWrite(MOTOR_L2, LOW);
    }
    
    // Control right motor
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_R1, HIGH);
        digitalWrite(MOTOR_R2, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(MOTOR_R1, LOW);
        digitalWrite(MOTOR_R2, HIGH);
    } else {
        digitalWrite(MOTOR_R1, LOW);
        digitalWrite(MOTOR_R2, LOW);
    }
    
    // Set PWM speeds
    ledcWrite(0, abs(leftSpeed));
    ledcWrite(1, abs(rightSpeed));
    
    // Execute for specified duration
    if (duration > 0) {
        delay(duration);
        // Stop motors
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
}

void sendCameraFrame() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }
    
    // Send frame via WebSocket (base64 encoded)
    size_t base64_len = 4 * ((fb->len + 2) / 3);
    char* base64_buf = (char*)malloc(base64_len + 1);
    
    if (base64_buf) {
        base64_encode(base64_buf, (char*)fb->buf, fb->len);
        
        DynamicJsonDocument doc(base64_len + 100);
        doc["type"] = "camera_frame";
        doc["data"] = base64_buf;
        doc["length"] = fb->len;
        
        String message;
        serializeJson(doc, message);
        webSocket.broadcastTXT(message);
        
        free(base64_buf);
    }
    
    esp_camera_fb_return(fb);

}