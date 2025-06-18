#include <WiFi.h>
#include <WiFiUdp.h>
#include <MPU9250_asukiaaa.h>

const char* apSSID = "DroneAP";
const char* apPassword = "12345678";

IPAddress laptopIP(192, 168,); // Replace with your laptop's IP address
const int udpPort = 8888;

WiFiUDP udp;
MPU9250_asukiaaa mpu;

void setup() {
    Serial.begin(115200);

    WiFi.softAP(apSSID, apPassword);
    Serial.print("ESP32 AP IP Address: ");  
    Serial.println(WiFi.softAPIP());

    Wire.begin();
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();
    Serial.println("MPU9250 Initialized!");
}

void loop() {
    mpu.accelUpdate();
    mpu.gyroUpdate();

    float ax = mpu.accelX();
    float ay = mpu.accelY();
    float az = mpu.accelZ();

    float gx = mpu.gyroX();
    float gy = mpu.gyroY();
    float gz = mpu.gyroZ();

    char msg[200];
    snprintf(msg, sizeof(msg),
             "AX:%.3f,AY:%.3f,AZ:%.3f,GX:%.3f,GY:%.3f,GZ:%.3f",
             ax, ay, az, gx, gy, gz);

    udp.beginPacket(laptopIP, udpPort);
    udp.print(msg);
    udp.endPacket();

    Serial.println(msg);

    delay(50); 
}
