#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  // Initialize communication with the MPU-6050
  Wire.begin();
  Serial.begin(115200);

  // Initialize the device
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed. Please check your connection with the sensor.");
    while (1);
  } else {
    Serial.println("MPU6050 connection successful.");
  }
}

void loop() {
  // Read raw values: accel (x, y, z), temperature, gyro (x, y, z)
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print raw sensor data
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100); // Delay a short while before reading again
}
