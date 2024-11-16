#include <Wire.h>

#define MPU6050_ADDR 0x68 // MPU6050 I2C address
#define PWR_MGMT_1 0x6B  // Power Management 1 register

// Registers for accelerometer and gyroscope
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// Accelerometer and Gyroscope Offsets
int accelOffsets[3] = {0, 0, 0}; // X, Y, Z accelerometer offsets
int gyroOffsets[3] = {0, 0, 0};  // X, Y, Z gyroscope offsets

// Number of samples for calibration
const int samples = 100;

// Setup function
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050 (by default, it's in sleep mode after power up)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Wake up MPU6050
  Wire.endTransmission();

  // Allow time for MPU6050 to wake up
  delay(1000);

  // Perform the calibration
  calibrateMPU6050();
}

// Calibration function
void calibrateMPU6050() {
  long accelSum[3] = {0, 0, 0};  // Sum of accelerometer readings
  long gyroSum[3] = {0, 0, 0};   // Sum of gyroscope readings

  Serial.println("Calibrating MPU6050... Please keep the sensor stationary.");

  // Collect sensor data for calibration
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;

    // Read accelerometer data
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    ax = ((int16_t)Wire.read() << 8) | Wire.read();
    ay = ((int16_t)Wire.read() << 8) | Wire.read();
    az = ((int16_t)Wire.read() << 8) | Wire.read();

    // Read gyroscope data
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    gx = ((int16_t)Wire.read() << 8) | Wire.read();
    gy = ((int16_t)Wire.read() << 8) | Wire.read();
    gz = ((int16_t)Wire.read() << 8) | Wire.read();

    // Add to the sums
    accelSum[0] += ax;
    accelSum[1] += ay;
    accelSum[2] += az;
    gyroSum[0] += gx;
    gyroSum[1] += gy;
    gyroSum[2] += gz;

    delay(10);  // Short delay between readings
  }

  // Calculate the average offsets
  accelOffsets[0] = accelSum[0] / samples;
  accelOffsets[1] = accelSum[1] / samples;
  accelOffsets[2] = accelSum[2] / samples;
  gyroOffsets[0] = gyroSum[0] / samples;
  gyroOffsets[1] = gyroSum[1] / samples;
  gyroOffsets[2] = gyroSum[2] / samples;

  // Print the calibration results
  Serial.print("Accelerometer Offsets: X=");
  Serial.print(accelOffsets[0]);
  Serial.print(" Y=");
  Serial.print(accelOffsets[1]);
  Serial.print(" Z=");
  Serial.println(accelOffsets[2]);

  Serial.print("Gyroscope Offsets: X=");
  Serial.print(gyroOffsets[0]);
  Serial.print(" Y=");
  Serial.print(gyroOffsets[1]);
  Serial.print(" Z=");
  Serial.println(gyroOffsets[2]);

  Serial.println("Calibration complete!");
}

// Function to read sensor values with offsets and calculate pitch, roll, yaw
void readMPU6050WithOffsets() {
  int16_t ax, ay, az, gx, gy, gz;

  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  ax = ((int16_t)Wire.read() << 8) | Wire.read();
  ay = ((int16_t)Wire.read() << 8) | Wire.read();
  az = ((int16_t)Wire.read() << 8) | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  gx = ((int16_t)Wire.read() << 8) | Wire.read();
  gy = ((int16_t)Wire.read() << 8) | Wire.read();
  gz = ((int16_t)Wire.read() << 8) | Wire.read();

  // Apply calibration offsets
  ax -= accelOffsets[0];
  ay -= accelOffsets[1];
  az -= accelOffsets[2];
  gx -= gyroOffsets[0];
  gy -= gyroOffsets[1];
  gz -= gyroOffsets[2];

  // Calculate Pitch, Roll, and Yaw
  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float roll = atan2(-ax, az) * 180.0 / PI;
  float yaw = atan2(gz, sqrt(gx * gx + gy * gy)) * 180.0 / PI;

  // Print corrected values and calculated angles
  Serial.print("Corrected Accelerometer: X=");
  Serial.print(ax);
  Serial.print(" Y=");
  Serial.print(ay);
  Serial.print(" Z=");
  Serial.println(az);

  Serial.print("Corrected Gyroscope: X=");
  Serial.print(gx);
  Serial.print(" Y=");
  Serial.print(gy);
  Serial.print(" Z=");
  Serial.println(gz);

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Yaw: ");
  Serial.println(yaw);
}

void loop() {
  // Continuously read sensor data with offsets
  readMPU6050WithOffsets();
  delay(1000);  // 1-second delay between readings
}
