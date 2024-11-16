#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t accX, accY, accZ; // Accelerometer raw data
int16_t gyroX, gyroY, gyroZ; // Gyroscope raw data
double roll, pitch, yaw; // Angle values
double kalmanX, kalmanY; // Kalman filter outputs

// Kalman filter variables
double q_angle = 0.001; // Process noise variance for the accelerometer
double q_bias = 0.003;  // Process noise variance for the gyro
double r_measure = 0.03; // Measurement noise variance

double angleX = 0; // Kalman angle
double angleY = 0; // Kalman angle
double biasX = 0; // Bias
double biasY = 0; // Bias

unsigned long lastTime;
double dt;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  lastTime = millis();
}

void loop() {
  // Read raw accelerometer and gyroscope data
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // Convert raw values to double for calculations
  double accX_double = (double)accX;
  double accY_double = (double)accY;
  double accZ_double = (double)accZ;
  double gyroX_double = (double)gyroX * (PI / 180); // Convert to radians/sec
  double gyroY_double = (double)gyroY * (PI / 180); // Convert to radians/sec

  // Calculate elapsed time
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0; // Convert to seconds
  lastTime = now;

  // Calculate the pitch and roll from accelerometer data
  double accRoll = atan2(accY_double, accZ_double) * 180 / PI; // Roll calculation
  double accPitch = atan2(accX_double, sqrt(accY_double * accY_double + accZ_double * accZ_double)) * 180 / PI; // Pitch calculation

  // Kalman filter for X (roll)
  angleX += (gyroX_double - biasX) * dt; // Predict
  double kalmanGainX = q_angle / (q_angle + r_measure);
  angleX += kalmanGainX * (accRoll - angleX); // Correct
  q_angle += q_bias * dt;

  // Kalman filter for Y (pitch)
  angleY += (gyroY_double - biasY) * dt; // Predict
  double kalmanGainY = q_angle / (q_angle + r_measure);
  angleY += kalmanGainY * (accPitch - angleY); // Correct
  q_angle += q_bias * dt;

  // Print out the results
  Serial.print("Roll: ");
  Serial.print(accRoll);
  Serial.print(" | Kalman Roll: ");
  Serial.print(angleX);
  Serial.print(" | Pitch: ");
  Serial.print(accPitch);
  Serial.print(" | Kalman Pitch: ");
  Serial.println(angleY);

  delay(100); // Adjust delay as needed
}
