#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SoftwareSerial.h> //softserail for UART
#include <TinyGPS.h>
#include <Adafruit_VL53L0X.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

SemaphoreHandle_t xSemaphore;

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 6
#define SCL_PIN 7
#endif

#define i2c_Address 0x3c 
#define SLAVE_ADDR 0x78 
SoftwareSerial mySerial(41, 42);

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET -1
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


TinyGPS gps;

Adafruit_BMP280 bme;
MPU9250_asukiaaa mySensor;

float aX, aY, aZ, aSqrt, gX, gY, gZ;
float latitude, longitude;
int distance_sensor1;
int distance_sensor2;
int distance_sensor3;
int distance_sensor4;
String dist="";
int tol=100;


#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#define SENSOR3_WIRE Wire
#define SENSOR4_WIRE Wire

// Setup mode for doing reads
typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_DEFAULT;
uint8_t show_command_list = 1;

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

// Actual object, could probably include in structure above61
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;

// Setup for 4 sensors
sensorList_t sensors[] = {
    {&sensor1, &SENSOR1_WIRE, 0x30, 10,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
    {&sensor2, &SENSOR2_WIRE, 0x31, 11,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0, 0},
    {&sensor3, &SENSOR3_WIRE, 0x32, 12,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {&sensor4, &SENSOR4_WIRE, 0x33, 13,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;


void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}
//====================================================================
// Simple Sync read sensors.
//====================================================================
void read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  digitalWrite(13, HIGH);
  uint32_t start_time = millis();
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
  }
  uint32_t delta_time = millis() - start_time;
  digitalWrite(13, LOW);

  Serial.print(delta_time, DEC);
  Serial.print(F(" "));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(i, DEC);
    Serial.print(F(":"));
    Serial.print(ranges_mm[i], DEC);
    Serial.print(F(" "));
    Serial.print(stop_times[i] - start_time, DEC);
    if (timeouts[i])
      Serial.print(F("(TIMEOUT) "));
    else
      Serial.print(F("          "));
    start_time = stop_times[i];
  }
  Serial.println();
}

//===============================================================
// Continuous range test code
//===============================================================

void start_continuous_range(uint16_t cycle_time) {
  if (cycle_time == 0)
    cycle_time = 100;
  Serial.print(F("start Continuous range mode cycle time: "));
  Serial.println(cycle_time, DEC);
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->startRangeContinuous(cycle_time); // do 100ms cycle
  }
  sensors_pending = ALL_SENSORS_PENDING;
  sensor_last_cycle_time = millis();
}

void stop_continuous_range() {
  Serial.println(F("Stop Continuous range mode"));
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->stopRangeContinuous();
  }
  delay(100); // give time for it to complete.
}

void Process_continuous_range() {

  uint16_t mask = 1;
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    bool range_complete = false;
    if (sensors_pending & mask) {
      if (range_complete) {
        sensors[i].range = sensors[i].psensor->readRangeResult();
        sensors[i].sensor_status = sensors[i].psensor->readRangeStatus();
        sensors_pending ^= mask;
      }
    }
    mask <<= 1; // setup to test next one
  }
  // See if we have all of our sensors read OK
  uint32_t delta_time = millis() - sensor_last_cycle_time;
  if (!sensors_pending || (delta_time > 1000)) {
    digitalWrite(13, !digitalRead(13));
    Serial.print(delta_time, DEC);
    Serial.print(F("("));
    Serial.print(sensors_pending, HEX);
    Serial.print(F(")"));
    mask = 1;
    for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
      Serial.print(F(" : "));
      if (sensors_pending & mask)
        Serial.print(F("TTT")); // show timeout in this one
      else {
        Serial.print(sensors[i].range, DEC);
        if (sensors[i].sensor_status == VL53L0X_ERROR_NONE)
          Serial.print(F("  "));
        else {
          Serial.print(F("#"));
          Serial.print(sensors[i].sensor_status, DEC);
        }
      }
    }
    // setup for next pass
    Serial.println();
    sensor_last_cycle_time = millis();
    sensors_pending = ALL_SENSORS_PENDING;
  }
}

void requestEvent() {
  vTaskSuspendAll();
String response = String("x") + "," + String(aX) + "," +
                 String(aY) + "," +
                 String(aZ) + "," +
                 String(gX) + "," +
                 String(gY) + "," +
                 String(gZ) + "," +
                 String(latitude) + "," + 
                 String(longitude) + "," +
                 String(dist) + "," +
                 String(distance_sensor1) +"," +
                 String(distance_sensor2) +"," +
                 String(distance_sensor3) +"," +
                 String(distance_sensor4) +"," + String("y");


    byte responseData[response.length() + 1];
    response.toCharArray((char*)responseData, response.length() + 1);
    Wire1.write(responseData, response.length() + 1);
    xTaskResumeAll();
}

// Task to read accelerometer data
void taskReadAccel(void *pvParameters) {
  while (true) {
    if (mySensor.accelUpdate() == 0) {
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      aSqrt = mySensor.accelSqrt();
      
      Serial.print("Accel -> X: " + String(aX) + " Y: " + String(aY) + " Z: " + String(aZ) + " Sqrt: " + String(aSqrt) + "\t");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow other tasks to run
  }
}

// Task to read gyroscope data
void taskReadGyro(void *pvParameters) {
  while (true) {
    if (mySensor.gyroUpdate() == 0) {
      gX = mySensor.gyroX();
      gY = mySensor.gyroY();
      gZ = mySensor.gyroZ();
      Serial.print("Gyro -> X: " + String(gX) + " Y: " + String(gY) + " Z: " + String(gZ) + "\t");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskOLED(void *pvParameters) {

  while(true){
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("We will win");
  display.display();
  
  vTaskDelay(500 / portTICK_PERIOD_MS);
   
}
}

void taskGPS(void *pvParameters) {
  while (true) {
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // For one second, read data from GPS
    while (mySerial.available() > 0) {
      gps.encode(mySerial.read());
      newData = true;
    }

    if (newData) {
     
      gps.f_get_position(&latitude, &longitude);
      Serial.print("Latitude: "); Serial.println(latitude, 6);
      Serial.print("Longitude: "); Serial.println(longitude, 6);
      
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to prevent flooding
  }
}


void taskToF(void *pvParameters){
  while (1) {
    if (sensors[0].psensor != nullptr) {
      distance_sensor1 = sensors[0].psensor->readRange();
      Serial.print("1: ");
      Serial.println(distance_sensor1);
    } else {
      Serial.println("Sensor 1 not initialized");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay between sensor readings

    if (sensors[1].psensor != nullptr) {
      distance_sensor2 = sensors[1].psensor->readRange();
      Serial.print("2: ");
      Serial.println(distance_sensor2);
    } else {
      Serial.println("Sensor 2 not initialized");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (sensors[2].psensor != nullptr) {
      distance_sensor3 = sensors[2].psensor->readRange();
      Serial.print("3: ");
      Serial.println(distance_sensor3);
    } else {
      Serial.println("Sensor 3 not initialized");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (sensors[3].psensor != nullptr) {
      distance_sensor4 = sensors[3].psensor->readRange();
      Serial.print("4: ");
      Serial.println(distance_sensor4);
    } else {
      Serial.println("Sensor 4 not initialized");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay between cycles
  }
}





void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  while (!Serial);

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire1.begin(SLAVE_ADDR,4,5,400000); //I2C slave
  Wire1.onRequest(requestEvent);

#ifdef _ESP32_HAL_I2C_H_
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#else
  Wire.begin();
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();

  Serial.println("ESP32 I2C Slave Ready...");

  delay(250);
  display.begin(i2c_Address, true); //OLED display
  display.display();
  delay(2000);
  display.clearDisplay();
  delay(2000);

    // initialize all of the pins.
  Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  
  for (int i = 0; i < COUNT_SENSORS; i++) { //ToF sensor
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }
  Serial.println(F("Starting..."));
  Initialize_sensors();

    xSemaphore = xSemaphoreCreateMutex();

  // Create RTOS tasks for each sensor
  xTaskCreatePinnedToCore(taskReadAccel, "ReadAccel", 2048, NULL, 1, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskReadGyro, "ReadGyro", 2048, NULL, 1, NULL, tskNO_AFFINITY);

  xTaskCreatePinnedToCore(taskOLED, "OLEDTASK", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskGPS, "OLEDGPS", 2048, NULL, 2, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(taskToF, "TOF", 2048, NULL, 3, NULL, tskNO_AFFINITY);

}


void loop() {
  // Empty loop as all tasks run independently
}
