#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;

// Define the SoftwareSerial pins for RX (GPIO 2) and TX (GPIO 1) on ESP32
SoftwareSerial mySerial(D3, D2);  // RX, TX

void setup()
{
  Serial.begin(9600);  // Start Serial for debugging
  mySerial.begin(9600);  // Start SoftwareSerial for GPS communication

  Serial.print("Simple TinyGPS library v. "); 
  Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  bool newData = false;
  unsigned long start = millis();
  
  // For one second, we parse GPS data
  while (millis() - start < 1000)
  {
    while (mySerial.available())  // Check if GPS data is available
    {
      char c = mySerial.read();  // Read the GPS data
      if (gps.encode(c))  // Parse the GPS data
      {
        newData = true;
      }
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    
    // Get latitude and longitude from GPS
    gps.f_get_position(&flat, &flon, &age);

    // Get the number of satellites
    int sat = gps.satellites();

    // Print latitude, longitude, and satellite count to the Serial Monitor
    Serial.print("Latitude: "); 
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "INVALID" : String(flat, 6));
    
    Serial.print(" Longitude: ");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? "INVALID" : String(flon, 6));
    
    Serial.print(" Satellites: ");
    Serial.println(sat == TinyGPS::GPS_INVALID_SATELLITES ? "INVALID" : String(sat));
  }
  else
  {
    Serial.println("No new GPS data received.");
  }
}
