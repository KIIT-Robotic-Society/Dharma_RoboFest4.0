#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS.h>

HardwareSerial mySerial(0);

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  
  Serial.print("Simple TinyGPS library v. "); 
  Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (mySerial.available())
    {
      char c = mySerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    float latitude = (flat == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flat;
    Serial.println(latitude,6);
    Serial.print(" LON=");
    float longitude = (flat == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flon;
    Serial.println(longitude,6);
    Serial.print(" SAT=");
    int satellite=(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.println(satellite);
    /*Serial.print(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());*/
    Serial.println("----------------xxxx---------------");
    delay(100);
  }
  
  gps.stats(&chars, &sentences, &failed);
  //Serial.print(" CHARS=");
  //Serial.print(chars);
  //Serial.print(" SENTENCES=");
  //Serial.print(sentences);
  //Serial.print(" CSUM ERR=");
  //Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}