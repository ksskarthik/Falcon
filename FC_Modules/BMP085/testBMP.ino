#include <Wire.h>
#include "BMP085.h"

BMP085 bmp;						//object name bmp
  
void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
	while (1) {}				//code gets stopped
  }
}
  
void loop() {
  Serial.print("Altitude = ");
   Serial.print(bmp.readAltitude());				//uncompensated one
  Serial.print("         ");
	Serial.print("Real altitude = ");
    Serial.println(bmp.readAltitude(101500));		//compensated one with sea level pressure (considering weather conditions) =101500 here
}
