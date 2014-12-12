 # include "UGPS.h"

GPS ublox;

void setup()  {
 Serial.begin(115200);
 ublox.initializeGps();
}

void loop()  {
 ublox.updateGps(); 
}
