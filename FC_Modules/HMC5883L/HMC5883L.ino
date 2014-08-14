#include <Wire.h> 
//#include "mag.c"
#include "mag.h"

HMC5883L mag;
int16_t mx, my, mz;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  mag.initialize_HMC5883L();
  Wire.endTransmission();
}

void loop(){
  
  Wire.beginTransmission(address);
  Wire.write(X_MSB_reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  mag.get_Data(&mx,&my,&mz);
  mag.print_values(&mx,&my,&mz);
//  mag.get_heading(&mx,&my);
}
