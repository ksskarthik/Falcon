#ifndef mag_cpp
#define mag_cpp

#include <Wire.h>
#include <Arduino.h>
#include "mag.h"
//#include <WProgram.h>

void HMC5883L::initialize_HMC5883L(){
  Wire.beginTransmission(address);
  Wire.write(Mode_reg); 
  Wire.write(Mode_select);
}

void HMC5883L::print_values(int16_t *mx, int16_t *my, int16_t *mz){
	//Print out values of each axis
  Serial.print("mx: ");
  Serial.print(*mx/1090);
  Serial.print("  my: ");
  Serial.print(*my/1090);
  Serial.print("  mz: ");
  Serial.println(*mz/1090);
}

void HMC5883L::get_Data(int16_t *mx, int16_t *my, int16_t *mz){
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    *mx = Wire.read()<<8; //X msb
    *mx |= Wire.read(); //X lsb
    *mz = Wire.read()<<8; //Z msb
    *mz |= Wire.read(); //Z lsb
    *my = Wire.read()<<8; //Y msb
    *my |= Wire.read(); //Y lsb

  }
  }
  
void HMC5883L::get_heading(int16_t *mx, int16_t *my){
    float heading = atan2(*my, *mx);
    if(heading < 0)
        heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);

}  
  #endif 
