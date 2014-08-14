#ifndef mag_h
#define mag_h

#include <Wire.h>
#include <Arduino.h>

#define address 0x1E          //0011110b, I2C 7bit address of HMC5883
#define Mode_reg 0x02         //select mode register
#define Mode_select 0x00      //continuous mesurement mode
#define X_MSB_reg 0x03

class HMC5883L{
  public:
    void get_Data(int16_t *mx, int16_t *my, int16_t *mz);
    void print_values(int16_t *mx, int16_t *my, int16_t *mz);
    void get_heading(int16_t *mx, int16_t *my);
    void initialize_HMC5883L();
};



 #endif;
