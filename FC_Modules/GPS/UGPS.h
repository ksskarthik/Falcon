#ifndef UGPS_h
#define UGPS_h

# include <Arduino.h>

#define UBLOX_5HZ   {UBX_5HZ,sizeof(UBX_5HZ)}
#define UBLOX_38400 {(unsigned char *)"$PUBX,41,1,0003,0003,38400,0*24\r\n",0}
#define UBLOX_CONFIGS UBLOX_5HZ,UBLOX_38400
#define GPS_MAXIDLE_DETECTING 200 // 2 seconds at 100Hz
#define GPS_MAXIDLE 500           // 5 seconds at 100Hz

enum {
  GPS_INVALID_ACCURACY = 0xFFFFFFFF, 
  GPS_INVALID_AGE = 0xFFFFFFFF, 
  GPS_INVALID_ANGLE = 0x7FFFFFFF, 
  GPS_INVALID_ALTITUDE = 2147483647,//999999999, 
  GPS_INVALID_DATE = 0,
  GPS_INVALID_TIME = 0xFFFFFFFF, 
  GPS_INVALID_SPEED = 999999999, 
  GPS_INVALID_FIX_TIME = 0xFFFFFFFF
};

enum { 
    GPS_DETECTING = 0, 
    GPS_NOFIX = 1,
    GPS_FIX2D = 2,
    GPS_FIX3D = 3,
    GPS_FIX3DD = 4 // differential fix 
};


/////////////////////////////////////////////////////////////////////////////////////////////////

 class GPS{
   public :
      void ubloxParseData();
      void gpsSendConfig();z
      void ubloxInit();
      int ubloxProcessData(unsigned char data);
      void updateGps() ;
      void initializeGpsData() ;
      void initializeGps() ;
 };
 
 #endif;
