#ifndef UGPS_cpp
#define UGPS_cpp

# include <Arduino.h>
# include "UGPS.h"

byte  gpsConfigsSent;  // number of cfg msgs sent
byte  gpsConfigTimer;  // 0 = no more work, 1 = send now, >1 wait

static const unsigned char UBX_5HZ[] = {0xb5,0x62,0x06,0x08,0x06,0x00,0xc8,0x00,0x01,0x00,0x01,0x00,0xde,0x6a};
const unsigned long gpsBaudRates[] = { 9600L, 19200L, 38400L, 57600L, 115200L};
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct GeodeticPosition {
  long latitude;
  long longitude;
  long altitude;
};
struct gpsData {
    int32_t  lat,lon;  // position as degrees (*10E7)
    int32_t  course;   // degrees (*10E5)
    uint32_t speed;    // cm/s
    int32_t  height;   // mm (from ellipsoid)
    uint32_t accuracy; // mm
    uint32_t fixage;   // fix 
    uint32_t fixtime;  // fix 
    uint32_t sentences; // sentences/packets processed from gps (just statistics)
    uint8_t  state;    // gps state
    uint8_t  sats;     // number of satellites active
    uint8_t  baudrate; // current baudrate (index) - used by autodetection
    uint8_t  type;     // current type - used by autodetection
    uint32_t idlecount; // how many times gpsUpdate has been called without getting a valid message
};

struct gpsConfigEntry {
  const unsigned char *data;
  const unsigned char len;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
struct gpsData gpsData; // This is accessed by the parser functions directly !
struct gpsConfigEntry gpsConfigEntries[] = {UBLOX_CONFIGS};
GeodeticPosition currentPosition;

////////////////////////////////////////////////////////////////////////////////////////////////////

struct ublox_NAV_STATUS { // 01 03 (16)
  uint32_t iTow;
  uint8_t  gpsFix;
  uint8_t  flags;
  uint8_t  fixStat;
  uint8_t  flags2;
  uint32_t ttfx;
  uint32_t msss;
};

struct ublox_NAV_POSLLH { // 01 02 (28)
  uint32_t iTow;
  int32_t lon; // 1e-7 degrees
  int32_t lat; // 1e-7 degrees
  int32_t height; // mm
  int32_t hMSL; // mm
  uint32_t hAcc; //mm
  uint32_t vAcc; //mm
};

struct ublox_NAV_SOL { // 01 6 (52)
  uint32_t iTow;
  int32_t  fTow;
  int16_t  week;
  uint8_t  gspFix;
  uint8_t  flags;
  int32_t  ecefX;
  int32_t  ecefY;
  int32_t  ecefZ;
  int32_t  pAcc;
  int32_t  ecefVX;
  int32_t  ecefVY;
  int32_t  ecefVZ;
  int32_t  sAcc;
  uint16_t pDOP;
  uint8_t  res1;
  uint8_t  numSV;
  uint32_t res2;
};

struct ublox_NAV_VELNED { // 01 12h (36)
  uint32_t iTow;
  int32_t  velN; // cm/s
  int32_t  velE; // cm/s
  int32_t  velD; // cm/s
  uint32_t  speed; // cm/s
  uint32_t  gSpeed; // cm/s
  int32_t  heading; // dev 1e-5
  uint32_t sAcc; // cm/s
  uint32_t cAcc; // deg 1e-5
};

unsigned short ubloxClass,ubloxId;
unsigned char  ubloxCKA,ubloxCKB;
unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;

union ublox_message {
  struct ublox_NAV_STATUS nav_status;
  struct ublox_NAV_POSLLH nav_posllh;
  struct ublox_NAV_VELNED nav_velned;
  struct ublox_NAV_SOL nav_sol;
  unsigned char raw[52];
} ubloxMessage;

/////////////////////////////////////////////////////////////////////////////////////////////////////
GPS rec;

void GPS::ubloxParseData() {// uses publib vars

  gpsData.sentences++;
  if (ubloxClass==1) { // NAV
    if (ubloxId==2) { // NAV:POSLLH
      gpsData.lat = ubloxMessage.nav_posllh.lat;
      gpsData.lon = ubloxMessage.nav_posllh.lon;
      gpsData.height = ubloxMessage.nav_posllh.height;
      gpsData.accuracy = ubloxMessage.nav_posllh.hAcc;
      Serial.print("Accuracy: ");Serial.print(gpsData.accuracy);
      gpsData.fixtime = ubloxMessage.nav_posllh.iTow;
    }
    else if (ubloxId==3) { //NAV:STATUS
      switch (ubloxMessage.nav_status.gpsFix) {
        case 2: 
          gpsData.state = GPS_FIX2D;
          Serial.print(" 2D Fix ");
          break;
		  
        case 3:
          gpsData.state = GPS_FIX3D;
          Serial.print(" 3D Fix ");
          break;
		  
        default:
          gpsData.state = GPS_NOFIX;
          Serial.print(" No Fix ");
          break;
      }
    }
    else if (ubloxId==6) { // NAV:SOL
      gpsData.sats = ubloxMessage.nav_sol.numSV;
      Serial.print("NO. os Sats: "); Serial.print(gpsData.sats);
    }
    else if (ubloxId==18) { // NAV:VELNED
      gpsData.course = ubloxMessage.nav_velned.heading / 100; // 10E-5 to millidegrees
      gpsData.speed = ubloxMessage.nav_velned.gSpeed;
    }
  } 
}



void GPS::gpsSendConfig() {

  if (gpsConfigEntries[gpsConfigsSent].data) {
    if (gpsConfigEntries[gpsConfigsSent].len) {
      for (int i=0; i<gpsConfigEntries[gpsConfigsSent].len; i++) {
        Serial2.write(gpsConfigEntries[gpsConfigsSent].data[i]);
      }
      gpsConfigTimer=gpsConfigEntries[gpsConfigsSent].len;
    }
    else {
      Serial2.print((char*)gpsConfigEntries[gpsConfigsSent].data);
      gpsConfigTimer=strlen((char*)gpsConfigEntries[gpsConfigsSent].data);
    }
   
   if (gpsConfigTimer<10) {
      gpsConfigTimer=10;
    }
    gpsConfigsSent++;
  }

}

enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID,
GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState;

void GPS::ubloxInit() {
  
  ubloxProcessDataState = WAIT_SYNC1;
}


int GPS::ubloxProcessData(unsigned char data) {

  int parsed = 0;
  
  switch (ubloxProcessDataState) {
  case WAIT_SYNC1:
    if (data == 0xb5) {
      ubloxProcessDataState = WAIT_SYNC2;
    }
    break;
	
  case WAIT_SYNC2:
    if (data == 0x62) {
      ubloxProcessDataState = GET_CLASS;
    }
    else if (data == 0xb5) {
      // ubloxProcessDataState = GET_SYNC2;
    }
    else {
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
  case GET_CLASS:
    ubloxClass=data;
    ubloxCKA=data;
    ubloxCKB=data;
    ubloxProcessDataState = GET_ID;
    break;
	
  case GET_ID:
    ubloxId=data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LL;
    break;
	
  case GET_LL:
    ubloxExpectedDataLength = data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LH;
    break;
	
  case GET_LH:
    ubloxExpectedDataLength += data << 8;
    ubloxDataLength=0;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    if (ubloxExpectedDataLength <= sizeof(ubloxMessage)) {
      ubloxProcessDataState = GET_DATA;
    }
    else {
      // discard overlong message
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
	
  case GET_DATA:
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    // next will discard data if it exceeds our biggest known msg
    if (ubloxDataLength < sizeof(ubloxMessage)) {
      ubloxMessage.raw[ubloxDataLength++] = data;
    }
    if (ubloxDataLength >= ubloxExpectedDataLength) {
      ubloxProcessDataState = GET_CKA;
    }
    break;
	
  case GET_CKA:
    if (ubloxCKA != data) {
      ubloxProcessDataState = WAIT_SYNC1;
    } 
	else {
      ubloxProcessDataState = GET_CKB;
    }
    break;
	
  case GET_CKB:
    if (ubloxCKB == data) {
      parsed = 1;
      //Serial.print("parsed  ");Serial.println(parsed);
      rec.ubloxParseData();
    }
    ubloxProcessDataState = WAIT_SYNC1;
    break;
	
  }
  return parsed;
}

void GPS::updateGps() {
    
  gpsData.idlecount++;
   if(gpsData.idlecount == 200)    {
     //Serial.println("NO data");
     gpsData.idlecount = 0;
   }
  // Serial.println(Serial2.available());
  while (Serial2.available()) {
    unsigned char c = Serial2.read();
    int ret=0;

    if (gpsData.state == GPS_DETECTING) {
        ret = rec.ubloxProcessData(c);
        if (ret) {
          // found GPS device start sending configuration
          gpsConfigsSent = 0;
          gpsConfigTimer = 1;
          break;
        }
      
    }
    else {
      // Normal operation just execute the detected parser
      ret = rec.ubloxProcessData(c);
    }

  if (gpsConfigTimer) {
    if (gpsConfigTimer==1) {
      gpsSendConfig();
    }
    gpsConfigTimer--;
  }
    // Upon a successfully parsed sentence, zero the idlecounter and update position data
    if (ret) {
      if (gpsData.state == GPS_DETECTING) {
         gpsData.state = GPS_NOFIX; // make sure to lose detecting state (state may not have been updated by parser)
      }
      gpsData.idlecount=0;
      currentPosition.latitude=gpsData.lat;
      currentPosition.longitude=gpsData.lon;
      currentPosition.altitude=gpsData.height;
      Serial.print("Latitutde = ");Serial.print(currentPosition.latitude);Serial.print(" Longitude = ");Serial.print(currentPosition.longitude);Serial.print(" Altitude = ");Serial.println(currentPosition.altitude);
    }
  }

  // Schedule confg sending if needed
  if (gpsConfigTimer) {
    if (gpsConfigTimer==1) {
      gpsSendConfig();
    }
    gpsConfigTimer--;
  }

  }

void GPS::initializeGpsData() {

  gpsData.lat = GPS_INVALID_ANGLE;
  gpsData.lon = GPS_INVALID_ANGLE;
  gpsData.course = GPS_INVALID_ANGLE;
  gpsData.speed = GPS_INVALID_SPEED;
  gpsData.height = GPS_INVALID_ALTITUDE;
  gpsData.accuracy = GPS_INVALID_ACCURACY;
  gpsData.fixage = GPS_INVALID_AGE;
  gpsData.state = GPS_DETECTING;
  gpsData.sentences = 0;
  gpsData.sats = 0;
  gpsData.fixtime = 0xFFFFFFFF;
}

void GPS::initializeGps() {
    gpsData.baudrate = 2;
    Serial2.begin(gpsBaudRates[gpsData.baudrate]);
    rec.ubloxInit();
    rec.initializeGpsData();
 }
 
 #endif;

////////////////////////////////////////////////////////////////////////////////////////////////////


