/******************************************************
 * Capture QVGA .bmp files on detection of motion
 * in camera field.  Optional use of ILI9341 display
 * and MTP file transfer
 */
#define USEILI9341
#define USEMTP

#ifdef USEMTP
#include <MTP.h>
#include <Storage.h>
#include <usb1_mtp.h>
#endif
#include "OV7670.h"
#include "SD.h"
#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <stdio.h>
#include <cstdlib>


#ifdef USEILI9341
#include <ILI9341_t3n.h>
#include "ili9341_t3n_font_Arial.h"
//Specify the pins used for Non-SPI functions
#define TFT_CS   10  // AD_B0_02
#define TFT_DC   9  // AD_B0_03
#define TFT_RST  8
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
#endif


File  bmpfile;

#define SD_CONFIG SdioConfig(FIFO_SDIO)
const char compileTime [] = " Compiled on " __DATE__ " " __TIME__;

const int ledpin = 13;
const int imarkpin = 32;
const int pinCamReset = 14;

#define  LEDON digitalWriteFast(ledpin, HIGH);
#define LEDOFF  digitalWriteFast(ledpin, LOW);

#ifdef USEMTP
MTPStorage_SD storage;
MTPD       mtpd(&storage);
#endif

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define FRAMEBYTES (FRAME_WIDTH * FRAME_HEIGHT *2)

uint8_t fb1[FRAMEBYTES] DMAMEM;
uint8_t fb2[FRAMEBYTES] DMAMEM;
uint8_t fcaptbuff[FRAMEBYTES];  // this one goes in regular memory


elapsedMillis autoMillis;
void setup() {
  Serial.begin(9600);
  delay(400);
  Wire.begin();

  pinMode(imarkpin, OUTPUT);
  pinMode(pinCamReset, OUTPUT);
  Serial.print("Initializing SD card...");

  if (!StartSDCard()) {
    Serial.println("initialization failed!");
    while (1) {
      pinMode(ledpin, OUTPUT);
      LEDON; delay(100);
      LEDOFF; delay(100);
    }
  }
  Serial.println("initialization done.");

  delay(100);
#ifdef USEMTP
  Serial.println("Starting MTP Responder");
  usb_mtp_configure();
  // NOTE: Next line requires the new SD implementation in TD 1.54B4
  if (!Storage_init(&SD.sdfs)) {
    Serial.println("Could not initialize MTP Storage!");
  }
#endif

#ifdef USEILI9341
  // Start ILI9341
  tft.begin();
  tft.setRotation(3);  // Need rot = 3 for right-side up on MJB proto board
#endif

  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB
  Serial.printf("\n\nOV7670 Camera Motion detection %s\n", compileTime);

  OV7670.begin(QVGA, fb1, fb2);
  OV7670.SetCamClock(24);
  setSyncProvider(getTeensy3Time); // helps put time into file directory data
  autoMillis = 0;
}


#define AUTODELAY 120000  // 2-minute autostart delay
bool mdflag = false;
void loop() {
  char ch;
  if(autoMillis > AUTODELAY){
    CMMD(); 
  }
  if(mdflag) autoMillis = 0;  // reset the autostart timer if already detecting motion
  if (Serial.available()) {
    ch = Serial.read();
    autoMillis = 0;  // reset the autostart timer if user presses a key
    if(mdflag){
      Serial.println("Motion detection halted.");
      mdflag = false;
    }

    if (ch == 's') CMSI();
#ifdef USEMTP
    if (ch == 'b') CMRB();
#endif
    if (ch == 'm') CMMD();
    if (ch == 'f') CMGF();
    if (ch == 'd') CMDI();
  }
#ifdef USEMTP
  mtpd.loop();
#endif
  if(mdflag) CheckMotionDetect();
}

#define THRESHHOLD 300
#define MDELAY 50
#define FRAMEDELAY 3000  // wait three seconds between captured frames
void CheckMotionDetect(void){
  uint16_t mval;
  mval = OV7670.CheckMotion(MDELAY);

  if(mval > THRESHHOLD){
   Serial.printf("CheckMotion returned %u\n",mval);   
    CMGF();  // save next frame
    delay(FRAMEDELAY);
  } 
}


#ifdef USEMTP
// This command will disconnect and reconnect the USB link.  This
// will cause the PC to restart MTP and request a rebuild of the 
// MTP directory.
void CMRB(void){  
  LEDON
  Serial.println("Reconnect serial port or restart Serial Monitor after USB reset.");
  delay(100);
  usb_init();  // shuts down USB if already started, then restarts
  delay(200);
  Serial.begin(9600);
  delay(200);
  LEDOFF

  usb_mtp_configure();
  // NOTE: Next line requires the new SD implementation in TD 1.54B4
  if (!Storage_init(&SD.sdfs)) {
    Serial.println("Could not initialize MTP Storage!");
  }
  Serial.println("USB disconnected and reconnected to force MTP update");
}
#endif

void CMSI(void) {

  Serial.printf("\n\nOV7670 Motion Detection 3 %s\n", compileTime);
  Serial.printf("buff1 at %p   buff2 at %p\n", &fb1, &fb2);

}


void CMDI(void){
  Serial.println("SD Card Directory");
  SD.sdfs.ls(LS_DATE | LS_SIZE);
  Serial.println();
}

// Start Motion Detection
void CMMD(void){
  Serial.println("Starting Motion Detection.");
  Serial.println("Press any key to halt motion detection");
  mdflag = true;
}



// Get a frame of data and write to .bmp file
void CMGF(void) {
  uint32_t imagesize;
  uint16_t buffnum;
  imagesize = OV7670.ImageSize();
  char * fname;

  fname = NewFileName();
  Serial.printf("Saving %s\n",fname);
  if (SD.exists(fname)) {
    // delete the file:
    Serial.printf("Removing %s...", fname);
    SD.remove(fname);
  }
  bmpfile = SD.open(fname, FILE_WRITE);
  bmpfile.write((const uint8_t *)OV7670.GetHeaderPtr(), sizeof(tBMPHDR565));
  OV7670.ClearFrameReady();
  // wait until buffer 2 has just been filled
  do {
    buffnum = OV7670.FrameReady();
  } while (buffnum != 2);
  bmpfile.write(fb2, imagesize);
  bmpfile.close();
  #ifdef USEILI9341
  ILIShowFrame();
  #endif
}


/*****************************************************************************
   Read the Teensy RTC and return a time_t (Unix Seconds) value

 ******************************************************************************/
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

//------------------------------------------------------------------------------
/*
   User provided date time callback function.
   See SdFile::dateTimeCallback() for usage.
*/
void dateTime(uint16_t* date, uint16_t* time) {
  // use the year(), month() day() etc. functions from timelib

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

bool StartSDCard() {

  if (!SD.begin(254)) {
    Serial.println("\nSdFs File initialization failed.\n");
    return false;
  } else  Serial.println("initialization done.");


  // set date time callback function
  SdFile::dateTimeCallback(dateTime);
  return true;
}

char* NewFileName(void) {
  static char fname[36];
  time_t nn;
  nn = now();
  int mo = month(nn);
  int dd = day(nn);
  int hh = hour(nn);
  int mn = minute(nn);
  int ss = second(nn)/4 ;
  char secchar = 'A' + ss;
  sprintf(fname, "MD_%02d%02d%02d%02d%c.bmp", mo, dd, hh, mn, secchar);

  return &fname[0];
}


#ifdef USEILI9341
// Generate a time stamp string for ILI9341
void MakeTimeStamp(char *str) {
  time_t nn;
  nn = now();
  int yr = year(nn);
  int mo = month(nn);
  int dd = day(nn);
  int hh = hour(nn);
  int mn = minute(nn);
  int se = second(nn);
  sprintf(str, "%02d/%02d/%04d  %02d:%02d:%02d", mo, dd, yr, hh, mn, se);

}

void ILIShowFrame(void ) {
  static char tmsg[32] = "01/01/2020 01:00:00";
  MakeTimeStamp(tmsg);
  memcpy(fcaptbuff, fb2, OV7670.ImageSize());
  uint32_t *tsptr = (uint32_t *)&fb2 ;
  // put the time stamp into first 4 bytes of image
  *tsptr = now();

  tft.setFont(Arial_12);
  tft.writeRect(0, 0, tft.width(), tft.height(), (uint16_t *)fcaptbuff);
  tft.setTextColor(ILI9341_GREEN);
  tft.drawString(tmsg, 5, 226);

}

#endif
