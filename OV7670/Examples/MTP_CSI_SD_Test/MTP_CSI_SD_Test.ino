
/********************************************
 * Simple test to save VGA images to .bmp files
 * A new file name is generate each  second.
 * 
 * Requires setup of modified MTP library where
 * an initialized file system is an input to Storeage_init()
 * 
 * NOTE: This file was built with TeensyDuino 1.54B4
 *       which has SD implemented as a wrapper to
 *       SDFat 2.0.
 */

#include <MTP.h>
#include <Storage.h>
#include <usb1_mtp.h>
#include <TimeLib.h>
#include <stdint.h>
#include <Wire.h>
#include "OV7670.h"
#include <SD.h>


#define FRAME_WIDTH 640l
#define FRAME_HEIGHT 480l

#define FB_WIDTH 640l
#define FB_HEIGHT 480l
#define FB_COUNT 2  // two frames in EXTMEM

#define FRAMEBYTES (FRAME_WIDTH * FRAME_HEIGHT *2)
#define FRAMEBUFFBYTES (FB_WIDTH *FB_HEIGHT *2)

uint8_t fb1[FRAMEBUFFBYTES] EXTMEM;
uint8_t fb2[FRAMEBUFFBYTES] EXTMEM;


File  bmpfile;

#define SD_CONFIG SdioConfig(FIFO_SDIO)
const char compileTime [] = " Compiled on " __DATE__ " " __TIME__;

const int ledpin    = 13;
const int imarkpin = 32;
const int pinCamReset = 14;

#define  LEDON digitalWriteFast(ledpin, HIGH);
#define LEDOFF  digitalWriteFast(ledpin, LOW);



MTPStorage_SD storage;
MTPD       mtpd(&storage);


#define  IMARKHI digitalWriteFast(imarkpin, HIGH); // Also marks IRQ handler timing
#define  IMARKLO digitalWriteFast(imarkpin, LOW);
#define  IMARKTOGGLE digitalToggleFast(imarkpin);

void setup() {
  Serial.begin(9600);
  delay(200);
  Wire.begin();

  pinMode(ledpin, OUTPUT);
  pinMode(imarkpin, OUTPUT);
  pinMode(pinCamReset, OUTPUT);
  Serial.print("Initializing SD card...");

  if (!StartSDCard()) {
    Serial.println("initialization failed!");
    while (1) {
      LEDON; delay(100);
      LEDOFF; delay(100);
    }
  }
  Serial.println("initialization done.");

  delay(100);

  StartMTP();
  memset(&fb1, 0, FRAMEBUFFBYTES * 2);

  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB

  Serial.printf("\n\nOV7670 Camera MTP Test %s\n", compileTime);

  OV7670.begin(VGA, fb1, fb2);

  setSyncProvider(getTeensy3Time); // helps put time into file directory data

}

void loop() {

  char ch;
  if (Serial.available()) {
    ch = Serial.read();
    if (ch == 's') CMSI();
    if (ch == 'c') CMCS();
    if (ch == 'b') CMRB();
    if (ch == 'r') CMCR();
    if (ch == 'f') CMGF();
    if (ch == 'd') CMDI();
  }
  mtpd.loop();
}

void ShowDirectory(uint16_t dtype) {
 // SD.ls(dtype);

}

//SCB_AIRCR = 0x05FA0004;// software reset

void StartMTP(void) {
  Serial.println("Starting MTP Responder");
  usb_mtp_configure();
  // NOTE: Next line requires the new SD implementation in TD 1.54B4
  if (!Storage_init(&SD.sdfs)) {
    Serial.println("Could not initialize MTP Storage!");
    fastBlink();
  }
}
// blink at 500Hz forever to signal unrecoverable error
void fastBlink(void) {
  while (1) {
    LEDON
    delay(100);
    LEDOFF
    delay(100);
  }
}


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

void ShowHeader(void) {
  uint16_t i;
  uint8_t *hp;
  Serial.println("BMPFile header: ");
  hp = (uint8_t *)OV7670.GetHeaderPtr();
  for (i = 0; i < sizeof(tBMPHDR565); i++) {
    if ((i % 16) == 0) Serial.println();
    Serial.printf("%02X ", *hp);
    hp++;
  }
  Serial.println();
}

// Show information
void CMSI(void) {

  Serial.printf("\n\nOV7670 Camera Test 3 %s\n", compileTime);
  Serial.printf("buff1 at %p   buff2 at %p\n", &fb1, &fb2);

}

void CMCS(void) {  // Show CSI registers
  OV7670.ShowCSIRegisters();
}

void CMDI(void){
  Serial.println("SD Card Directory");
  SD.sdfs.ls(LS_DATE | LS_SIZE);
  Serial.println();
}

// Show Camera Registers
void CMCR(void) {
  uint8_t regs[200];
  OV7670.ReadAll(regs);
  OV7670.ShowAll(regs);
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
  Serial.println("File saved to SD card.");
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
  sprintf(fname, "BMP__%02d%02d%02d%02d%c.bmp", mo, dd, hh, mn, secchar);

  return &fname[0];
}
