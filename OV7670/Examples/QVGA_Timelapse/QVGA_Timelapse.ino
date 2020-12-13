/****************************************************

    QVGA and ILI9341 test program for OV7670 library

    Transfers a QVGA image to ILI9341 TFT screen
    Capable of 15.0 Frames per second

    This code takes advantage of the fact that the
    OV7670 in QVGA mode matches the 320x240 pixels
    of the ILI9341

    requires T4.1  and saves frames in  DMAMEM
    which means that you do not need PSRAM
 ******************************************************** */

#include <OV7670.h>
#include <ILI9341_t3n.h>
#include <SD.h>
#include <TimeLib.h>

//Specify the pins used for Non-SPI functions
#define TFT_CS   10  // AD_B0_02
#define TFT_DC   9  // AD_B0_03
#define TFT_RST  8


ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);

const int imarkpin = 32;  // used for o'scope frame rate timing

#define  IMARKHI digitalWriteFast(imarkpin, HIGH); // Also marks IRQ handler timing
#define  IMARKLO digitalWriteFast(imarkpin, LOW);
#define  IMARKTOGGLE digitalToggleFast(imarkpin);

#define AUTODELAY 120000

//  Put all the buffers in DMAMEM
uint8_t image[320l * 240l * 2] DMAMEM;
uint8_t cbuff1[320l * 240l * 2] DMAMEM;
uint8_t cbuff2[320l * 240l * 2] DMAMEM;

uint16_t frameinterval = 2;  // default to 1 frame per 2 seconds
const char compileTime [] = " Compiled on " __DATE__ " " __TIME__;

File tlFile;
char tlFileName[64];  // save name of most recent file
uint32_t framescollected;

const int pinCamReset = 14;

elapsedMillis autoMillis;

void setup() {
  Serial.begin(9600);
  delay(200);
  Wire.begin();

  pinMode(pinCamReset, OUTPUT);
  pinMode(imarkpin, OUTPUT);

  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB

  if (!StartSDCard()) {
    Serial.println("initialization failed!");
  }

  if (OV7670.begin(QVGA, cbuff1, cbuff2)) {
    Serial.println("OV7670 camera initialized.");
    Serial.printf("cbuff1 at   %p\n", cbuff1);
    Serial.printf("cbuff2 at    %p\n", cbuff2);
  } else {
    Serial.println("Error initializing OV7670");
  }
  setSyncProvider(getTeensy3Time); // helps put time into file directory data

  OV7670.SetCamClock(12);
  // Start ILI9341
  tft.begin();
  tft.setRotation(3);  // Need rot = 3 for right-side up on MJB proto board

  CMSI();
  autoMillis = 0;
}

static bool collecting = false;
void loop() {
  // Choices:  's' System Info  'q' stop time laps collection  't' Start time-lapse capture
  //           'd' Show disk directory  
  //            '1', '2', '4', 8'8' : set interval between time-lapse captures
  char ch;

  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
      case '1' :
      case '2' :
      case '4' :
      case '8' :
        frameinterval = ch - '0';
        Serial.printf("Time Lapse frame interval set to %d seconds.\n", frameinterval);
        break;
      case 's' :  CMSI();
        break;
      case 't' :
        CMTL(); // gets a file name and opens the file for writing
        break;
      case 'd' :
        Serial.println("\nSD Card Directory");
        SD.sdfs.ls(LS_DATE | LS_SIZE);
        Serial.println();
        break;
      case 'q' :
        collecting = false;
        if (tlFile) tlFile.close();
        break;
    }
    
  }
  if (collecting){
    CheckCollection();
    autoMillis = 0;
  } else {
    if(autoMillis > AUTODELAY) CMTL();
  }
}

elapsedMillis collectionMillis;

void CheckCollection(void) {
    if (collectionMillis > frameinterval * 1000) {
      collectionMillis = 0;  // reset the timer
      framescollected++;
      CMGF(); // get frame into image buffer
      // put time in first 4 bytes of image  
      uint32_t *tsptr = (uint32_t *)&image;
      *tsptr = now();
      tlFile.write(image, OV7670.ImageSize());
      tlFile.flush();
    }
  }

  void CMSI(void) {
    Serial.printf("\n\nOV7670 Camera and ILI9341 Time Lapse Test 3 %s\n", compileTime);
    OV7670.ShowCamConfig();
    if (tlFile) {
      Serial.printf("Collecting frames at %u second intervals.\n", frameinterval);
      Serial.printf("%lu frames in file.\n", framescollected);
    }
  }


  // Get a file name and open the file
  void CMTL(void) {
    Serial.println("Press 'q' to halt collection");
    framescollected = 0;
    strcpy(tlFileName, NewFileName());
    tlFile = SD.open(tlFileName, FILE_WRITE);
    if (tlFile) {
      Serial.printf("File <%s> open for collection.\n", tlFileName);
      collecting = true;
    } else {
      Serial.printf("Could not open file <%s>. \n", tlFileName);
      collecting = false;
    }
  }

  // Capture and display a single frame from OV7670
  void CMGF(void) {
    uint16_t readyframe;
    uint32_t imagesize;
    imagesize = OV7670.ImageSize();
    uint16_t lastframe = OV7670.FrameReady();
    do {
      readyframe = OV7670.FrameReady();
    } while (readyframe == lastframe); // wait until a frame just completed
    // copy the frame to image buffer to prevent tearing and glitches
    if (readyframe == 2) memcpy(image, cbuff2, imagesize);
    if (readyframe == 1) memcpy(image, cbuff1, imagesize);

    // display the frame.  Seems simple---until you look behind the curtain!
    // That's where KurtE does the hard work!
    IMARKHI
    tft.writeRect(0, 0, tft.width(), tft.height(), (uint16_t *)&image);
    // This takes about 42mSec
    IMARKLO
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

    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("\nSD File initialization failed.\n");
      return false;
    } else  Serial.println("initialization done.");

    // set date time callback function
    SdFile::dateTimeCallback(dateTime);
    return true;
  }

  char* NewFileName(void) {
    static char fname[32];
    time_t nn;
    nn = now();
    int mo = month(nn);
    int dd = day(nn);
    int hh = hour(nn);
    int mn = minute(nn);

    sprintf(fname, "TL_%02d%02d%02d%02d.tld", mo, dd, hh, mn);
    return &fname[0];
  }
