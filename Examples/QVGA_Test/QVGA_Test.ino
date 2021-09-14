/****************************************************

    QVGA  test program for OV7670 library

    Saves a QVGA image as a .bmp file on SD card

    requires T4.1  and saves frames in  DMAMEM
    which means that you do not need PSRAM
 ******************************************************** */

#include <OV7670.h>
#include <SD.h>

const int ledpin = 13;

//  Put all the buffers in DMAMEM
uint8_t fcaptbuff[320l * 240l * 2] DMAMEM;
uint8_t cbuff1[320l * 240l * 2] DMAMEM;
uint8_t cbuff2[320l * 240l * 2] DMAMEM;


const char compileTime [] = " Compiled on " __DATE__ " " __TIME__;

#define SD_CONFIG SdioConfig(DMA_SDIO)

#define  LEDON digitalWriteFast(ledpin, HIGH); // Also marks IRQ handler timing
#define  LEDOFF digitalWriteFast(ledpin, LOW);

tBMPHDR565 fileheader;
File bmpfile;
const int pinCamReset = 14;

void BlinkForever(void) {
  while (1) {
    LEDON; delay(100);
    LEDOFF; delay(100);
  }
}

void CMSI(void) {
  Serial.printf("\n\nOV7670 Camera  QVGA Test 3 %s\n", compileTime);
  OV7670.ShowCamConfig();
}


void CMCS(void) {  // Show CSI registers
  OV7670.ShowCSIRegisters();
}


// Show Camera Registers
void CMCR(void) {
  uint8_t regs[200];
  OV7670.ReadAll(regs);
  OV7670.ShowAll(regs);
}

void CMGF(void) {
  uint32_t imagesize, hdrsize;

  imagesize = OV7670.ImageSize();

  hdrsize = sizeof(tBMPHDR565);
  Serial.printf("BMP Hdr is %lu bytes at %p \n", hdrsize, OV7670.GetHeaderPtr());
  Serial.printf("Ready to save %lu bytes.\n", imagesize);
  Serial.println("Saving OV7670.bmp ");

  if (SD.exists("OV7670.bmp")) {
    // delete the file:
    Serial.println("Removing old  OV7670.bmp...");
    SD.remove("OV7670.bmp");
  }
  Serial.println("Opening File\n");
  bmpfile = SD.open("OV7670.bmp", FILE_WRITE);

  if (bmpfile) {
    Serial.print("Writing...");
  } else {
    Serial.println("Could not open file.");
    return;
  }

  delay(10);
  bmpfile.write((const uint8_t *)OV7670.GetHeaderPtr(), 66);
  OV7670.ClearFrameReady();
  // wait until cbuff2 is ready
  while(OV7670.FrameReady() != 2){}
  // copy the image to the capture buffer
  memcpy(fcaptbuff, cbuff2, imagesize);
  bmpfile.write(cbuff2, imagesize);
  delay(5);
  Serial.println("Write complete");

  delay(5);
  bmpfile.close();
  Serial.printf("bmpfile closed \n");
  Serial.println("File saved to SD card.");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  pinMode(pinCamReset, OUTPUT);
  pinMode(ledpin, OUTPUT);

  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB

  Serial.print("\n\nInitializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    BlinkForever();
  }
  Serial.println("initialization done.");

  if (OV7670.begin(QVGA, cbuff1, cbuff2)) {
    Serial.println("OV7670 camera initialized.");
    Serial.printf("cbuff1 at   %p\n", cbuff1);
    Serial.printf("cbuff2 at    %p\n", cbuff2);
    Serial.printf("fcaptbuff at %p\n", fcaptbuff);
  } else {
    Serial.println("Error initializing OV7670");
    BlinkForever();
  }
  CMSI();

  LEDOFF
}

void loop() {

  char ch;
  if (Serial.available()) {
    ch = Serial.read();
    if (ch == 's') CMSI();
    if (ch == 'c') CMCS();
    if (ch == 'r') CMCR();
    if (ch == 'f') CMGF();
  }

}
