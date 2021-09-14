/****************************************************
 * 
 *Simple test program for OV7670 library
 *
 *  Saves a VGA image as a .bmp file on SD card
 *  
 *  requires T4.1 with PSRAM as VGA image is 614,400 bytes
 *  which is too large for onboard RAM
 ******************************************************** */

#include <OV7670.h>
#include <SD.h>

const int ledpin = 13;

//  Put all the buffers in EXTMEM  (PSRAM)
uint8_t fcaptbuff[640l * 480l * 2] EXTMEM;
uint8_t cbuff1[640l * 480l * 2] EXTMEM;
uint8_t cbuff2[640l * 480l * 2] EXTMEM;


const char compileTime [] = " Compiled on " __DATE__ " " __TIME__;

#define SD_CONFIG SdioConfig(DMA_SDIO)

#define  LEDON digitalWriteFast(ledpin, HIGH); // Also marks IRQ handler timing
#define  LEDOFF digitalWriteFast(ledpin, LOW);

tBMPHDR565 fileheader;
File bmpfile;
const int pinCamReset = 14;

void CMSI(void) {
  Serial.printf("\n\nOV7670 Camera  VGA Test 3 %s\n", compileTime);
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
  Serial.printf("bmpfile: %p\n", bmpfile);
  if (bmpfile) {
    Serial.println("Writing...");
  } else {
    Serial.println("Could not open file.");
    return;
  }

  delay(10);
  bmpfile.write((const uint8_t *)OV7670.GetHeaderPtr(), 66);
  bmpfile.write(cbuff2, imagesize);

  Serial.printf("bmpfile: %p\n", bmpfile);
  delay(5);
  Serial.println("Write complete");
  Serial.printf("bmpfile: %p\n", bmpfile);
  delay(5);
  bmpfile.close();
  Serial.printf("bmpfile: %p  closed \n", bmpfile);
  Serial.println("File saved to SD card.");
}

void setup() {
  Serial.begin(9600);
  delay(200);
  Wire.begin();

  pinMode(pinCamReset, OUTPUT);
  pinMode(ledpin, OUTPUT);

  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB

  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1){
        LEDON; delay(100);
        LEDOFF; delay(100);
      }
  }
  Serial.println("initialization done.");

  if (OV7670.begin(VGA, cbuff1, cbuff2)) {
    Serial.println("OV7670 camera initialized.");
    Serial.printf("cbuff1 at   %p\n", cbuff1);
    Serial.printf("cbuff2 at    %p\n", cbuff2);
    Serial.printf("fcaptbuff at %p\n", fcaptbuff);
  }
  OV7670.ShowCamConfig();

  LEDOFF
}

void loop() {
  // put your main code here, to run repeatedly:

  char ch;
  if (Serial.available()) {
    ch = Serial.read();
    if (ch == 's') CMSI();
    if (ch == 'c') CMCS();
    if (ch == 'r') CMCR();
    if (ch == 'f') CMGF();
  }

}
