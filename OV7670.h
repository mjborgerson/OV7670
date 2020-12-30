/***************************************************************
* OV7670 Library    10/28/2020   MJB
* 
* Collects data from camera in user-specified frame buffers
* This code works only on the T4.1 and uses the CMOS Sensor
* Interface (CSI).  It requires that the camera use
* a special set of IO pins that can be connected to the CSI 
* with IOMux assignments.
*****************************************************************/


#ifndef  OV7670_H
#define OV7670_H


//#include <SdFat.h>
#include <stdint.h>
#include <Wire.h>
#include <stdio.h>
#include <TimeLib.h>

#include "OV7670_RegDefs.h"


#ifdef __cplusplus
extern "C" {
#endif

/*********************************************
*  OV7670 Register Setup Lists
*  Actual values in OV7670_Reglists.cpp
***********************************************/
extern struct regval_entry vga_ov7670[];
extern struct regval_entry qvga_ov7670[];
extern struct regval_entry qqvga_ov7670[];
extern struct regval_entry rgb565_ov7670[];
extern struct regval_entry ov7670_default_regs[];
extern struct regval_entry yuv422_ov7670[];


// define the bits in the CSISR status register
#define BIT_SOF_INT 16
#define BIT_EOF_INT 17
#define OV7670_VERSION 1.00



enum timagetype  {CUSTOM, QQVGA, QVGA, VGA};
enum tcsiclock {CSI12,CSI24};
enum toutmode {RGB565, YUV422};



typedef struct TCamConfig{
	float CSI_Frequency = 12.0;
	float Cam_Frequency = 12.0;
	uint16_t fwidth;
	uint16_t fheight;
	uint8_t  *fb1;		// pointer to frame buffer 1
	uint8_t  *fb2;		// pointer to frame buffer 2
	timagetype imtype;
	toutmode outmode = RGB565;
	tcsiclock CSI_ClockSelect = CSI12;
} CamConfig;

// this file header structure needs to be packed because of the first uint16_t being
// followed by a uint32_t.  Without packing you get 2 0x00 bytes after the bfType.
// This form of default value initialization only works after C Version 11
// This default is set up for VGA RGB565 bitmaps
struct  tBMPHDR565{
  uint16_t  bfType = 0x4d42;   //'bm';
  uint32_t  bfSize = 614466;// 614400 pixels + 66 header 
  uint16_t  bfReserved1 = 0;
  uint16_t  bfReserved2 = 0;
  uint32_t  bfOffBits =  66; // 14 bytes to here
  uint32_t  biSize = 40;
  int32_t   biWidth = 640;
  int32_t   biHeight = -480;  // Windows wants negative for top-down image
  int16_t   biPlanes = 1;
  uint16_t  biBitCount = 16 ;
  uint32_t  biCompression = 3;  // bitfields used needs color masks
  uint32_t  biSizeImage = 614400;  // 640 * 480 * 2
  int32_t   biXPelsPerMeter = 0;
  int32_t   biYPelsPerMeter = 0;
  uint32_t  biClrUsed  = 0;
  uint32_t  biClrImportant = 0;// 54 bytes
  uint32_t  rmask = 0x0000F800;
  uint32_t  gmask = 0x000007E0; 
  uint32_t  bmask = 0x0000001F;  //66 bytes
}__attribute__((__packed__));

#define MAXSAMPLEIDX 84

class clOV7670
{
	protected:
	private:
	int16_t pixvals1[MAXSAMPLEIDX], pixvals2[MAXSAMPLEIDX];
	uint32_t SamplePixIdx[MAXSAMPLEIDX];
	bool capture_running = false;
	tBMPHDR565 fileheader;
	CamConfig config;

	void 	CSI_Stop(void);
	void 	CSI_Start(void);
	void 	CSI_InitMCLK(tcsiclock csispeed) ;
	void 	CSI_ClearFIFOs(void);
	void 	CSI_Setup(void);
	void 	CSI_InitIOMux(void);
	void 	SetFileHeader(timagetype itype);
	void 	GetSample(int16_t* vals, uint8_t* buff);
	void 	SetSamplePixIdx(void);
 
	public:
	bool 	begin(timagetype itype, uint8_t *fb1, uint8_t *fb2);
	bool 	SetResolution(timagetype itype);
	toutmode	OutMode(void);
	void	SetOutMode(toutmode mode);
	void	SetYUVOrder(uint16_t yo);
	uint16_t CheckMotion(uint16_t mdelay);	
	void 	SetCamClock(float camfreq);
	float 	GetCamClock(void);	
	void  	SetCSIClock(tcsiclock);
	float 	GetCSIClock(void);
	
	uint8_t ReadRegister(uint8_t reg);
	void 	WriteRegister(uint8_t reg, uint8_t data);
	void 	WriteRegList(struct regval_entry *reglist); 
	void 	ReadAll(uint8_t rvals[]);
	void 	ShowAll(uint8_t rvals[]);

	void 	ShowCSIRegisters(void);
	void 	ShowCamConfig(void);
	uint8_t *GetHeaderPtr(void);
	timagetype 	GetImageType(void);
	uint32_t 	ImageSize(void);
	void 	CSIService(void);
	void 	ClearFrameReady(void);
	uint16_t FrameReady(void);
	void 	CaptureFrame(void *framebuff);
	bool 	CaptureRunning(void);
	void 	SetContrast(uint8_t cval);
	void 	SetBrightness(uint8_t brval);
	void 	SetLEDComp(bool comp);
	void	SetInfoStream(Stream *infoptr);

};

extern clOV7670 OV7670;

#ifdef __cplusplus
}
#endif


#endif // OV7670.h
