// OV7670 library

/****************************************************************************
MIT License

Copyright (c) 2020  Mark J. Borgerson

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***************************************************************************/
#include "OV7670.h"
#include <Wire.h>


// return 5-bit red, green and blue values from RGB565 pixel
// Since these values are used in motion detection, the green
// value is reduced from 6 to 5 bits so that all colors have the
// same range and resolution
__inline int16_t Red(uint16_t pixval){
  return (pixval & 0xF800) >> 11;
}

__inline int16_t Grn(uint16_t pixval){
  return (pixval & 0x07E0) >> 6; 
}

__inline int16_t Blu(uint16_t pixval){
  return pixval & 0x001F;
}

 /*********************************************/
// This IRQ handler has to be defined outside the class
void CSI_IRQProxy(void) {
  OV7670.CSIService();
}

// instantiate the OV7670 object
clOV7670 OV7670;
/*************************************************************************
  Private methods---mostly CSI setup
************************************************************************/
/************************************************
  Set CSI internal CCM clock to 24 MHz or 12 MHz
  Max VGA Frame rate at 12MHz is 15FPS.
  At 24MHz, you can get 30 VGA FPS, but the system seems
  to fail to store the first byte of data, resulting in
  images that look like false-color thermal images.
* ************************************************/
const int camAddress = 0x21;
volatile uint16_t lastframeread = 2;
volatile uint32_t last_fc = 0;
uint32_t csisr_state = 0;

uint8_t *fcaptptr;
Stream *iptr = &Serial;    // information output stream


// These variables are set by the CSI interrupt handler
volatile uint32_t framecount, fb1count, fb2count, bswitcherrors;

void clOV7670::CSI_Start(void) {
  CSI_CSICR18 |= (CSI_CSICR18_CSI_ENABLE //CSI_ENABLE
                  | CSI_CSICR18_BASEADDR_SWITCH_EN    // CSI switches base addresses at frame start
                  | CSI_CSICR18_BASEADDR_CHANGE_ERROR_IE); //  Enables base address error interrupt
}

void clOV7670::CSI_Stop(void) {
  CSI_CSICR18 &= ~CSI_CSICR18_CSI_ENABLE; ///  clear the enable bit
}

// IRQ Handler for CSI Interrupt
void clOV7670::CSIService(void) {

  csisr_state =  CSI_CSISR;

  if (csisr_state & (CSI_CSISR_SOF_INT  )) {
    framecount += (CSI_CSICR3 >> 16) - last_fc;
    last_fc = framecount;
  }
  if (csisr_state & CSI_CSISR_DMA_TSF_DONE_FB2 ){
    fb2count++; // increment buffer 2 count
	lastframeread = 2;
	if(capture_running){ // return to config.fb2 from capture buffer
		capture_running = false;
		CSI_CSIDMASA_FB2 = (uint32_t)(config.fb2);		
	}
  }
  if (csisr_state & CSI_CSISR_DMA_TSF_DONE_FB1  ){
	fb1count++; // increment buffer 1 count'
	lastframeread = 1;
  }

  if (csisr_state & CSI_CSISR_BASEADDR_CHHANGE_ERROR ) {
	bswitcherrors++; // buffer switch before DMA done
  }
  CSI_CSISR = csisr_state; // clear all interrupts
}


// Initialize the clock for the CSI
void clOV7670::CSI_InitMCLK(tcsiclock csispeed) {

  CCM_CCGR2 &= (~0b1100);  // turn off csi clock

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0b100; // alt4
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = 0x18U;  // 50MHz speed, DSE_3
  // anything but 24MHz defaults to 12MHz for now
  if (csispeed == CSI24) {
    config.CSI_Frequency = 24.0;
    CCM_CSCDR3 = 0b0;  // set csi clock source and divide by 1 = 24 MHz
  } else {
    config.CSI_Frequency = 12.0;
    CCM_CSCDR3 = (1 << 11);  // set csi clock source and divide by 2 = 12 MHz
  }
  CCM_CCGR2 = (CCM_CCGR2 & (~0b1100)) | 0b1100;  // turn on csi clock

}


  // Clear the FIFOs--which can only be done when FCC is 0
void clOV7670::CSI_ClearFIFOs(void) {
  uint32_t creg, maskbits;

  creg = CSI_CSICR1;
  CSI_CSICR1 = (creg & ~CSI_CSICR1_FCC ); // clear FCC bit
  maskbits = (CSI_CSICR1_CLR_STATFIFO)  | (CSI_CSICR1_CLR_RXFIFO ); // bits high for clearing stat and rx FIFOs
  CSI_CSICR1 = creg & ( ~CSI_CSICR1_FCC  | maskbits); // keep FCC off and clear FIFOs
  // wait while clear completes
  while ((CSI_CSICR1 & maskbits) != 0) {}
  CSI_CSICR1 = creg;  // restore original status
}

/*******************************************************************
   Set up the CSI registers.  For now we assume RGB565 mode
   which uses 2 bytes per pixel and needs 61440 bytes per VGA frame.
   That means that VGA frame buffers have to be in the EXTMEM PSRAM.
********************************************************************/

void clOV7670::CSI_Setup(void) {
  // get configuration from config
  //uint16_t fwidth, uint16_t fheight, uint8_t *fb1, uint8_t *fb2
  uint32_t creg;
  NVIC_DISABLE_IRQ(IRQ_CSI);
  // set initial values in CSICR1
  CSI_CSICR1 |= (
                  CSI_CSICR1_SOF_POL  | // 1 for  VSYNC rising edge
                  CSI_CSICR1_FCC | // FCC_MASK
                  CSI_CSICR1_HSYNC_POL  | // HSYNC_POL 1 for active high
                  CSI_CSICR1_GCLK_MODE  | // 1= GCLK_MODE PCLK only valid when HSYNC high
                  CSI_CSICR1_REDGE );  // Use pixel clock rising edge

  // Set Image Parameters--width in bytes and height in rows
  // bytes in row goes into upper 16 bits, height in lower 16 bits.
  CSI_CSIIMAG_PARA = (2 * config.fwidth << 16) | config.fheight;

  CSI_CSICR3 =  CSI_CSICR3_FRMCNT_RST | // clear the frame counter
                CSI_CSICR3_DMA_REQ_EN_RFF;//  Enable RxFIFO DMA request
  CSI_CSICR2 |= CSI_CSICR2_DMA_BURST_TYPE_RFF(3);  // RxFIFO DMA_BURST_TYPE increment by 16

  CSI_CSICR18 = (CSI_CSICR18 & ~((CSI_CSICR18_MASK_OPTION(3)) | 
				CSI_CSICR18_MASK_OPTION(3)));  // CSI_CSICR18_MASK_OPTION(3)

  // CSI_CSICR2 skip count not needed as frame width matches buffer width
  CSI_CSIDMASA_FB1 = (uint32_t)(config.fb1);
  CSI_CSIDMASA_FB2 = (uint32_t)(config.fb2);
  // enable interrupt flags.  This needs to be done to read status, even if you
  // don't enable the CSI interrupt in the NVIC.   
  CSI_CSICR1 |= ( CSI_CSICR1_FB1_DMA_DONE_INTEN |  // FB1_DMA_DONE_INTEN
                  CSI_CSICR1_FB2_DMA_DONE_INTEN | // FB2_DMA_DONE_INTEN
                  CSI_CSICR1_SOF_INTEN   );  //  SOF Interrupt enable

  CSI_ClearFIFOs();

  // Reset the Receive data DMA hardware
  // Why they call it 'reflashing' is a mystery to me!
  CSI_CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF ;  // CSI_CSICR3_DMA_REFLASH_RFF_MASK;
  while (CSI_CSICR3 & CSI_CSICR3_DMA_REFLASH_RFF );  // wait until reflash done

  // first clear, then set the RxFIFO full level to 16 double words or 64 bytes
  // I think this level is chosen to match the FIFO size of the FlexSPI used for
  // the external PSRAM.
  CSI_CSICR3 = ((CSI_CSICR3 & ~CSI_CSICR3_RxFF_LEVEL(7) ) | 
				CSI_CSICR3_RxFF_LEVEL(2)) ; // RxFF_LEVEL 2 is 16 double words

  creg = CSI_CSISR;
  CSI_CSISR = creg;  // write back 1's to clear all interrupts

  attachInterruptVector(IRQ_CSI, &CSI_IRQProxy );
  NVIC_ENABLE_IRQ(IRQ_CSI);
}

/***************************************************/

// change the input pins on clocks to have hysteresis  MJB 10/24/2020
void clOV7670::CSI_InitIOMux(void) {
  // VSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 0x4U;
  IOMUXC_CSI_VSYNC_SELECT_INPUT = 0x1U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 = 0x00010000U;
  /* Alt VSYNC on pin 34 B1_13 ALT2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13 = 0x2U;
    IOMUXC_CSI_VSYNC_SELECT_INPUT = 0x2U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_13 = 0x0U; */
  // HSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 0x4U;
  IOMUXC_CSI_HSYNC_SELECT_INPUT = 0x1U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 = 0x00010000U;
  // DCLK or Pixel Clk
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 0x4U;
  IOMUXC_CSI_PIXCLK_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x00010000U;
  // D7-D0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 0x4U;
  IOMUXC_CSI_DATA09_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 0x4U;
  IOMUXC_CSI_DATA08_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = 0x4U;
  IOMUXC_CSI_DATA07_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = 0x4U;
  IOMUXC_CSI_DATA06_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 = 0x4U;
  IOMUXC_CSI_DATA05_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 = 0x4U;
  IOMUXC_CSI_DATA04_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x4U;
  IOMUXC_CSI_DATA03_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = 0x0U;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x4U;
  IOMUXC_CSI_DATA02_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = 0x0U;
}

#define PV(N) iptr->print(#N); iptr->print(":\t"); iptr->println(N, HEX);
void print_csi_registers() {

  PV(CSI_CSICR1)
  PV(CSI_CSICR2)
  PV(CSI_CSICR3)
  PV(CSI_CSISTATFIFO)
  PV(CSI_CSIRFIFO)
  PV(CSI_CSIRXCNT)
  PV(CSI_CSISR)
  PV(CSI_CSIDMASA_STATFIFO)
  PV(CSI_CSIDMATS_STATFIFO)
  PV(CSI_CSIDMASA_FB1)
  PV(CSI_CSIDMASA_FB2)
  PV(CSI_CSIFBUF_PARA)
  PV(CSI_CSIIMAG_PARA)
  PV(CSI_CSICR18)
  PV(CSI_CSICR19)
  PV(CSI_CSIRFIFO)
}

// Adjust fields of BMP file header to match image size
void clOV7670::SetFileHeader(timagetype itype){
  switch(itype){
    case VGA :
      fileheader.bfSize  = (640 * 480 * 2) + 66;
      fileheader.biWidth = 640;
      fileheader.biHeight = -480;
      fileheader.biSizeImage = (640 * 480 * 2);
      break;
    case QVGA :
      fileheader.bfSize  = (320 * 240 * 2) + 66;;
      fileheader.biWidth = 320;
      fileheader.biHeight = -240;
      fileheader.biSizeImage = (320 * 240 * 2);
      break;
    case QQVGA :
      fileheader.bfSize  = (160 * 120 * 2) + 66;;
      fileheader.biWidth = 160;
      fileheader.biHeight = -120;
      fileheader.biSizeImage = (160 * 120 * 2);
      break;
    case CUSTOM :

      break;
  }
}


// Set the indices of the pixel to sample for motion
// the sample grid looks like this:
//     *   *   *   *   *   *   *   *
//       *   *   *   *   *   *   *
//     *   *   *   *   *   *   *   *
//       *   *   *   *   *   *   *
//     *   *   *   *   *   *   *   *
//       *   *   *   *   *   *   *
//     *   *   *   *   *   *   *   *   etc.


void clOV7670::SetSamplePixIdx(void){
  uint16_t xmax,ymax, istep;
  uint16_t iy, xstart;
  uint32_t xv,yv;
  xmax = config.fwidth; 
  ymax = config.fheight; 
  istep = 40; // default to VGA spacing
  if(config.imtype == QVGA) istep = 20;
  if(config.imtype == QQVGA)istep = 10;
  uint32_t idx = 0;
  for(iy = 1; iy < ymax/istep; iy++){
    yv = iy*istep;
    xstart = istep*(2 - (iy&1)) ; // start at 20 or 40 for VGA
    for(xv = xstart; xv < xmax; xv+= istep*2){
      if(idx < MAXSAMPLEIDX) SamplePixIdx[idx] = yv*xmax+xv;
      //Serial.printf("IDX: %3u Y: %3u   X: %3u  pixidx: %5lu\n",idx, yv, xv, yv*xmax+xv);
      idx++;
    }
  }  
}

/****************************************************/

void ShowSample(int16_t*  vals){
  uint16_t i;
  for(i= 0; i<MAXSAMPLEIDX/2; i++){
    if((i%10)== 0) Serial.println();
    iptr->printf("%3d ", vals[i]);
  }
  iptr->println();
}
void ShowSampleDiff(int16_t* vals1, int16_t* vals2){
  uint16_t i;
  for(i= 0; i<MAXSAMPLEIDX/2; i++){
    if((i%10)== 0) Serial.println();
    iptr->printf("%3d ", vals1[i]-vals2[i]);
  }
  iptr->println();
}
/******************************************************
*  Collect 83 sample pixels data and convert to brightness
*  as sum of reg, green and blue components.
*  Note the presence of arm_dcache_delete, which is required
*  to make sure that the foreground access in this function
*  retrieves the data most recently transfered by the CSI
*  frame buffer DMA transfer.
**********************************************************/

void clOV7670::GetSample(int16_t* vals, uint8_t* buff){
  uint16_t i;
  uint16_t pixval, *bptr;
  int16_t sampval;
  bptr = (uint16_t *) buff;
  arm_dcache_delete(buff, ImageSize());
  bptr[0] = 0;
  //Serial.printf("bptr = %p\n", bptr);
  for(i=0; i<MAXSAMPLEIDX; i++){
    pixval = bptr[SamplePixIdx[i]];
    sampval = Red(pixval) + Grn(pixval) + Blu(pixval);
    vals[i] = sampval;
  }
}


/*************************************************************************
  Public methods
************************************************************************/


bool clOV7670::begin(timagetype itype, uint8_t *xfb1, uint8_t *xfb2) {
  config.fb1 = xfb1;
  config.fb2 = xfb2;
  struct regval_entry *rlist = vga_ov7670;

  CSI_Stop();
  
  // if the image type is VGA, then the buffers must be in EXTMEM,
  // which starts at 0x70000000.
  if(((uint32_t)xfb1 < 0x70000000) && (itype == VGA)) return false;
  
  switch (itype) {
    case VGA :
	  config.imtype = VGA;
      config.fwidth = 640;
      config.fheight = 480;
      rlist = vga_ov7670;
      break;
    case QVGA :
	  config.imtype = QVGA;
      config.fwidth = 320;
      config.fheight = 240;
      rlist = qvga_ov7670;
      break;
    case QQVGA :
      config.imtype = QQVGA;
      config.fwidth = 160;
      config.fheight = 120;
      rlist = qqvga_ov7670;
      break;
    case CUSTOM :

      break;

  };   // end of switch(itype)
  SetSamplePixIdx();
  SetFileHeader(itype);  // make sure that a .bmp file will have proper header

  CSI_InitIOMux(); // set up connections to IO Pins
  CSI_InitMCLK(config.CSI_ClockSelect);  
  CSI_Setup();
  SetCamClock(config.Cam_Frequency);  
  WriteRegList(ov7670_default_regs);
  delay(10);
  WriteRegList(rlist);// Resolution Specification List
  delay(10);
  WriteRegList(rgb565_ov7670);

  CSI_Start();
  delay(200);

  return true;
}

// Change image resolution without all the overhead of begin()
bool clOV7670::SetResolution(timagetype itype){

  struct regval_entry *rlist = vga_ov7670;
  CSI_Stop();
  // if the image type is VGA, then the buffers must be in EXTMEM,
  // which starts at 0x70000000.
  if(((uint32_t)config.fb1 < 0x70000000) && (itype == VGA)) return false;
  
  switch (itype) {
    case VGA :
	  config.imtype = VGA;
      config.fwidth = 640;
      config.fheight = 480;
      rlist = vga_ov7670;
      break;
    case QVGA :
	  config.imtype = QVGA;
      config.fwidth = 320;
      config.fheight = 240;
      rlist = qvga_ov7670;
      break;
    case QQVGA :
      config.imtype = QQVGA;
      config.fwidth = 160;
      config.fheight = 120;
      rlist = qqvga_ov7670;
      break;
    case CUSTOM :

      break;

  };   // end of switch(itype)
  SetSamplePixIdx();
  SetFileHeader(itype);  
  CSI_Setup();  // setup will configure buffers according to config structure
  WriteRegList(rlist);// Resolution Specification List
  delay(10);


  CSI_Start();
  delay(200);

  return true;
	
}


toutmode clOV7670::OutMode(void){
	return config.outmode;
}

// Set the camera output mode
void clOV7670::SetOutMode(toutmode mode){
	if(mode != config.outmode){
		if(mode == RGB565){
			WriteRegList(rgb565_ov7670);
		}		
		if(mode == YUV422){
			WriteRegList(yuv422_ov7670);	
			SetYUVOrder(0);  // Imagehost and tooJpeg expect YCbYCr format
		}				
	}
	config.outmode = mode;
}	

void clOV7670::SetYUVOrder(uint16_t yo){
	uint8_t reg3a,reg3d;
	reg3a = OV7670.ReadRegister(0x3a) & ~0x08;  // clear bit 3
    reg3d = OV7670.ReadRegister(0x3d) & ~0x01; // clear lsb
    if (yo & 0x02) reg3a |= 0x08 ; // set bit 3 in TSLB
    if (yo & 0x01) reg3d |= 0x01 ; // set bit 0 in COM12
    OV7670.WriteRegister(0x3A, reg3a);
    OV7670.WriteRegister(0x3D, reg3d);	
}
	 

// Get two samples of 83 pixels, separated by mdelay milliseconds
// Return the sum of differences in sampled pixel values.
uint16_t clOV7670::CheckMotion(uint16_t mdelay){
  uint16_t i,  msum;
  uint16_t readyframe;

  ClearFrameReady();
  do {
    readyframe = OV7670.FrameReady();
  } while (readyframe != 1);
  GetSample(pixvals1, config.fb1);

  delay(mdelay);
  ClearFrameReady();
  do {
    readyframe = OV7670.FrameReady();
  } while (readyframe != 2);
  GetSample(pixvals2, config.fb2);
  msum = 0; 
  // accumulate the sum of absolute values of the
  // difference in samples between the two sampled frames
  for(i= 0; i<MAXSAMPLEIDX; i++){
   msum+= abs(pixvals1[i]-pixvals2[i]);
  }
  return msum;
}


void clOV7670::SetCamClock(float camfreq) // Gets XCLK from config.CamFrequency
{ // Clock = external-clock * pll_mull(0,4,6 or 8 on bits 6 and 7) / (2*divider_5_lsb+1)
  float r, ratio, bestr;
  uint8_t reg;
  uint16_t bestpll, bestdiv, p, d;
  float const pll[3] = { 4.0f, 6.0f, 8.0f };
  ratio =  camfreq / config.Cam_Frequency; // this is what we want to get to... find closest value...

  bestpll = 0, bestdiv = 0; bestr = 1000.0;
  for (p = 0; p < 3; p++) {
    for (d = 0; d < 32; d++) {
      r = pll[p] / (2 * (d + 1));
      if (fabs(r - ratio) < fabs(ratio - bestr)) {
        bestr = r;
        bestpll = p + 1;
        bestdiv = d;
      }
    }
  }
  // CLKRC register: Prescaler divide by 31
  config.Cam_Frequency = camfreq;
  reg = ReadRegister(0x11); WriteRegister(0x11, (reg & 0b1000000) | bestdiv);
  // DBLV register: PLL = 6
  reg = ReadRegister(0x6B); WriteRegister(0x6B, (reg & 0b00111111) | (bestpll << 6));
//  Serial.printf( "cam clock %6.2f with pll=%d and div=%d\n", config.Cam_Frequency, bestpll, bestdiv);

}


// Gets Camera clock from config record
float clOV7670::GetCamClock(void){
  return config.Cam_Frequency; 
}



// Set CSI clock and update config record
void clOV7670::SetCSIClock(tcsiclock csispeed){
  CSI_InitMCLK(csispeed);
  config.CSI_ClockSelect = csispeed;
}

// Gets CSI clock from config record
float clOV7670::GetCSIClock(void){
  return config.CSI_Frequency; 
}



// Read a single uint8_t from address and return it as a uint8_t
uint8_t clOV7670::ReadRegister(uint8_t reg) {
	uint8_t data;
	Wire.beginTransmission(camAddress);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(camAddress, 1);
	data = Wire.read();
	Wire.endTransmission();
	return data;
}

// Writes a single uint8_t (data) into address
void clOV7670::WriteRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(camAddress);
  Wire.write(reg); Wire.write(data);
  Wire.endTransmission();
}


// simplified test for end of reglist is  reg_num == 0xFF
// This will have to change if we ever get a camera with
// 255 or more registers!
void clOV7670::WriteRegList(struct regval_entry *reglist) {
  uint16_t index = 0;
  uint8_t reg_addr, reg_val;
  while (reglist[index].reg_num != 0xFF) {
    reg_addr = reglist[index].reg_num;
    reg_val = reglist[index].value;
    WriteRegister(reg_addr, reg_val);
    delayMicroseconds(100);
    index++;
  }
}


void clOV7670::ShowCSIRegisters(void) {
  iptr->println("==== CSI ====\n");
  iptr->printf("CSI Clock:  %5.1f\n",config.CSI_Frequency);
  print_csi_registers();
  iptr->println();
}



void clOV7670::ShowCamConfig(void) {
  iptr->println("\n   ***** OV7670 Configuration *****");
  iptr->printf("CSI Frequency: %5.1fMHz   Camera  Frequency: %5.1fMHz\n",
                config.CSI_Frequency, config.Cam_Frequency);
  iptr->printf("Frame Width: %u   Frame Height: %u\n", config.fwidth, config.fheight);
  iptr->printf("fb1: %p  fb2: %p  \n",
                config.fb1, config.fb2);
  iptr->printf("fb1count: %lu   fb2count: %lu\n",fb1count,fb2count);
}


// return a pointer to the BMP header so main program can write .BMP files
uint8_t* clOV7670::GetHeaderPtr(void) {
  return (uint8_t *)&fileheader;
}

timagetype clOV7670::GetImageType(void){
		return config.imtype;
}

void clOV7670::ReadAll(uint8_t rvals[]) {
  for (int i = 0; i <= 0xC9; i ++) {
    rvals[i] = ReadRegister(i);
  }

}

void clOV7670::ShowAll(uint8_t rvals[]) {
  for (int i = 0; i <= 0xC9; i ++) {
    if ((i % 8) == 0)iptr->println();
    iptr->printf("%02X:%02X  ", i, rvals[i]);
  }
  iptr->println();
}


// return the number of bytes in the RGB565 bitmap
uint32_t clOV7670::ImageSize(void) {
  return (uint32_t)(config.fwidth) * config.fheight * 2;
}

void clOV7670::ClearFrameReady(void){
	lastframeread = 0;
}

// return the number of the last frame  finished
// after ClearFrameReady, the value stays zero until
// a new frame is finished.  This ensures that the
// main program has a full frame interval to copy
// or process the last frame read.
uint16_t clOV7670::FrameReady(void){
  return lastframeread;

}


// This function captures a frame by using a new buffer just for the 
// captured frame.  Subsequent frames collected by the CSI won't 
// interfere with the capture buffer.
void clOV7670::CaptureFrame(void *framebuff){
	capture_running = false;
	if((config.imtype == VGA) && ((uint32_t)framebuff < 0x70000000)){
			iptr->println("VGA Images must be located in EXTMEM!");
			return;
	}
	lastframeread = 0;
	fcaptptr = (uint8_t*)framebuff;
	while(lastframeread != 2){};  // wait until frame 2 is finished
	// the current buffer pointer for 2nd frame is in config.fb2
	CSI_CSIDMASA_FB2 = (uint32_t)framebuff;
	capture_running = true;
	// Completing the capture can take up to two frame intervals
} 

bool clOV7670::CaptureRunning(void){
	 if(!capture_running){
		if ((uint32_t)fcaptptr > 0x2020000) { // makes camera dma data visible
			arm_dcache_delete((void *)fcaptptr, ImageSize());
		}
		
	 }
	 return capture_running;
 }


void clOV7670::SetContrast(uint8_t cval){
	WriteRegister(REG_CONTRAS, cval);
}


void clOV7670::SetBrightness(uint8_t brval){
	WriteRegister(REG_BRIGHT, brval);
}	

// Compensate for greenish cast of LED lights
void clOV7670::SetLEDComp(bool comp){
		if(comp){
			WriteRegister(REG_GFIX, 0x05); // Set red and blue gain to 1.25
		} else {
			WriteRegister(REG_GFIX, 0x00); // Set all gains to 1.0
		}
}

void clOV7670::SetInfoStream(Stream *infoptr){
		iptr = infoptr;
}

