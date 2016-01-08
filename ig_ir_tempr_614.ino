#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>
// mlx90614
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
#define DEVICE                          0x5A
#define SDA_PORT PORTC
#define SDA_PIN 4   //define the SDA pin         
#define SCL_PORT PORTC
#define SCL_PIN 5   //define the SCL pin
#define GRAPH_0 240   //Axis X
#define GRAPH_ST_X 3   //Axis Y
#define GRAPH_MAX_X GRAPH_0-GRAPH_ST_X-2


#define GFX_RGB565_R(color)      ((0xF800 & color) >> 11)
#define GFX_RGB565_G(color)     ((0x07E0 & color) >> 5)
#define GFX_RGB565_B(color)     (0x001F & color)
#define GFX_RGB565(r, g, b)      ((((r & 0xF8) >> 3) << 11) | (((g & 0xFC) >> 2) << 5) | ((b & 0xF8) >> 3))





#include <Wire.h>         // this #include still required because the RTClib depends on it
#include "RTClib.h"
//#include <Adafruit_MLX90614.h>
#include "SoftI2CMaster.h"

//Adafruit_MLX90614 mlx = Adafruit_MLX90614();


//43
#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif
#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin
#define TS_MINX GRAPH_0
#define TS_MINY 210
#define TS_MAXX 892
#define TS_MAXY 939


#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY    0xF7DE
#define SETKA   0x8C90







#define BOXSIZE 40
#define PENRADIUS 3

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
RTC_Millis rtc;



int oldcolor, currentcolor;
int DH=0;
int pin = 13;
volatile int state = LOW;
volatile int STATE = LOW;
char buffer[200];
float L_ambient, L_object;
int lastSec=70;
  int xGraphPos=0;
  int GraphFrames=0;
DateTime now;


float GetTemperature(int Temperature_kind) {
    int dev = DEVICE<<1;
    int dataLow = 0;
    int dataHigh = 0;
    int pec = 0;
    
    i2c_start(dev+I2C_WRITE);
    i2c_write(Temperature_kind);  
    // read
    i2c_rep_start(dev+I2C_READ);
    dataLow = i2c_read(false); 
    dataHigh = i2c_read(false); 
    pec = i2c_read(true);
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((dataHigh & 0x007F) << 8) + dataLow);
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;

    return celcius;
  }

void setup(void) {
  //attachInterrupt(0, blink, CHANGE); // привязываем 0-е прерывание к функции blink().
  pinMode(pin, OUTPUT);
  
  uint16_t identifier = tft.readID();
  tft.begin(identifier);

  Serial.begin(230400);
    rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
   Serial.println("Adafruit MLX90614 test");  


 tft.fillScreen(BLACK);
 tft.setTextColor(CYAN); 
 tft.setTextSize(1);
 tft.setCursor(0, 30);
 tft.print("  IR               Ambient");
 tft.setTextSize(2);

  now = rtc.now();
  lastSec=now.second();
  tft.setCursor(0, 00);
  sprintf(buffer, "%d.%d.%d",now.day(), now.month(), now.year());
  tft.print(buffer);
  tft.fillRect(00, 40, 240, 15, RED);
  tft.drawFastVLine(GRAPH_ST_X-2, 82, 200, YELLOW);
  tft.drawFastHLine(1, GRAPH_0, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0-30, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0-60, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0-90, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0-120, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0-150, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0+20, 3, YELLOW);
  tft.drawFastHLine(1, GRAPH_0+40, 3, YELLOW);

 i2c_init(); 

 
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000



void LPad0(char *pszBuffer, int lenn)
{
  int i;
  while ((i<lenn)&&pszBuffer[i]==' ')   {
    pszBuffer[i]='0'; i++;
    }
}

void sprintfFloat(char *pszBuffer, float val,int lenL,int lenR)
{
  int v;
  v=(int)val;
  char Bufl[10],Bufr[10];
  if (lenL==1) sprintf(Bufl, "%1d", v);
  else if (lenL==2) sprintf(Bufl, "%2d", v);  
  else if (lenL==3) sprintf(Bufl, "%3d", v);  
  else if (lenL==4) sprintf(Bufl, "%4d", v);
  //LPad0(Bufl, lenL);
  memcpy((char*)&(pszBuffer[0]),Bufl, lenL);
  pszBuffer[lenL]='.';
  
  int i=0,j=0,mult=1;
  while (i<lenR) {
    mult *=10;
    i++;
    } 
  if (val>0)  
     v=(int)((val-(int)val)*mult);
  else
     v=(int)((-val+(int)val)*mult);
  if (lenR==1) sprintf(Bufr, "%1d", v);
  else if (lenR==2) sprintf(Bufr, "%2d", v);  
  else if (lenR==3) sprintf(Bufr, "%3d", v);  
  else if (lenR==4) sprintf(Bufr, "%4d", v);
  
  LPad0(Bufr, lenR);
  memcpy((char*)&(pszBuffer[lenL+1]),Bufr,lenR);
  pszBuffer[lenL+lenR+1]=0;  

}


void loop()
{
  TSPoint p = ts.getPoint();
 
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  now = rtc.now();

  
  
  L_object=GetTemperature(MLX90614_TOBJ1);
  //L_object -=30;
  char buffer1[10];
  if (xGraphPos>=GRAPH_MAX_X){
    GraphFrames++;
    xGraphPos=0;
  }
  // display value ob objec temp
  if  (xGraphPos%5==1) {
    tft.fillRect(00, 40, 80, 15, RED);
    sprintfFloat(buffer1, L_object,3,2);
    tft.setCursor(0, 40);
    tft.print(buffer1);
  }
  // display value of ambient temp
  if  (xGraphPos%20==1) {
    L_ambient=GetTemperature(MLX90614_TA);
    sprintfFloat(buffer1, L_ambient,3,2);
    tft.fillRect(100, 40, 80, 15, RED);
    tft.setCursor(100, 40);
    tft.print(buffer1);
  }
    
  int dt,i,l_r,l_g,l_b;
  uint16_t colLine;
  
  //clear line
  tft.drawFastVLine(GRAPH_ST_X+(xGraphPos+10)%(GRAPH_MAX_X), 80, 200, BLACK);
  if (L_object>=0) {
    dt=3*(int)L_object;
    if (dt>160) dt=160;
    l_r=(L_object*5.0>255?255:(L_object)*5.0);
    l_g=255*(float)((L_object>100?0:100.0-L_object)/100.0);
    l_b=255*(float)((L_object>25?0:25.0-L_object)/25.0);
    colLine=GFX_RGB565(l_r, l_g, l_b) ;
    //tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0-dt, dt, (lastSec!=now.second()?SETKA:GREEN));
    tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0-dt, dt, (lastSec!=now.second()?SETKA:colLine));
    //positve horisontal grid 
    i=1;
    while ((i<=160/10/3)&&(dt>3*10*i)) {
      tft.drawPixel(GRAPH_ST_X+xGraphPos, GRAPH_0-3*10*i, SETKA); 
      i++;
    }
  } else  {
   dt=2*(int)(-L_object);
   if (dt>=40) dt=39;
    l_r=0;
    l_g=255*(float)((80+L_object)/170.0);
    l_b=255*(float)((-L_object>14?14:-L_object)/14.0);
    colLine=GFX_RGB565(l_r, l_g, l_b) ;
   //tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0+1, dt, (lastSec!=now.second()?SETKA:BLUE));
   tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0+1, dt, (lastSec!=now.second()?SETKA:colLine));
    //negative horisontal grid 
    i=1;
    while ((i<=80/10/2)&&(dt>2*10*i)) {
      tft.drawPixel(GRAPH_ST_X+xGraphPos, GRAPH_0+2*10*i, SETKA); 
      i++;
    }
   
  }
  tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0-1, 2, YELLOW);
  // one second tic 
  if (lastSec!=now.second()) {
    //tft.fillRect(140, 0, 100, 15, RED);
    //tft.fillRect(140, 0, 60, 15, RED);
    tft.fillRect(211, 0, 26, 15, BLACK);
    if (lastSec==59) tft.fillRect(140, 0, 60, 15, BLACK); 
    tft.setCursor(140, 0);
    sprintf(buffer, "%2d:%2d:%2d",now.hour(),now.minute(),now.second());
    int i=0;
    while (i<9) {
      if (buffer[i]==' ') buffer[i]='0'; 
      i++;
    }
    tft.print(buffer);
    lastSec=now.second();
    tft.drawFastVLine(GRAPH_ST_X+xGraphPos, GRAPH_0-3, 6, YELLOW);
  }
  xGraphPos++;




  delay(50);
    

  
  /*if ((state==!STATE)||(p.z > MINPRESSURE && p.z < MAXPRESSURE)) {
    Serial.print("X = "); Serial.print(p.y);
    Serial.print("\tY = "); Serial.print(p.x);
    Serial.print("\tPressure = "); Serial.println(p.z);


  
    tft.print("Xs=");tft.print(p.y);tft.print(" Ys=");tft.println(p.x);
    p.y = map(p.y, TS_MINX, TS_MAXX, tft.width(), 0);
    p.x = map(p.x, TS_MINY, TS_MAXY, tft.height(), 0);
    tft.print("Xd=");tft.print(p.y);tft.print(" Yd=");tft.println(p.x);
  }*/
}

