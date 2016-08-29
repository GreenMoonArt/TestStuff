
/*** testing various things - this software is not ready yet - just posting here to back up to cloud ***/


/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SD.h>              // SD card has image bitmaps on it
#include <SPI.h>
#include "happy.c"

typedef Adafruit_ST7735  displayType; // Using TFT display(s)


// Microphone is connected to A0 (Teensy pin 14)
const int sampleWindow = 25; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned long startMillis;
unsigned int peakToPeak;
unsigned int signalMax;
unsigned int signalMin;

unsigned int sleepLevel = 60;
unsigned int conversationLevel = 100;
unsigned int loudLevel = 250;



// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define DISPLAY_DC       7     // TFT_DC was 8
#define DISPLAY_RESET    8     // TFT_RST was 9 ... you can also connect this to the Arduino reset
                               // in which case, set this #define pin to 0!
#define SELECT_L_PIN     9     // TFT_CS_Left was 10
#define SELECT_R_PIN    10     // TFT_CS_Right was 7

#define WINK_L_PIN      0 // Pin for LEFT eye wink button
#define BLINK_PIN       1 // Pin for blink button (BOTH eyes)
#define WINK_R_PIN      2 // Pin for RIGHT eye wink button
#define AUTOBLINK         // If enabled, eyes blink autonomously
// Eye blinks are a tiny 3-state machine.  Per-eye allows winks + blinks.
#define NOBLINK 0     // Not currently engaged in a blink
#define ENBLINK 1     // Eyelid is currently closing
#define DEBLINK 2     // Eyelid is currently opening

typedef struct {
  int8_t   pin;       // Optional button here for indiv. wink
  uint8_t  state;     // NOBLINK/ENBLINK/DEBLINK
  int32_t  duration;  // Duration of blink state (micros)
  uint32_t startTime; // Time (micros) of last state change
} eyeBlink;

struct {
  displayType display; // OLED/TFT object
  uint8_t     cs;      // Chip select pin
  eyeBlink    blink;   // Current blink state
} eye[] = { // OK to comment out one of these for single-eye display:
  displayType(SELECT_L_PIN,DISPLAY_DC,0),SELECT_L_PIN,{WINK_L_PIN,NOBLINK},
  displayType(SELECT_R_PIN,DISPLAY_DC,0),SELECT_R_PIN,{WINK_R_PIN,NOBLINK},
};
#define NUM_EYES (sizeof(eye) / sizeof(eye[0]))


#define SD_CS    4  // Chip select line for SD card

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_ST7735 tft_Left = Adafruit_ST7735(TFT_CS_Left,  TFT_DC, TFT_RST);

//Adafruit_ST7735 tft_Right = Adafruit_ST7735(TFT_CS_Right, TFT_DC, TFT_RST);

//TFT_ILI9163C tft[2] = { TFT_ILI9163C(REDPCB_NEW, __CS1, __DC) , TFT_ILI9163C(REDPCB_OLD, __CS2, __DC) };
// https://github.com/sumotoy/TFT_ILI9163C/blob/Pre-Release-1.0p7/examples/doubleDisplay_bars/doubleDisplay_bars.ino4

//Adafruit_ST7735 tft[2] = { Adafruit_ST7735(TFT_CS_Left,  TFT_DC, TFT_RST), Adafruit_ST7735(TFT_CS_Right, TFT_DC, TFT_RST) };


// Option 2: use any pins but a little slower!
//#define TFT_SCLK 13   // set these to be whatever pins you like!
//#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft[0] = Adafruit_ST7735(TFT_CS_Left, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


float p = 3.1415926;


boolean blnAsleep = true;
int blinkRate = 50;
int eyeOpenRate = 1000;


// Use names for states so it's easier to
// understand what the code is doing
const int S_SLEEPING = 1;
const int S_WAKING_LAZY = 2;
const int S_WAKING_BLINKING = 3;
const int S_AWAKE = 4;
const int S_BACKTOSLEEP = 5;
const int S_STARTLED = 6;

int eyeState = S_SLEEPING;    // keep track of state - start in sleep state

unsigned long previousMillis = 0;


void setup(void) {
  Serial.begin(9600);

  // Use this initializer if you're using a 1.8" TFT
  //tft[0].initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Both displays share a common reset line; 0 is passed to display
  // constructor (so no reset in begin()) -- must reset manually here:
  pinMode(DISPLAY_RESET, OUTPUT);
  digitalWrite(DISPLAY_RESET, LOW);  delay(1);
  digitalWrite(DISPLAY_RESET, HIGH); delay(50);

  for(int e=0; e<NUM_EYES; e++) { // Deselect all
    pinMode(eye[e].cs, OUTPUT);
    digitalWrite(eye[e].cs, HIGH);
  }

  for(int e=0; e < NUM_EYES; e++) 
  {
    digitalWrite(eye[e].cs, LOW); // Select one eye for init
    eye[e].display.initR(INITR_144GREENTAB);
    if(eye[e].blink.pin >= 0) pinMode(eye[e].blink.pin, INPUT_PULLUP);

    eye[e].display.fillScreen(ST7735_BLACK);

    digitalWrite(eye[e].cs, HIGH); // Deselect
  }

  //Serial.println("Initialized");

  // from https://forum.pjrc.com/threads/31221-SD-card-which-library
  //SPI.setMOSI(TFT_MOSI);
  //SPI.setSCK(TFT_SCLK);


  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) 
  {
    Serial.println("failed!");
    //return;

    //buying some time, retrying -- it doesn't always "take" the first time
    delay(250);
    if (!SD.begin(SD_CS)) 
    {
      Serial.println("failed again!");
      return;
    }

  }
  else
    {Serial.println("SD OK!");}

    

    //tft[0].fillScreen(ST7735_BLACK);
    //testdrawtext("Eye see you!", ST7735_WHITE);
    //bmpDraw("eyepurp.bmp", 0, 0, 0);   // left eye
    //bmpDraw("eyepurp.bmp", 0, 0, 1);   // right eye
    //delay(50);
    bmpDraw("asleepL.bmp", 0, 0, 0);   // left eye
    bmpDraw("asleepR.bmp", 0, 0, 1);   // right eye

    delay(1000);


    // drawXBitmap()
    // Draw XBitMap Files (*.xbm), exported from GIMP,
    // Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
    // C Array can be directly used with this function

    //test drawing from PROGMEM
    //eye[0].display.drawBitmap(0, 0, awakeL, 128, 128, ST7735_WHITE);   //monochromatic
    //eye[0].display.drawXBitmap(0, 0, awakel2, 128, 128, ST7735_WHITE);  //monochromatic
}

int countSpuriousPeak = 0;

void loop() {

    // old microphone approach
    //int wakeVolume = 425;
    //int sadVolume = 550;

    const long quietPeriod = 20000; // interval at which to go back to sleep
    unsigned long currentMillis = millis();

    // *** SOUND LEVELS ***
    startMillis= millis();  // Start of sampling window
    peakToPeak = 0;   // peak-to-peak level
    signalMax = 0;
    signalMin = 1024;
    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow)
    {
      sample = analogRead(A0);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
    }
    peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    //double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
    Serial.print("***Sound level: ");
    Serial.println(peakToPeak);

   


  switch(eyeState)
  {
    case S_SLEEPING:

      if( peakToPeak > sleepLevel ) 
        { 
          // something odd happening with sound level
          if(peakToPeak > 190)
            { // stay in sleep mode
              countSpuriousPeak++;
              if(countSpuriousPeak > 3)
                {
                  eyeState = S_WAKING_LAZY; 

                  countSpuriousPeak = 0;
                }
              
            }
            
            else 
            { eyeState = S_WAKING_LAZY;  }
          
          }

      else if(!blnAsleep)  //only read eyes from SD is not previously asleep
        { asleep(); 
          eyeState = S_SLEEPING;
        }     

      blnAsleep = true;

      Serial.print("SLEEPING level: ");
      Serial.println(peakToPeak);
      
      break;


    case S_WAKING_LAZY:

      sleepyEyes();

      eyeState = S_WAKING_BLINKING;  // transition to next state immediately
      blnAsleep = false;
      previousMillis = currentMillis;

      Serial.print("WAKING_LAZY level: ");
      Serial.println(peakToPeak);
      
      break;

    case S_WAKING_BLINKING:
      transitionToWaking(true);

      eyeState = S_AWAKE;
      blnAsleep = false;
      previousMillis = currentMillis;

      Serial.print("WAKING_BLINKING level: ");
      Serial.println(peakToPeak);
      
      break;


    case S_AWAKE:

      if( peakToPeak > loudLevel )  // too loud - sad eyes 
        { eyeState = S_STARTLED; }

      else if (currentMillis - previousMillis >= quietPeriod)
        { 
          if( peakToPeak <= sleepLevel ) 
                {eyeState = S_BACKTOSLEEP; }
        }

      else
        { awakeState(); }

      Serial.print("AWAKE level: ");
      Serial.println(peakToPeak);

      //previousMillis = currentMillis;
      blnAsleep = false;
      break;


    case S_BACKTOSLEEP:

      transitionToSleeping();
      eyeState = S_SLEEPING;
      blnAsleep = false;
      previousMillis = currentMillis;

      Serial.print("BACKTOSLEEP level: ");
      Serial.println(peakToPeak);
      
      break;



    case S_STARTLED:

      startledState(1000);

      if( peakToPeak < loudLevel )
        {eyeState = S_AWAKE; }  // back to awake state

      blnAsleep = false;
      previousMillis = currentMillis;

      Serial.print("STARTLED level: ");
      Serial.println(peakToPeak);
      
      break;



    default:
      break;

  }



/*
    // old logic - replaced by state machine 

    if( peakToPeak > loudLevel)  // too loud - sad eyes
    {
      // sad/startled
      Serial.println("TOO LOUD!");
      startledState(1000);
      blnAsleep = false;

      eyeState = S_STARTLED;

      previousMillis = currentMillis;
    }

    else if( peakToPeak > sleepLevel )
    {
      // blinking
      Serial.println("AWAKE...");
      awakeState();
      blnAsleep = false;

      previousMillis = currentMillis;
    }

    else
    {
      if(!blnAsleep)  // if already asleep, no need to keep reading asleep images
      {
        //if sleep timer, then sleep
        if (currentMillis - previousMillis >= quietPeriod)
        {
          //go to sleep
          Serial.println("GOING TO SLEEP ---- Yawn");

          //bmpDraw("eyelid2.bmp", 0, 0, 0);   // left eye
          //bmpDraw("eyelid2.bmp", 0, 0, 1);   // right eye

          bmpDraw("sleepyL.bmp", 0, 0, 0);   // left eye
          bmpDraw("sleepyR.bmp", 0, 0, 1);   // right eye
          delay(2500);
          bmpDraw("asleepL.bmp", 0, 0, 0);   // left eye
          bmpDraw("asleepR.bmp", 0, 0, 1);   // right eye
          delay(3000);
          blnAsleep = true;

          eyeState = S_SLEEPING;
        }
      }

    }

*/

/*
    Serial.println("--------------------");
    Serial.print("current : ");
    Serial.println(currentMillis);
    Serial.print("previous : ");
    Serial.println(previousMillis);
*/
    

} // end of loop() 






/*  *********************************  */
/*  state functions  */
void asleep()
{
  Serial.println("ASLEEP ...");
  bmpDraw("asleepL.bmp", 0, 0, 0);   // left eye
  bmpDraw("asleepR.bmp", 0, 0, 1);   // right eye

}


void sleepyEyes()
{
    bmpDraw("sleepyL.bmp", 0, 0, 0);
    bmpDraw("sleepyR.bmp", 0, 0, 1); 
    delay(1000);
}


void transitionToWaking(bool blnWaking)
{
  // blnWaking = true if waking up
  // blnWaking = false if going back to sleep

  if(blnWaking)
  {
      //opening eyes
      bmpDraw("asleepL.bmp", 0, 0, 0);
      bmpDraw("asleepR.bmp", 0, 0, 1);   
      delay(blinkRate);
      bmpDraw("wake1L.bmp", 0, 0, 0);
      bmpDraw("wake1R.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("wake2L.bmp", 0, 0, 0);
      bmpDraw("wake2R.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("wake3L.bmp", 0, 0, 0);
      bmpDraw("wake3R.bmp", 0, 0, 1);
      delay(blinkRate);

  }
  else   // reverse order of waking
  {
      bmpDraw("sleepyL.bmp", 0, 0, 0);
      bmpDraw("sleepyR.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("wake3L.bmp", 0, 0, 0);
      bmpDraw("wake3R.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("wake2L.bmp", 0, 0, 0);
      bmpDraw("wake2R.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("wake1L.bmp", 0, 0, 0);
      bmpDraw("wake1R.bmp", 0, 0, 1);
      delay(blinkRate);
      bmpDraw("asleepL.bmp", 0, 0, 0);
      bmpDraw("asleepR.bmp", 0, 0, 1);   
      delay(blinkRate);

  }

  
}


void transitionToSleeping()
{
    transitionToWaking(false);  //call transitionToWaking with false argument
}



void awakeState()
{

      // **** Need a blink timer ****

      Serial.println("AWAKE...");
      // blinking
      //bmpDraw("asleepL.bmp", 0, 0, 0);
      //bmpDraw("asleepR.bmp", 0, 0, 1);   
      //delay(blinkRate);
      bmpDraw("wake1L.bmp", 0, 0, 0);
      bmpDraw("wake1R.bmp", 0, 0, 1);
      delay(blinkRate);
      //bmpDraw("wake2L.bmp", 0, 0, 0);
      //bmpDraw("wake2R.bmp", 0, 0, 1);
      //delay(blinkRate);
      bmpDraw("wake3L.bmp", 0, 0, 0);
      bmpDraw("wake3R.bmp", 0, 0, 1);
      delay(blinkRate);


      //Wide open
      bmpDraw("awakeL.bmp", 0, 0, 0);   // left eye
      bmpDraw("awakeR.bmp", 0, 0, 1);   // right eye
      delay(eyeOpenRate);
      delay(eyeOpenRate);
      delay(eyeOpenRate);
      //delay(eyeOpenRate);
      //delay(eyeOpenRate);

      
      delay(eyeOpenRate);
}

void startledState(int delAmount)
{
      Serial.println("TOO LOUD!");
      //bmpDraw("eyepurp1.bmp", 32, 32, 0);  //look right
      bmpDraw("sadL.bmp", 0, 0, 0);  //look right
      bmpDraw("sadR.bmp", 0, 0, 1);  //look right
      delay(delAmount);
      //bmpDraw("rotate2.bmp", 0, 0, 0);  //look left
      //bmpDraw("rotate1.bmp", 0, 0, 1);  //look left
      //delay(delAmount);
}



// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

SPISettings settings(24000000, MSBFIRST, SPI_MODE3); // Teensy 3.1 max SPI

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint8_t y, 
              uint8_t e )     // Eye array index; 0 or 1 for left/right

{

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  boolean  deBug = true;

  if((x >= eye[e].display.width()) || (y >= eye[e].display.height())) return;

  if(deBug){
  //Serial.println();
  //Serial.print("Loading image '");
  //Serial.print(filename);
  //Serial.println('\'');
  }

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    int fileSize = read32(bmpFile);
    //if(deBug){Serial.print("File size: "); Serial.println(fileSize);}
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    //if(deBug){Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);}
    // Read DIB header
    int bmpHeader = read32(bmpFile);
    //if(deBug){Serial.print("Header size: "); Serial.println(bmpHeader);}
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      //if(deBug){Serial.print("Bit Depth: "); Serial.println(bmpDepth);}
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        if(deBug){
        //Serial.print("Image size: ");
        //Serial.print(bmpWidth);
        //Serial.print('x');
        //Serial.println(bmpHeight);
        }

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= eye[e].display.width())  w = eye[e].display.width()  - x;
        if((y+h-1) >= eye[e].display.height()) h = eye[e].display.height() - y;

        // Set TFT address window to clipped image bounds
        eye[e].display.setAddrWindow(x, y, x+w-1, y+h-1);

        SPI.beginTransaction(settings);
        // frequent selecting and de-selecting is causing weird image pixelation and distortion
        //digitalWrite(eye[e].cs, LOW);                       // Chip select
        //digitalWrite(DISPLAY_DC, HIGH);                     // Data mode

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            eye[e].display.pushColor(eye[e].display.Color565(r,g,b));
          } // end pixel
        } // end scanline

        if(deBug){
        //Serial.print("Loaded in ");
        //Serial.print(millis() - startTime);
        //Serial.println(" ms");
        }

      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println("BMP format not recognized.");

  //digitalWrite(eye[e].cs, HIGH);          // Deselect
  SPI.endTransaction();
}



// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}








