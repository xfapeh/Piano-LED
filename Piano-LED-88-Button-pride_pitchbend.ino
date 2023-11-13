#include <FastLED.h>
#include <MIDI.h>
//#include "esp_camera.h"       // not neccessary
//#include <SoftwareSerial.h>   // not neccessary

#define NUM_LEDS 177
// 177 -> 176 hue"+1" weg
//#define DATA_PIN 10              // Arduino Nano
//#define DATA_PIN 4               // ESP32-CAM, ESP8266 = D2
#define DATA_PIN 4              // ESP8266
//#define DATA_PIN 18              // APA SPI
//#define CLOCK_PIN 23             // APA SPI


CRGB leds[NUM_LEDS];
CRGB lastcolor[NUM_LEDS] = {};
CHSV lasthsv;
    uint8_t hue = 0;
    uint8_t prehue = 0;
    uint8_t bendhue = 0;          // pitch number for pitchbend start
    uint8_t bendprehue = 0;       // previous pitch number for pitchbend start, 2-polyfony
    uint8_t bendvel = 0;
    uint8_t bendprevel = 0;
    uint8_t pitchbendup   =  2;   // adjust in Synth to match
    uint8_t pitchbenddown = 12;   // adjust in Synth to match
    bool sustain = 0;
    bool note[127] = {};
    int num = 0;
    int oddeven = 0;
    float huescale = 0.5; 
    uint32_t reduceloop = 0;
    int beat=0;
    int flickerstrength = 30;
    int wave = 0;
    int wavebright = 30;
    int wavefreq = 10;    // 2  = slow + wide
    int wavephase = 30;   // 39 = slow + wide
    int minbright = 5;   // min 25 for colorchange, 20 for pride
    int brightness[NUM_LEDS];
// ---------------------------------------------------------------------------------------------------------------
    //const int buttonPin1 = 2;       // alle,      ESP8266 = D4 geht nicht! beim Upload Button drücken, ist immer HIGH
    //const int buttonPin1 = 12;       // ESP8266 = D6
    //const int buttonPin2 = 3;     // Arduino Nano
    //const int buttonPin3 = 4;     // Arduino Nano
    //const int buttonPin2 = 5;    // ESP32-CAM, ESP8266 = D7
    //const int buttonPin3 = 14;    // ESP32-CAM ESP8266 = D5
    const int buttonPin1 = 18;       // ESP32-DEV
    const int buttonPin2 = 19;       // ESP32-DEV
    const int buttonPin3 = 21;     // ESP32-DEV

    int buttonState1 = 0;
    int buttonState2 = 0;
    int buttonState3 = 0;
// ---------------------------------------------------------------------------------------------------------------
// button 1 (RED)
// playing modes:
    // 0: only when key is pressed (best for solo lead)
    // 1: fadetoblack: LEDs of approx last 15 keys are lit (doesnt stop when playing stops - panic?)
    // 2: key pressed + sustained notes, sustain release = all of except currently pressed keys (slow shutdown)
    // 3: PianoTutor ???
int playmode = 2;
// ---------------------------------------------------------------------------------------------------------------
// button 2 (GREEN)
// color modes:
    // 0: 1 LED / key : slow changing color wave
    // 1: 2 LED / key : slow changing color wave   
    // 2: 1 LED / key : rainbow on octave  
    // 3: 2 LED / key : rainbow on octave
    // 4: 1 LED / key : rainbow on full keyboard  
    // 5: 2 LED / key : rainbow on full keyboard
    // 6: 1 LED / key , residual keys still lit up, colorchange/pride, temporary playmode 2 only! without pitchbend
    // toDo: pitchbend only last note, 1 LED
int colormode = 6;
// ---------------------------------------------------------------------------------------------------------------
// button 3 (BLUE)
// pitchbend modes: +pitchbendup -pitchbenddown
    // 0: +2 -2
    // 1: +2 -12
    // 2: +12 -12
    // 3: +12 -24
    // 4: +24 -24
// ---------------------------------------------------------------------------------------------------------------

MIDI_CREATE_DEFAULT_INSTANCE();

// -----------------------------------------------------------------------------

// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
    // Do whatever you want when a note is pressed.
    hue=pitch;
    bendvel = velocity;
    //uint8_t beat = beatsin8( 1, 64, 255);
    uint8_t beat = triwave8( num );

    if (colormode == 0 || colormode == 1) {
    // slow changing rainbow over whole keyboard, 2 LEDs / key
    // 177 LEDs -> note plus 1 down = better for 2 LEDs
    // 177 LEDs -> 1 LED : -42+2*hue+1
    
    leds[-42+2*hue+1]   = CHSV(huescale*(-42+2*hue)  -beat, 255, (255*abs(velocity-20))/127);  // -beat : change from left to right
      if (colormode == 1) {
      leds[-42+2*hue] = CHSV(huescale*(-42+2*hue+1)-beat, 255, (255*abs(velocity-20))/127);  // +beat : change from right to left
      }
    if        (num!=127 && oddeven==0){ num++; }
      else if (num!=127 && oddeven!=0){}
      else                            { num=0; };
      oddeven++;
      oddeven=oddeven % 4;
    }

    if (colormode == 2 || colormode == 3) {
    // full color / octave, 2 LEDs / key
    // 177 LED -> note+1
    leds[-42+2*hue+1]   = CHSV(( -(-42+2*hue+1))%24*255/24, 255, (255*abs(velocity-20))/127);
      if (colormode == 3) {
      leds[-42+2*hue] = CHSV((-(-42+2*hue+1+1))%24*255/24, 255, (255*abs(velocity-20))/127);    //leds[56] = CRGB(50,50,50); 
      }
    }
    
    if (colormode == 4 || colormode == 5) {
    // full color / whole keyboard, 2 LEDs / key
    // 177 LED -> note+1
    leds[-42+2*hue+1]   = CHSV(( -(-42+2*hue+1))%176*255/176, 255, (255*abs(velocity-20))/127);
      if (colormode == 5) {
      leds[-42+2*hue] = CHSV((-(-42+2*hue+1+1))%176*255/176, 255, (255*abs(velocity-20))/127);    //leds[56] = CRGB(50,50,50); 
      }
    }
    
    if (colormode == 6 ) {
    // pride or other fading color
    // 177 LED -> note+1
      int i=-42+2*hue+1;
      brightness[i] = min(2*(255*velocity)/127 , 255);
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate( leds[i] ).h , 255 , brightness[i] ) , leds[i] );
    }

    FastLED.show();
    //delay(1000);
    note[pitch]=true;
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
    if (playmode == 0) {
    // for pitch bend:
    prehue=hue;
    bendprevel=bendvel;
    hue=pitch;
    leds[-42+2*hue+1] = CHSV(huescale*(-42+2*hue+1),255,0);
    if (colormode == 1 || colormode == 3) {
      leds[-42+2*hue] = CHSV(huescale*(-42+2*hue),255,0);
    }
    FastLED.show();
    }

    if (playmode == 1) {
    fadeToBlackBy( leds, NUM_LEDS, 10);
    FastLED.show();
    }
    
    if (playmode == 2) {
      if ( colormode != 6 ){
    if(sustain==0)
    {
    hue=pitch;
    leds[-42+2*hue+1] = CHSV(huescale*(-42+2*hue+1),255,0);
    if (colormode == 1 || colormode == 3 || colormode == 5) {
      leds[-42+2*hue] = CHSV(huescale*(-42+2*hue),255,0);
    }
    FastLED.show();
    }
      }
      if ( colormode == 6 ){
    if(sustain==0)
    {
    hue=pitch;
    int j = -42+2*hue+1;
    int k = -42+2*hue;
    hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[j] ).h , 255 , minbright ) , leds[j] );
    hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[k] ).h , 255 , minbright ) , leds[k] );
    brightness[j]=minbright;
    brightness[k]=minbright;
    
    FastLED.show();
    }    
      }
    }

    note[pitch]=false;
}

void handleControlChange(byte channel, byte number, byte value)
{
    if (playmode == 2) {
    if(number==64 && value==127)
    {
    sustain=1;
    }

    if(number==64 && value==0)
    {
        
      for(int i=21; i<=108; i++)
      {
        //FastLED.clear();
        //FastLED.show();
        if(note[i]==false)
        { 
        if (colormode == 6) {
          int j = -42+2*i+1; 
          int k = -42+2*i;
          hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[j] ).h , 255 , minbright ) , leds[j] );
          brightness[j]=minbright;
          if (colormode == 1 || colormode == 3 || colormode == 5) {
            hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[k] ).h , 255 , minbright ) , leds[k] );
            brightness[k]=minbright;
          }
        }
        else {
          leds[-42+2*i+1] = CRGB(0,0,0);       
          if (colormode == 1 || colormode == 3 || colormode == 5) {
            leds[-42+2*i] = CRGB(0,0,0);
          }
        }
          //fill_solid(leds, 144, CRGB(0,0,0));
        }
      }
      FastLED.show();  
    
      sustain=0;
    }
    }

    if (number==91) {
      flickerstrength = value;
    }
    if (number==93) {
      wavebright = value;
    }
    if (number==95) {
      wavefreq = value;
    }
    if (number==96) {
      wavephase = value;
    }
}

void pitchbend(byte channel, int bend)
{ if(bend!=0)
  {
  if(note[hue]==true)
  {
  leds[-42+2*hue+1]         = CHSV(0,255,0);    // primary note LED off
  leds[-42+1*(bendhue)+1]   = CHSV(0,255,0);    // switch previous bend led off
  // adjust bend range up and down -------- BEGIN
  if (bend > 0) {
    bendhue=2*hue+pitchbendup*bend/4095;                    // pitch for EVERY LED, 8192=2HT=4LED
  }
  else if (bend < 0) {
    bendhue=2*hue+pitchbenddown*bend/4095;                  // pitch for EVERY LED, 8192=2HT=4LED
  }
  // adjust bend range up and down -------- END
  
  // leds[-42+1*(bendhue)+1]   = CHSV(( -(-42+2*hue+1))%24*255/24, 255, (255*abs(bendvel-20))/127); // wrong: 1 color for all
  if (colormode == 0 || colormode == 1) {
    leds[-42+1*(bendhue)+1]   = CHSV(huescale*(-42+2*hue)  -beat, 255, (255*abs(bendvel-20))/127);
  }
  if (colormode == 2 || colormode == 3) {
    leds[-42+1*(bendhue)+1]   = CHSV(( -(-42+2*hue+1))%24*255/24, 255, (255*abs(bendvel-20))/127);
  }
  if (colormode == 4 || colormode == 5) {
    leds[-42+1*(bendhue)+1]   = CHSV(( -(-42+2*hue+1))%176*255/176, 255, (255*abs(bendvel-20))/127);
  }
  FastLED.show();
  }
  else if(note[prehue]==true)
  {
  leds[-42+2*prehue+1]         = CHSV(0,255,0);    // primary note LED off
  leds[-42+1*(bendprehue)+1]   = CHSV(0,255,0);    // switch previous bend led off
  // adjust bend range up and down -------- BEGIN
  if (bend > 0) {
    bendprehue=2*prehue+pitchbendup*bend/4095;                    // pitch for EVERY LED, 8192=2HT=4LED
  }
  else if (bend < 0) {
    bendprehue=2*prehue+pitchbenddown*bend/4095;                  // pitch for EVERY LED, 8192=2HT=4LED
  }
  // adjust bend range up and down -------- END
  
  // leds[-42+1*(bendprehue)+1]   = CHSV(( -(-42+2*prehue+1))%24*255/24, 255, (255*abs(bendprevel-20))/127); // wrong: 1 color for all
  if (colormode == 0 || colormode == 1) {
    leds[-42+1*(bendprehue)+1]   = CHSV(huescale*(-42+2*prehue)  -beat, 255, (255*abs(bendprevel-20))/127);
  }
  if (colormode == 2 || colormode == 3) {
    leds[-42+1*(bendprehue)+1]   = CHSV(( -(-42+2*prehue+1))%24*255/24, 255, (255*abs(bendprevel-20))/127);
  }
  if (colormode == 4 || colormode == 5) {
    leds[-42+1*(bendprehue)+1]   = CHSV(( -(-42+2*prehue+1))%176*255/176, 255, (255*abs(bendprevel-20))/127);
  }
  FastLED.show();
  }
  }
  else
  {
  if(note[hue]==false)   // note off -> LED off (+left+right)
  {  
    leds[-42+1*(bendhue)+1]   = CHSV(0,255,0);
/*      
  bendhue=2*hue+0*bend/4000;
  leds[-42+1*(bendhue)+1]     = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1+1]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1-1]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1+2]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1-2]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1+3]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1-3]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1+4]   = CHSV(0,255,0);
  leds[-42+1*(bendhue)+1-4]   = CHSV(0,255,0);
  FastLED.show(); 
*/
  }
  if(note[prehue]==false)   // note off -> LED off (+left+right)
  {  
    leds[-42+1*(bendprehue)+1]   = CHSV(0,255,0); 
  }
  FastLED.show();
  }    
}


// -----------------------------------------------------------------------------

void init_colormode6() {
    for(int i=0; i<NUM_LEDS; i++) {
      //lastcolor[i] = CRGB( 0,0,0 );
      leds[i] = CRGB( 0,0,0 );
      //hsv2rgb_spectrum( CHSV ( 100 , 255 , minbright ) , lastcolor[i] );
      hsv2rgb_spectrum( CHSV ( 170 , 255 , minbright ) , leds[i] );
      brightness[i]=minbright;
    }
}
void init_all_but6() {
    for(int i=0; i<NUM_LEDS; i++) {
      lastcolor[i] = CRGB( 0,0,0 );
      leds[i] = CRGB( 0,0,0 );
    }
}

void setup()
{
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
    //FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);  // BGR ordering is typical
    //FastLED.delay(0);
    // Connect the handleNoteOn function to the library,
    // so it is called upon reception of a NoteOn.
    MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function

    // Do the same for NoteOffs
    MIDI.setHandleNoteOff(handleNoteOff);

    MIDI.setHandleControlChange(handleControlChange);

    MIDI.setHandlePitchBend(pitchbend);

    // Initiate MIDI communications, listen to all channels
    //MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.begin(0);   // faster?
 
    pinMode(buttonPin1, INPUT);
    pinMode(buttonPin2, INPUT);
    pinMode(buttonPin3, INPUT);

    if (colormode==6){
      init_colormode6();
    }
    else {
      init_all_but6();
    }

    FastLED.show();
    delay(1000);
    FastLED.show();
    delay(1000);
}


void colorchange() {
  uint16_t wavenumber = 1000; // 2000 = 32 LEDs, 4000 = 16 LEDs
  uint16_t frequency  = 3;    // BPM, 2/60 = 1/30 Hz, 60/60 = 1 Hz

  //dynamic wavenumber and frequency
  wavenumber = beatsin16 ( 1 , 200 , 750 );  // works
  ////frequency = beatsin16 ( 2 , 10 , 20 );       // fail, phase glitch
  
  uint16_t hue16 = 0;
  //hue16 = beatsin16( frequency , 0 , 65535 , beatsin( 3 , 2 , 5 )*millis() , 64); // 16bit hue value, (BPM, min, max, timebase controls freq, phase)
  //hue16 = beatsin16( frequency , 0 , 65535 ); // 16bit hue value, sin (BPM, min, max)
  hue16 = 65535 - beat16( frequency );                // 16bit hue value, saw (BPM, min, max)

  for ( int i=0 ; i<NUM_LEDS ; i++ ) {
    hue16 += wavenumber;
    uint8_t hue8 = hue16 / 256;
    
    //CHSV lastled = rgb2hsv_approximate ( leds[i] );
    //hsv2rgb_spectrum( CHSV ( hue8 , 255 , lastled.v ) , leds[i] );
    
    //hsv2rgb_spectrum( CHSV ( hue8 , 255 , rgb2hsv_approximate ( leds[i] ).v) , leds[i] );       // VALUE asymptotes 255 in 4-5 steps
    hsv2rgb_spectrum( CHSV ( hue8 , 255 , brightness[i] ) , leds[i] );            // VALUE chaotes to 0
    
  } 
/* // not necessary
  for(int i=21; i<=108; i++) {
    if(note[i]==false && sustain == 0) {
      int j = -42+2*i+1;
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[j] ).h , 255 , minbright ) , leds[j] );
      brightness[j]=minbright; 
      int k = -42+2*i;
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate ( leds[k] ).h , 255 , minbright ) , leds[k] );
      brightness[k]=minbright; 
    }
  }
*/
}

void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  //uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint8_t brightdepth = beatsin88( 341, 96, 134);   // higher lowlevel to prevent gaps, minbright can then be lower than 20
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    //brightness[i] = bri8;
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    //nblend( leds[pixelnumber], newcolor, 64);
    hsv2rgb_spectrum( CHSV ( hue8 , 255 , brightness[pixelnumber]*bri8/255 ) , leds[pixelnumber] );            // VALUE chaotes to 0
  }
}

void loop()
{
    // Call MIDI.read the fastest you can for real-time performance.
  if (colormode==6){
    //colorchange();
    pride();
    FastLED.show();
    //delay(1000);    
    MIDI.read();
  }
  else{
    MIDI.read();

    
    reduceloop += 1;
    if (reduceloop%10000 == 0) {
    for(int i=0; i<=175; i++)
   // next 5 comment -> glitter off 
      { lastcolor[i] = leds[i];
        lasthsv   = rgb2hsv_approximate( lastcolor[i] );
        //leds[i] = CRGB( abs((lastcolor.r+random8(0,50)-25)%255), abs((lastcolor.g+random8(0,50)-25)%255), abs((lastcolor.b+random8(0,50)-25)%255)); // ging nicht so ...
   //     leds[i] = CHSV( lasthsv.h+random8(0,flickerstrength)-flickerstrength/2, lasthsv.s, lasthsv.v); // add some color flicker noise
   //     if( random8() < 5 && leds[i]!=CRGB(0,0,0)) {                                                   // add glitter
   //       leds[i] += CRGB(100,100,100);
   //     }
        if ((colormode == 0 || colormode == 2 || colormode == 4) && i%2 == 0) {
          if( random8() < 3*flickerstrength) {
   //       leds[i] = CHSV( lasthsv.h, lasthsv.s, min( max( lasthsv.v+random16(0,10)-8,0),255)); // snow random flicker in between (1 LED / key)     
          }       
          wave = 0;   
          beat = 0.2*beatsin8(wavefreq,0,wavebright,wavephase*i,0);
        //  leds[i] = CHSV( lasthsv.h, lasthsv.s, max(beat*beat-330,0)); // white wave
        }
      }
    FastLED.show();
    for(int i=0; i<=175; i++) {
      if (colormode == 0 || colormode == 2 || colormode == 4 && wave == 1 ) {  // white wave mode!
        if (i%2 == 1) {
        leds[i] = lastcolor[i];
        }
      }
      else { 
        leds[i] = lastcolor[i];
      }
    }
    reduceloop=0;
    }

  }
    
    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);

// button1 -> set playmode
if      (buttonState1 == HIGH) {
  playmode = 0;
  FastLED.clear();
  leds[0] = CRGB(100,0,0);
  FastLED.show(); 
  delay(1000);
  FastLED.clear();
  FastLED.show();
  buttonState1 = digitalRead(buttonPin1);
  if    (buttonState1 == HIGH) {
    playmode = 1;
    FastLED.clear();
    leds[1] = CRGB(100,0,0);
    FastLED.show(); 
    delay(1000);
    FastLED.clear();
    FastLED.show();
    buttonState1 = digitalRead(buttonPin1);
    if  (buttonState1 == HIGH) {
      playmode = 2;
      FastLED.clear();
      leds[2] = CRGB(100,0,0);
      FastLED.show(); 
      delay(1000);
      FastLED.clear();
      FastLED.show();
    }
  }
}

// button2 -> set colormode
if      (buttonState2 == HIGH) {
  colormode = 0;
  FastLED.clear();
  leds[0] = CRGB(0,100,0);
  FastLED.show(); 
  delay(1000);
  FastLED.clear();
  FastLED.show();
  buttonState2 = digitalRead(buttonPin2);
  if    (buttonState2 == HIGH) {
    colormode = 1;
    FastLED.clear();
    leds[1] = CRGB(0,100,0);
    FastLED.show(); 
    delay(1000);
    FastLED.clear();
    FastLED.show();
    buttonState2 = digitalRead(buttonPin2);
    if  (buttonState2 == HIGH) {
      colormode = 2;
      FastLED.clear();
      leds[2] = CRGB(0,100,0);
      FastLED.show(); 
      delay(1000);
      FastLED.clear();
      FastLED.show();
      buttonState2 = digitalRead(buttonPin2);
      if  (buttonState2 == HIGH) {
        colormode = 3;
        FastLED.clear();
        leds[3] = CRGB(0,100,0);
        FastLED.show(); 
        delay(1000);
        FastLED.clear();
        FastLED.show();
        buttonState2 = digitalRead(buttonPin2);
        if  (buttonState2 == HIGH) {
          colormode = 4;
          FastLED.clear();
          leds[4] = CRGB(0,100,0);
          FastLED.show(); 
          delay(1000);
          FastLED.clear();
          FastLED.show();
          buttonState2 = digitalRead(buttonPin2);
          if  (buttonState2 == HIGH) {
            colormode = 5;
            FastLED.clear();
            leds[5] = CRGB(0,100,0);
            FastLED.show(); 
            delay(1000);
            FastLED.clear();
            FastLED.show();
            buttonState2 = digitalRead(buttonPin2);
            if  (buttonState2 == HIGH) {
              colormode = 6;
              FastLED.clear();
              leds[6] = CRGB(0,100,0);
              FastLED.show(); 
              delay(1000);
              FastLED.clear();
              init_colormode6();
              FastLED.show();
            }
          }
        }
      }
    }
  }
}

// button3 -> set pitchbend range
if      (buttonState3 == HIGH) {
  pitchbendup   = 2;
  pitchbenddown = 2;
  FastLED.clear();
  leds[0] = CRGB(0,0,100);
  FastLED.show(); 
  delay(1000);
  FastLED.clear();
  FastLED.show();
  buttonState3 = digitalRead(buttonPin3);
  if    (buttonState3 == HIGH) {
  pitchbendup   = 2;
  pitchbenddown = 12;
    FastLED.clear();
    leds[1] = CRGB(0,0,100);
    FastLED.show(); 
    delay(1000);
    FastLED.clear();
    FastLED.show();
    buttonState3 = digitalRead(buttonPin3);
    if  (buttonState3 == HIGH) {
  pitchbendup   = 12;
  pitchbenddown = 12;
      FastLED.clear();
      leds[2] = CRGB(0,0,100);
      FastLED.show(); 
      delay(1000);
      FastLED.clear();
      FastLED.show();
      buttonState3 = digitalRead(buttonPin3);
      if  (buttonState3 == HIGH) {
  pitchbendup   = 12;
  pitchbenddown = 24;
        FastLED.clear();
        leds[3] = CRGB(0,0,100);
        FastLED.show(); 
        delay(1000);
        FastLED.clear();
        FastLED.show();
        buttonState3 = digitalRead(buttonPin3);
        if  (buttonState3 == HIGH) {
  pitchbendup   = 24;
  pitchbenddown = 24;
          FastLED.clear();
          leds[4] = CRGB(0,0,100);
          FastLED.show(); 
          delay(1000);
          FastLED.clear();
          FastLED.show();
          buttonState3 = digitalRead(buttonPin3);
          /*
          if  (buttonState3 == HIGH) {
  pitchbendup   = 2;
  pitchbenddown = 2;
            FastLED.clear();
            leds[5] = CRGB(0,0,100);
            FastLED.show(); 
            delay(1000);
            FastLED.clear();
            FastLED.show();
            buttonState3 = digitalRead(buttonPin3);
            if  (buttonState3 == HIGH) {
  pitchbendup   = 2;
  pitchbenddown = 2;
              FastLED.clear();
              leds[6] = CRGB(0,0,100);
              FastLED.show(); 
              delay(1000);
              FastLED.clear();
              init_colormode6();
              FastLED.show();
            }
          }
          */
        }
      }
    }
  }
}

// Demo Mode Rainbow
/*
      for(int i=21; i<=108; i++)
      {
//      uint8_t beat = beatsin8( 5, 64, 255);
      uint8_t beat = triwave8( num );
      leds[-42+2*i+1] = CHSV(huescale*(-42+2*i+1)+beat, 255, 255);
      leds[-42+2*i] = CHSV(huescale*(-42+2*i)+beat, 255, 255);
      }
      FastLED.show();  
      if (num!=127){ num++; }
      else {num=0;};
      delay(10);
*/
      
}
