 /* Arduino_LC_meter_sketch.ino
   a sketch for Arduino based LC meter firmware based on an
   assembly language program written for the PIC-based
   digital LC meter design published in the May 2008 issue
   of SILICON CHIP.
   Written by Jim Rowe (Silicon Chip)
   Version 1.0, updated 28/03/2017 at 4:45 pm
 
   Note: the project is designed to use a 16x2 LCD module with a
   serial I2C 'piggyback' interface based on a PCF8574T serial to
   parallel converter. These have an I2C address in the range from
   20-27h, with the highest address corresponding to all three
   links on the piggyback PCB being OPEN.
   However some modules have a PCF8574AT converter chip instead,
   and these have a different I2C address range: from 38-3Fh.
   If you have one of these latter modules, the only change that
   should need to be made is to change the address in line 21 below.
 */
 
#include "EEPROMUtils.h"
#include <EnableInterrupt.h>
#include "TimerTwo.h"
#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#include "OLEDUtils.h"
                                    

#include <FreqCount.h>            // and the FreqCount library

// ====================================================================
// declaration of global variables
int charSize = 2;

const float Pi = 3.14159;     // define Pi
const float FourPiSqrd = 39.4784;  // and Pi squared
const float uHmult = 1.0e6;   // multiplier for converting H into uH
const float mHmult = 1.0e3;   // multiplier for converting H into mH
const float pFmult = 1.0e12;  // multiplier for converting F into pF
const float nFmult = 1.0e9;   // multiplier for converting F into nF
const float uFmult = 1.0e6;   // multiplier for converting F into uF
const float C2val = 1.000e-9;   // nominal value of C2 (1nF = 1 x 10^-9F)
                              // (change to actual value if known)
const int CLbarPin = 2;   // digital IO pin 2 for C/L-bar sensing
const int RelayPin = 3;   // digital IO pin 3 for driving RLY1
const int DecrPin = 4;    // digital IO pin 4 for sensing DECR pos of S3
const int OscSigPin = 5;  // digital IO pin 5 for input of oscillator signal
const int IncrPin = 10;    // digital IO pin 6 for sensing INCR pos of S3 // was pd6
const int CalLkPin = 7;   // digital IO pin 7 for sensing CAL link LK1
const int CalErase = 8;   // jumper to clear eeprom calibration settings for factory reset *CJS

float C1val;     // C1's calculated value after calibration (in F)
float CXval;     // calculated value for Cx (in F)
float L1val;     // calculated value for L1 after calibration (H)
float LXval;     // calculated value for Lx (in H)
float F1sqrd;    // calc value for Freq1 squared (as a float)
float F2sqrd;    // calc value for Freq2 squared (as a float)
float F3sqrd;    // calc value for Freq3 squared (as a float)

volatile float CF;  // calibration factor, set by nudging via S3   

long Fcount;   // raw frequency count figure (from GetFrequency function)
long Freq1;    // measured Freq1 in Hz (L1 & C1 only)
long Freq2;    // measured Freq2 in Hz (L1 & (C1 + C2))
long Freq3;    // measured Freq3 in Hz (C1+Cx/L1 or C1/L1+Lx)

byte MTorNot = 0;   // flag: 0 = EEPROM addresses empty (== 255)
int manCalibEnabled = 0;

#define DEBOUNCE_DELAY 300 // in ms
unsigned long lastDebounceTimeIncr = 0;  // the last time the output pin was toggled

unsigned long lastDebounceTimeDecr = 0;  // the last time the output pin was toggled



// ====================================================================
// setup function begins here
// ====================================================================
void setup()
{ 
  Wire.begin();
  
  if(digitalRead(CalLkPin) == LOW) 
  {
    manCalibEnabled = 1;
  }

  Serial.begin(9600);
  
  pinMode(CLbarPin, INPUT_PULLUP);    // make pin 2 an input with pullup
  pinMode(RelayPin, OUTPUT);          // but make pin3 an output
  digitalWrite(RelayPin, LOW);        // and initialise it to LOW
  pinMode(DecrPin, INPUT_PULLUP);     // make pin 4 an input with pullup
  pinMode(OscSigPin, INPUT);          // make pin 5 an input with no pullup
  pinMode(IncrPin, INPUT_PULLUP);     // make pin 6 an input with pullup
  pinMode(CalLkPin, INPUT_PULLUP);    // and make pin 7 an input with pullup
  pinMode(CalErase, INPUT_PULLUP);    // make pin 8 an input with pullup *CJS


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  lcd_update();

  // show splash screen
  
  lcd_reset();
  lcd_update();

  lcd_setText(1,1);
  display.setCursor(15,4);
  lcd_print("Digital LC Meter");
  
  
  if(digitalRead(CalErase) == LOW) 
  {   
    display.setCursor(15,12);
    eraseEEPROM();
    CF = getEEPROM();
    lcd_print("Factory Reset!");
  }
  
  lcd_update();
  delay(500);
  display.setCursor(15,20);
  lcd_print("Starting");
  lcd_update();
  delay(500);
  for(int d=0;d<8;d++)
  {
    lcd_print(".");
    delay(500);
    lcd_update();
  }

  // do callibration and show results
  
  lcd_reset();
  lcd_setText(1,1);
   
  Serial.print("Getting stored CF: ");
  CF = getEEPROM();
  Serial.println(CF);
  
  
  if(digitalRead(CLbarPin) == LOW)
  {
    delay(500);
    display.setCursor(15,7);
    lcd_print("Set to CX mode");
    display.setCursor(15,15);
    lcd_print("and press reset!");
    lcd_update();
    while(1)  { /* no go for callibration, idle */ }
  }                               
  
  lcd_printLn("Calibration Complete!\n");
  
  
  FindC1andL1();    // now do calibration, to find F1, F2, C1 & L1
  
  DispCalData();    // then display C1 & L1 (pausing for 3 secs)
  lcd_update();
  delay(2000);
  
  // setup nudge and callibration-enable interrupts

  enableInterrupt(DecrPin, DecrementCalClBk, CHANGE);
  enableInterrupt(IncrPin, IncrementCalClBk, CHANGE);
  enableInterrupt(CalLkPin, CalLkClBk, CHANGE);
} 

 // ===================================================================
 // main loop() function begins here
 // ===================================================================
void loop()
{
  lcd_reset();
  
  
  GetFrequency();             // go get current osc frequency
  Freq3 = Fcount;             // and copy into Freq3
  float F3 = float(Freq3);    // then convert to a float
  F3sqrd = pow(F3, 2.0);      // so it can be squared
  
  
   
  if(digitalRead(CLbarPin) == HIGH)  // if we're measuring a C
  {
    measureCapacitance();
  }
  else    // CLbarPin == LOW, so we must be measuring an L
  {
    measureInductance();
  }
  
  if(digitalRead(CalLkPin) == LOW)  
  {              
    manCalibEnabled = 1;
    display.setCursor(0,25);
    lcd_setText(1,1);
    lcd_print("Man Cal Enabled:");
    lcd_printLn(String(CF,3));
  }
  else 
  {
    manCalibEnabled = 0;
  }
  lcd_update();
  //delay(50);
}     

// =====================================================================
// Callback - Calibration Lk  switch
// =====================================================================

void CalLkClBk()
{
  if(digitalRead(CalLkPin) == LOW)  
  {              
    manCalibEnabled = 1;
    display.setCursor(0,25);
    lcd_setText(1,1);
    lcd_print("Man Cal Enabled:");
    lcd_printLn(String(CF,3));
  }
  else 
  {
    manCalibEnabled = 0;
  }
}

// =====================================================================
// Callback - IncrementCalClBk()
// =====================================================================




void IncrementCalClBk() 
{
  uint32_t interrupt_time = millis();
  
  if (interrupt_time - lastDebounceTimeIncr > DEBOUNCE_DELAY) 
  {
    if(digitalRead(CalLkPin) == LOW)  
    {              
      CF = CF * 1.002;   // nudge up CF by 0.5%
      updateEEPROM(CF);

    }
  }
  lastDebounceTimeIncr = interrupt_time;
  
}

// =====================================================================
// Callback - DecrementCalClBk() 
// =====================================================================


void DecrementCalClBk() 
{
  uint32_t interrupt_time = millis();
  
  if (interrupt_time - lastDebounceTimeDecr > DEBOUNCE_DELAY) 
  {
    if(digitalRead(CalLkPin) == LOW) 
    {    
      CF = CF * 0.998;  // nudge CF down by 0.5%
      updateEEPROM(CF);
    
    }
  }
  lastDebounceTimeDecr = interrupt_time;
  
}

// =====================================================================
// measureCapacitance() 
// =====================================================================

void measureCapacitance() 
{
  String unit;
  String Cdisp;               // declare C display string
  display.setCursor(16,0);

  lcd_print("Capacitance Mode");

  
  CXval = C1val * CF * (float(F1sqrd/F3sqrd) - 1.0); // work it out
   
  if(CXval < 1.0e-9)     // if CXval < 1nF
  {
    float CXpF = CXval * pFmult;  // get equiv value in pF
    Cdisp += String(CXpF, 3);
    unit = " pF";
  }
  else    // value must be 1nF or more
  {
    if(CXval < 1.0e-6)    // but check if it's less than 1uF
    {
      float CXnF = CXval * nFmult;  // if so, get equiv value in nF
      Cdisp += String(CXnF, 3);
      unit = " nF";
    }
    else    // value must be 1uF or more
    {
      float CXuF = CXval * uFmult;  // so get equiv value in uF
      Cdisp += String(CXuF, 3);
      unit = " uF";  
    }
  }
  
  // display the cap value string and unit
  lcd_setText(1,charSize);
  
  if(manCalibEnabled) // we have limited space
  {
    display.setCursor(0,8);
    lcd_print(Cdisp);
    display.setCursor(90,8);
    lcd_print(unit);
  }
  else // we have more space
  {
    display.setCursor(0,12);
    lcd_print(Cdisp);
    display.setCursor(90,12);
    lcd_print(unit);
  }
  lcd_setText(1,1);
  
  
}

// =====================================================================
// measureInductance() 
// =====================================================================

void measureInductance() 
{
  String unit;
  String Ldisp;               // declare L display string  
  display.setCursor(16,0);
  lcd_print("Inductance Mode");
  LXval = L1val * CF * (float(F1sqrd/F3sqrd) - 1.0); // work it out

  if(LXval < 1.0e-3)     // if LXval < 1mH
  {
    float LXuH = LXval * uHmult; // convert to uH 
    Ldisp += String(LXuH, 3);
    unit = " uH";
  }
  else      // value must be 1mH or more
  {
    if(LXval < 1.5e-1)
    {
      float LXmH = LXval * mHmult; // convert to mH 
      Ldisp += String(LXmH, 3);
      unit = " mH";
    }
    else
    {
      Ldisp = "Over Range";     
    }
  }
  
  lcd_setText(1,charSize); 
  
  if(manCalibEnabled) // we have limited space
  {
    display.setCursor(0,8); 
    lcd_printLn(Ldisp);
    display.setCursor(90,8);
    lcd_print(unit);
  }
  else                // we have more space
  {
    display.setCursor(0,12); 
    lcd_printLn(Ldisp);
    display.setCursor(90,12);
    lcd_print(unit);
  }
  lcd_setText(1,1);

}


// =====================================================================
// FindC1andL1() fn: for initial calibration
// =====================================================================

void FindC1andL1()
{
  digitalWrite(RelayPin, LOW);  // make sure RLY1 is off (for C1 only)
  GetFrequency();               // go get Fosc
  Freq1 = Fcount;               // and copy into Freq1
  digitalWrite(RelayPin, HIGH); // now turn on relay to add C2
  delay(1000);                  // pause to allow osc to settle
  GetFrequency();               // then go get Fosc again
  Freq2 = Fcount;               // and copy into Freq2
  digitalWrite(RelayPin, LOW);  // then turn off relay again
  
  // now find the values of C1 and L1, knowing C2, Freq1 & Freq2
  float F1 = float(Freq1);      // first convert F1 and F2 to floats
  float F2 = float(Freq2);
  F1sqrd = pow(F1, 2.0);       // then find their squares
  F2sqrd = pow(F2, 2.0);
  float F1sqlessF2sq = (F1sqrd - F2sqrd); // & their difference
  C1val = C2val * float(F2sqrd / F1sqlessF2sq);    // then work out C1
  L1val = 1.0/(FourPiSqrd * float(F1sqrd) * C1val);  // and L1
  return; 
}  


// =====================================================================
// GetFrequency() fn: to measure IC1's current oscillating frequency
// =====================================================================

void GetFrequency()
  {
  FreqCount.begin(1000);   // start counter, with a gating time of 1 sec  
  while (!FreqCount.available())  // loop until Fcount is available
    {
    }
  Fcount = FreqCount.read();   // when it's available, read into Fcount
  FreqCount.end();        // and close off counter before leaving
  return;
  } 


// =====================================================================
// DispCalData() fn: to display calibration data (C1 and L1)
// =====================================================================

void DispCalData()
{
  float L1uH = L1val * uHmult; // find uH equivalent of L1val
  float C1pF = C1val * pFmult; // and the pF equivalent of C1val
  
  
  String C1disp = "C1= ";  // then assemble C1 display string
  C1disp += String(C1pF, 2);
  
  display.setCursor(16,15);
  lcd_print(C1disp);
  display.setCursor(100,15);
  lcd_print("pF");
  
  String L1disp = "L1= ";
  L1disp += String(L1uH, 2);
  display.setCursor(16,23);
  lcd_print(L1disp);
  display.setCursor(100,23);
  lcd_print("uH");
 
  return;         // before returning
} 

