/**************************************************************************
  KX3ExtTuningKnob - (C) Ken Kaplan WB2ART and Mark Hatch AJ6CU - June 2022 SMD version 1.06
  Rotary encoder and cw code de W8BH.
  ***** This version is for the single board smd tuning knob pcb *****
    using Arduino 1.8.19
    STM32F1 Boards (Arduino_STM32) - Generic STM32F103C Series

   6/10/22 - removed void ledFade() as we do not need led's to fade, only be on/off/blink
             removed reference to led0Cntl as led0 is power led wired to +5v
   6/09/22 - added macro definitions for KX2 - //MJH
   6/01/22 - change some logic definitions - //MJH
   3/24/22 - added commands to clear and turn off RIT
   8/28/21 - added read/write function key strings for EEPROM
   8/16/21 - reset version to 1.01
   8/14/21 - fixed compile warning due to ButtonEvents library.properties (see below) version 6.18
   8/13/21 - added audio output (tone) pin PB4 version 6.17
             added enum rockerSw to control when cw sent if rocker switch position changes
             added morse code routines, sent when the position of the rocker switch changes
   8/02/21 - added PB4,PB5,PA9,PA10
   7/23/21 - added command to turn on RIT when rocker switch in RIT position
             added initialize Aux pins to setup
   7/22/21 - added 'fade' function to led's
   7/08/21 - added RIT UP/DOWN to tuning knob
   7/08/21 - requires ST Link to program chip
   7/04/21 - added function to flash pwr led when in bypass mode
   7/03/21 - removed function switch double tap
             added switch on rear panel to activate bypass mode (bypassAll)
   7/03/21 - new version for KX3 Remote Tuning Knob SMD version (6.0)
   1/02/21 - finished final hardware build / modified code for most recent pcb version 3.0 (version 5.0 code)
  12/26/20 - plan to add volume control - AG 000-255
  12/12/20 - added bypassAll variable. When true will only send characters to/from the KX3.
             No tuning or button pushes. This is for conflict when running WSJT-X or loading firmware.
             The rocker switch is the only switch checked.  version 4.8
             ***** IF YOU ARE LOADING FIRMWARE INTO THE KX3, PX3, OR KXPA100 MAKE SURE THIS SWITCH ACTIVTES bypassAll,
             SO THE POLLING ROUTINE IS NOT ACTIVATED *****
  11/01/20 - new tuning rate routine implemented - version 4.6
  10/26/20 - fixed tuning rate for 2m and maybe other transverter bands - version 4.5
  10/17/20 - fixed tuning rate for bands 160-6m - version 4.0
  -----------------------------------------------------------------------------------
  The MIT License (MIT)
  Copyright (c) 2022 Ken Kaplan

  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this hardware/software and associated documentation files (the PCB/"Software"), to deal in
  the PCB/Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  the PCB/Software, and to permit persons to whom the PCB/Software is furnished to do so,
  subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the PCB/Software.

  THE PCB/SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE PCB/SOFTWARE OR THE USE OR OTHER DEALINGS IN THE PCB/SOFTWARE.
  --------------------------------------------------------------------------------
  Refer to the KX3 Programmers Manual for commands

  Serial0 = usb connector (Serial)
  Serial1 = PA9-Txd  PA10-Rxd
  Serial2 = PA2-Txd  PA3-Rxd
  Serial3 = PB10-Txd PB11-Rxd

  TUNING RATES:
  UPn; DNn;  n: 0=1 Hz; 1=10 Hz; 2=20 Hz; 3=50 Hz; 4=1 kHz; 5=2 kHz; 6=3 kHz; 7=5 kHz; 8=100 Hz; 9=200 Hz.

  There does not seem to be a command in the programmers manual to return the tuning rate.

  How is the tuning rate for KX3 determined?
  A DS; command is sent to the KX3, and the response is the VFO A display - see below
  ---------------------------------------------------------------------------------------------------------------------------------------------------------
  FROM THE KX3 PROGRAMMERS MANUAL: The format of the response message is: DSttttttttaf; where tttttttt is the LCD text
  and decimal point data, a is icon data, and f is icon flash data, a and f data are ignored in this sketch.
  TEXT and decimal point data: This field contains 8 bytes, with values 0x30 - 0xFF (hex).
  The first byte is the left-most displayed character. Bit 7 (MSB) of each byte indicates whether the decimal point to
  the left of each character is on (1) or off (0). The other bits contain an ASCII character that corresponds to the displayed character.
  ---------------------------------------------------------------------------------------------------------------------------------------------------------
  The response on 160-40 meters looks like: DS@70095@@ (we only need the 'tttttttt' field, not including the 'DS')
  - the '@' signs are translated into spaces.
  We count the number of '@' signs, ignoring the first '@' sign. Then we actually only use the last 7 characters.
  The '@@' means we are in 100Hz tuning mode, so we need to send UP8; or DN8; to tune the KX3 100 Hz.
  If we are in 10Hz tuning mode, then the response is DS@700955@, and if in 1Hz tuning mode, DS@7009554.
  You can see that the number of '@' signs can be counted to determine the tuning mode.
  On 30-6 meters, there is no leading '@' sign ie: DS140065@@ (100Hz), DS1400650@ (10Hz) and DS14006505 (1Hz).

  For 2 meters, the DS response is DS1459977@, in 100Hz tuning mode. All other 100 Hz tuning modes have 2 '@' signs, however,
  if we check character 6, we see that the MSB is set (1), and there is a decimal point to the left of that position. That means
  there are 3 digits (145) in front of the decimal point. For 10 Hz tuing mode, there are still 3 digits (145) to the left of the
  decimal point, but NO '@' signs. 1 Hz tuning mode is the same as for 160-6 meters, with 2 digits to the left of the decimal point, and
  no '@'signs. There is variable called 'vhfBnd' that takes care of the tuning mode switching in the same routine as for 160-6 meters.

  Unfortunately, I cannot determine the tuning mode for any transverter band above 2 meters, as I have no capability to test those bands.
*/

// install in prefrences: http://dan.drown.org/stm32duino/package_STM32duino_index.json
#include <ButtonEvents.h> // see: https://github.com/fasteddy516/ButtonEvents and https://github.com/thomasfredericks/Bounce2
// ignore the following error from the arduino ide:
// WARNING: library ButtonsEvents-master claims to run on avr architecture(s) and may be incompatible with your current board
// which runs on STM32F1 architecture(s).
// PLEASE NOTE: To eliminate the above error, change the last line in libraries/ButtonEvents-master/library.properties from
// architectures=avr to architectures=*

#include "EEPROM.h"

//#define KX2    //define either KX2 or KX3
#define KX3

#ifdef KX3
// Default command strings for KX3
String F1tap  = "SWT12;";  // RATE  - was SWR12 for KX3
String F2tap  = "SWH12;";  // KHZ
String F3tap  = "SWT28;";  // SPOT
String F4tap  = "SWT25;SWT25;SB1;UP4;"; // A>B, A>B, DUAL WATCH ON, VFOA UP 1 - sets vfoB to receive freq, vfoA to xmit +1
String F5tap  = "SWT24;SWT25;DN4;SB0;"; // A/B, A>B, VFOA DN 1, DUAL WATCH OFF, VFOA DN 1 - sets vfoA to vfoB
String F6tap  = "SWH34;";  // PWR
String F7tap  = "SWT25;";  // A>B
String F8tap  = "SWT24;";  // A/B
String F1hold = "MN146;MP001;PC100;MN255;"; // KXPA100 ON - PWR to 100w
String F2hold = "SWH35;";  // OFS/B
String F3hold = "SWT44;";  // ATU TUNE
String F4hold = "SWH33;";  // NORM
String F5hold = "SWT08;";  // BAND +
String F6hold = "SWT41;";  // BAND -
String F7hold = "SWH25;";  // SPLIT
String F8hold = "SWH29;";  // VOX

#else

// Default command strings for KX2 // MJH
String F1tap  = "SWT08;";  // MODE Cycles thru modes
String F2tap  = "SWT41;";  // RATE switches between 100hz and 10hz steps for vfoa and vfob
String F3tap  = "SWT34;";  // KYR-SPT/MIC
String F4tap  = "SWT44;"; // A/B
String F5tap  = "SWT18;"; // RIT
String F6tap  = "SWT19;";  // PRE (/ATTN)
String F7tap  = "SWT20;";  // ATU
String F8tap  = "SWT35;";  // OFS/B
String F1hold = "SWH08"; // RCL
String F2hold = "SWT32;";  // NB
String F3hold = "SWH19;"; // NR
String F4hold = "SWH27;"; // APF/AN on/off
String F5hold = "SWT14;UP;SWT14;"; // BAND +
String F6hold = "SWT14;DN;SWT14;"; // BAND -
String F7hold = "SWH18;"; // SPLIT
String F8hold = "SWT16;"; // VOX
#endif

enum class rockerSw {NONE, VFOA, VFOB, RIT};
rockerSw tuningCmd;

int ledOn = 500; // used for led flash
int ledOff = 200; // used for led flash
int ledUpdate = 10; // used for led fade
int fadeAmount = 8; // used for led fade
int tone1 = 550; // 550Hz
int tone2 = 700;
int tone3 = 850;
int duration1 = 2000; // 2 seconds
int codeSpeed = 18;
int ditPeriod = 1200 / codeSpeed;
int pitch = 550;

int bypassPinState = HIGH;        //mjh
int vfoAPinState = HIGH;          //mjh
int ritPinState = HIGH;           //mjh

// constants should match the pcb
const byte F1 = PA1;
const byte F2 = PB8;
const byte F3 = PB7;
const byte F4 = PB6;
const byte F5 = PB1;
const byte F6 = PB5;
const byte F7 = PB4;
const byte F8 = PB12;
const byte vfoAPin = PA15;
const byte ritPin  = PB3;
const byte bypassPin = PB9;
const byte led0 = LED_BUILTIN; // PC13 red smd led
const byte led1 = PA8;  // D1 all led's green
const byte led2 = PB15; // D2
const byte led3 = PB14; // D3
const byte led4 = PB13; // D4
const byte aux1Pin = PB0;
const byte aux2Pin = PA6;
const byte aux3Pin = PA7;
const byte PA9Pin  = PA9; // Txd1
const byte PA10Pin = PA10; // Rxd1
const byte audioPin = PA11;
const byte PA12Pin = PA12; // extra pin
const byte PB2Pin = PB2; // BOOT1
const byte PA0Pin = PA0; // WAKEUP

//=================================== Encoder Hardware Connections =============================
#define ENCODER_A PA4 // Rotary Encoder output A
#define ENCODER_B PA5 // Rotary Encoder output B

String cmdStr = "$$$";
String tempcmdStr;
bool bypassAll = false;
bool sentDS = false;
bool compSending = false;
bool vhfBand = false;
bool audioGain = false;
bool sentCW = false;
int tuningRate = 1; // 8 = 100 Hz, 1 = 10 Hz, 0 = 1 Hz
int previousDir = 0;
int previousVolts = 0;
int bandNumber = 0;
int atCount = 0;
int charCount = 0;
int auxSwValue = 0;
int addrOffset = 0;
unsigned long previousMillis = 0;
const long pollingInterval = 1500; // polling interval, set faster ie:1000 or slower ie:2000 default 1500

// setup button events
ButtonEvents F1Btn;
ButtonEvents F2Btn;
ButtonEvents F3Btn;
ButtonEvents F4Btn;
ButtonEvents F5Btn;
ButtonEvents F6Btn;
ButtonEvents F7Btn;
ButtonEvents F8Btn;

class ledControl
{
    // Class Member Variables
    int ledPin;      // the number of the LED pin
    long OnTime;     // milliseconds of on-time
    long OffTime;    // milliseconds of off-time
    int brightness = 0;
    const int OnState = LOW;   //MJH Circuit uses negagive logic, Set to HIGH for positive logic
    const int OffState = HIGH;  //MJH

    // These maintain the current state for flashing
    int ledState;  // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

  public:
    void off()  //mjh
    {
      ledState = OffState;   // circuit uses negative logic, HIGH=OFF
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }  //MJH

    void on()  //mjh
    {
      ledState = OnState;   // circuit uses negative logic, LOW=ON
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }  //MJH

    ledControl(int pin, long t_on, long t_off)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = t_on;
      OffTime = t_off;

      ledState = OffState;  //MJH using constants for off and on
      previousMillis = 0;
    }

    void ledFlash()  // uses digitalWrite
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();

      if ((ledState == OnState) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = OffState;  //MJH Turn it off using constant
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }
      else if ((ledState == OffState) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = OnState;               //MJH Turn it on using constant
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(ledPin, ledState);   // Update the actual LED
      }
    }

    //MJH This in theory might need to be updated to deal with positive/negative logic. But since this fades and unfades I didn't
    //MJH want to mess with it

/*
    void ledFade() // uses analogWrite
    {
      unsigned long currentMillis = millis();

      if ((currentMillis - previousMillis) > ledUpdate) // time to update
      {
        previousMillis = currentMillis;
        brightness += fadeAmount;  // increase or decrease brightness as needed
        analogWrite(ledPin, brightness);
        // Serial.print("brightness level: ");
        // Serial.println(brightness);
        if ((brightness >= 1023) || (brightness <= 0))
        {
          fadeAmount = -fadeAmount; // reverse direction
        }
      }
    } */
}
;

//=================================== Constants =============================
#define ENCODER_TICKS 1  // Ticks required to register movement
//===================================  Rotary Encoder Variables =========================
volatile int rotaryCounter = 0;           // "position" of rotary encoder (increments CW)
//===================================  Rotary Encoder Code  =============================
/*
   Rotary Encoder Interrupt Service Routine ---------------
   It gives twice the resolution at a cost of using an additional interrupt line
   This function will run when either encoder pin A or B changes state.
   The states array maps each transition 0000..1111 into CW/CCW rotation (or invalid).
   The rotary "position" is held in rotary_counter, increasing for CW rotation, decreasing
   for CCW rotation. If the position changes, rotary_change will be set true.
   You should set this to false after handling the change.
   To implement, attachInterrupts to encoder pin A *and* pin B
*/

void rotaryISR()
{
  const int states[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  static byte transition = 0;                     // holds current and previous encoder states
  transition <<= 2;                               // shift previous state up 2 bits
  transition |= (digitalRead(ENCODER_A));         // put encoder_A on bit 0
  transition |= (digitalRead(ENCODER_B) << 1);    // put encoder_B on bit 1
  transition &= 0x0F;                             // zero upper 4 bits
  rotaryCounter += states[transition];            // update counter +/- 1 based on rotation
}

/*
   readEncoder() returns 0 if no significant encoder movement since last call,
   +1 if clockwise rotation, and -1 for counter-clockwise rotation
*/
int readEncoder(int numTicks = ENCODER_TICKS)
{
  static int prevCounter = 0;                     // holds last encoder position
  int change = rotaryCounter - prevCounter;       // how many ticks since last call?
  if (abs(change) <= numTicks)                    // not enough ticks?
    return 0;                                     // so exit with a 0.
  prevCounter = rotaryCounter;                    // enough clicks, so save current counter values
  return (change > 0) ? 1 : -1;                   // return +1 for CW rotation, -1 for CCW
}

void setupSwitches() // and debounce and hold times
{
  // Need some pins (PB3 specifically) that are normally used for jtag
  disableDebugPorts();            //mjh

  pinMode(F1, INPUT_PULLUP);
  pinMode(F2, INPUT_PULLUP);
  pinMode(F3, INPUT_PULLUP);
  pinMode(F4, INPUT_PULLUP);
  pinMode(F5, INPUT_PULLUP);
  pinMode(F6, INPUT_PULLUP);
  pinMode(F7, INPUT_PULLUP);
  pinMode(F8, INPUT_PULLUP);
  pinMode(vfoAPin, INPUT_PULLUP);
  pinMode(ritPin, INPUT_PULLUP);  //mjh
  pinMode(audioPin, OUTPUT);

  // MJH  Following pinModes are redundant because they are set in the constructor of the ledControl class
  // MJH  pinMode(led0, OUTPUT);
  // MJH  pinMode(led1, OUTPUT);
  // MJH  pinMode(led2, OUTPUT);
  // MJH  pinMode(led3, OUTPUT);
  // MJH  pinMode(led4, OUTPUT);

  F1Btn.attach(F1);
  F2Btn.attach(F2);
  F3Btn.attach(F3);
  F4Btn.attach(F4);
  F5Btn.attach(F5);
  F6Btn.attach(F6);
  F7Btn.attach(F7);
  F8Btn.attach(F8);

  F1Btn.debounceTime(10); // 10ms
  F2Btn.debounceTime(10);
  F3Btn.debounceTime(10);
  F4Btn.debounceTime(10);
  F5Btn.debounceTime(10);
  F6Btn.debounceTime(10);
  F7Btn.debounceTime(10);
  F8Btn.debounceTime(10);

  F1Btn.holdTime(1000); // 1sec
  F2Btn.holdTime(1000);
  F3Btn.holdTime(1000);
  F4Btn.holdTime(1000);
  F5Btn.holdTime(1000);
  F6Btn.holdTime(1000);
  F7Btn.holdTime(1000);
  F8Btn.holdTime(1000);
}

void setupEncoder()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), rotaryISR, CHANGE);
}

void serial1Check()
{
  if (Serial1.available()) // check if Serial 1 has sent data
  {
    int inByte = Serial1.read(); // read a character
    Serial2.write(inByte); // send character to KX3
  }
}

void serial2Check()
{
  if (Serial2.available()) // check if KX3 has sent data
  {
    int inByte = Serial2.read(); // read a character from KX3
    Serial3.write(inByte); // send character to computer
  }
}

void serial3Check()
{
  if (Serial3.available()) // check if computer has sent command
  {
    compSending = true;
    int inByte = Serial3.read();
    Serial2.write(inByte); // send command to KX3
    //Serial.println(char(inByte));
  }
  else compSending = false;
}

void initEEPROM()
{
  EEPROM.PageBase0 = 0x801F000;                   // EEPROM starts here
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;                       // STM32F103CB has 1K pages
  EEPROM.init();
}

// CW Routines
void ditSpaces(int spaces)
{
  for (int i = 0; i < spaces; i++)
    delay(ditPeriod);
}

void dit()
{
  tone(audioPin, pitch);
  ditSpaces(1);
  noTone(audioPin);
}

void dah()
{
  tone(audioPin, pitch);
  ditSpaces(3);
  noTone(audioPin);
  ditSpaces(1);
}

void sendR()
{
  dit(); dah(); dit();
}

void sendA()
{
  dit(); dah();
}

void sendB()
{
  dah(); dit(); dit(); dit();
}

// end CW Routines

// control flash time for led's
// ledControl led0Cntl(led0, ledOn, ledOff); // led0 is power on indicator wired to +5v
ledControl led1Cntl(led1, ledOn, ledOff);
ledControl led2Cntl(led2, ledOn, ledOff);
ledControl led3Cntl(led3, ledOn, ledOff);
ledControl led4Cntl(led4, ledOn, ledOff);

void setup()
{
  // initialize serial ports
  Serial.begin(38400);  // input/output to computer - serial monitor
  Serial1.begin(38400); //
  Serial2.begin(38400); // input/output to KX3
  Serial3.begin(38400); // input/output to computer

  initEEPROM();
  setupEncoder();
  setupSwitches();
  Serial2.println("AI0;"); // turn off auto-info mode

  //MJH  Switch the initiation to use the led0Cntl class
  //MJH digitalWrite(led0, HIGH); // smd red led on
  //MJH digitalWrite(led4, LOW); // blinks if bypass mode false
//  led0Cntl.off();  //mjh
  led1Cntl.off();  //mjh
  led2Cntl.off();  //mjh
  led3Cntl.off();  //mjh
  led4Cntl.off();  //mjh

  sendR();
  tuningCmd = rockerSw::NONE;
}

void loop()
{
  bypassPinState = digitalRead(bypassPin);  // MJH - Get current state of bypassPin
  if (bypassPinState == HIGH) // MJH normally pulled up
  {
    bypassAll = true; // run the normal code block

    //MJH Switched to ledControl class
    //digitalWrite(led4, LOW); // turn off led4
//    led0Cntl.on();    //MJH
    led4Cntl.off();   //MJH
  }
  else // BypassPin is LOW    ***** Pwr LED is hardwired to +5v in the smd version *****
  {
    bypassAll = false; // pass characters between computer and KX3, routine at end of code
//    led0Cntl.ledFlash(); // flash LED0 when in Bypass mode - led_builtin
    led4Cntl.ledFlash(); // flash led4 to indicate bypass mode
  }

  // led0Cntl.ledFlash();  //mjh bug? always flashing...

  if (bypassAll) // if bypassAll = true, then run entire loop
  {
    // Get current state of Rocker Switch
    vfoAPinState = digitalRead(vfoAPin);      //MJH
    ritPinState = digitalRead(ritPin);        //MJH

    // check Rocker Switch
    if (vfoAPinState == HIGH && ritPinState == HIGH) // MJH the rocker switch must be in the center position for the 2 outer pins to be HIGH
    {
      if (tuningCmd != rockerSw::VFOB) // send cw only once if rocker switch position is changed
      {
        tuningCmd = rockerSw::VFOB; // setup the tuning command for VFOB

        // MJH VFOB state is indicated by led's off,on,off (middle position, middle led...)
        led1Cntl.off();  //MJH switched to ledControl
        led2Cntl.on();   //MJH
        led3Cntl.off();  //MJH

        //MJH digitalWrite(led1, LOW); // led1 off
        //MJH digitalWrite(led2, HIGH); // led2 on
        //MJH digitalWrite(led3, LOW); // led3 off */

        sendB(); // send 'B' in cw to indicate rocker switch set to VFOB
        Serial2.println("RC;"); // clear RIT to 0A
        Serial2.println("RT0;");  // turn off RIT
      }
    }
    if (vfoAPinState == LOW)        //MJH
    {
      if (tuningCmd != rockerSw::VFOA)
      {
        tuningCmd = rockerSw::VFOA; // setup the tuning command for VFOA

        // MJH VFOA state is indicated by led's on,off,off (leftmost led, rocker in left position)
        led1Cntl.on();  //MJH switched to ledControl
        led2Cntl.off();   //MJH
        led3Cntl.off();  //MJH

        //MJH Switched to ledControl
        //MJH digitalWrite(led1, HIGH); // led1 on
        //MJH digitalWrite(led2, LOW);  // led2 off
        //MJH digitalWrite(led3, LOW);  // led3 off

        sendA; // send 'A' in cw to indicate rocker switch set to VFOA
        Serial2.println("RC;"); // clear RIT to 0
        Serial2.println("RT0;");  // turn off RIT
      }
    }
    if (ritPinState == LOW)       //MJH
    {
      if (tuningCmd != rockerSw::RIT)
      {
        tuningCmd = rockerSw::RIT; // setup the tuning command for RIT
        //MJH Switched to ledControl class
        //MJH RIT is Off, Off, On  (rocker switch towards right, only right most led lit)

        led1Cntl.off();    //MJH
        led2Cntl.off();   //MJH
        led3Cntl.on();   //MJH
        //MJH digitalWrite(led1, LOW); // led1 off
        //MJH digitalWrite(led2, LOW); // led2 off
        //MJH digitalWrite(led3, HIGH); // led3 on*/

        sendR(); // send 'R' in cw to indicate rocker switch set to RIT
        Serial2.println("RT1;");  // turn on RIT
        Serial2.println("RC;"); // clear RIT to 0
      }
    }

    // check the encoder
    int currentDir = readEncoder(); // check encoder
    if (currentDir != previousDir)  // did it move?
    {
      if (currentDir >= 1)
      {
        switch (tuningCmd)
        {
          case rockerSw::VFOA : // send the 'UP' command VFOA, step by tuning rate
            {
              Serial2.println("UP" + String(tuningRate) + ";");
              break;
            }
          case rockerSw::VFOB : // send the 'UPB' command VFOB, step by tuning rate
            {
              Serial2.println("UPB" + String(tuningRate) + ";");
              break;
            }
          case rockerSw::RIT : // send the 'UP' command RIT, step by tuning rate
            {
              Serial2.println("RU;");  // send rit up command to KX3
              break;
            }
        }
      }

      if (currentDir < 0)
      {
        switch (tuningCmd)
        {
          case rockerSw::VFOA : // send the 'DN' command VFOA, step by tuning rate
            {
              Serial2.println("DN" + String(tuningRate) + ";");
              break;
            }
          case rockerSw::VFOB : // send the 'DNB' command VFOB, step by tuning rate
            {
              Serial2.println("DNB" + String(tuningRate) + ";");
              break;
            }
          case rockerSw::RIT : // send the 'DN' command RIT, step by tuning rate
            {
              Serial2.println("RD;");  // send rit down command to KX3
              break;
            }
        }
      }
    }
    previousDir = currentDir; // store last encoder position
    // end check encoder

    // poll the KX3. Send DS; to get VFO A info. This will tell us if in 100, 10 or 1Hz tuning mode;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= pollingInterval)
    {
      //Serial.print(".");
      if (!compSending) // we don't want to poll the KX3 if the computer is polling
      {
        Serial2.println("DS;"); // send 'DS" command to KX3
        sentDS = true;
        atCount = 0;
        charCount = 0;
        vhfBand = false;
        previousMillis = currentMillis;
        // Serial.println("Polling");
      }
    }

    if (Serial2.available()) // check if KX3 has sent data
    { // ***** NEED TO PARSE FOR DIFFERENT RESPONSES *****
      int inByte = Serial2.read(); // read a character from KX3
      if (!sentDS) Serial3.write(inByte); // do not send KX3 data to computer if listening for DS; response
      int vhfByte = inByte; // we need a byte here, that does not have bit 7 cleared
      bitClear(inByte, 7);
      // Serial.write(inByte);
      // Serial.println();
      if (sentDS)
      {
        if (inByte != ';') // ';' ends the response from the KX3
        {
          ++charCount; // charCount is 0 when DS; is sent to KX3
          if (charCount >= 4 && charCount <= 10) // this line eliminates the 'DS@' as we only need bytes at 4,5,6,7,8,9,10
          {
            if (charCount == 6 && bitRead(vhfByte, 7) == 1) vhfBand = true; // if bit is set, then band = 2m (or above?)
            if (inByte == '@') ++atCount; // atCount is 0 when DS; is sent to KX3
          }
        } // end if (inbyte != ';')
        else
        {
          sentDS = false; // reset sentDS flag, because we already have complete atCount, as soon as a ; arrives.

          switch (atCount) // set tuningRate based on number of @ signs found - vhf will only have 0 or 1 '@' sign
          {
            case 1:
              { // 1 @ sign, so must be in 10Hz tuning mode
                tuningRate = 1; // Serial.println("Rate=1");
                if (vhfBand) tuningRate = 8;
                break;
              }
            case 2:
              { // 2 @ signs, so must be in 100Hz tuning mode
                tuningRate = 8;// Serial.println("Rate=8");
                break;
              }
            default:
              { // no @ signs, so must be in 1Hz tuning mode
                tuningRate = 0; //Serial.println("Rate=0");
                if (vhfBand) tuningRate = 1;
                break;
              }
          } // end switch
        } //  else, after ';' is received
      } // end if(sentDS)
    }

    serial3Check(); //  check any data sent from the computer

    // each of the following case sections has detection for switches SW1-SW8 for tap and hold functions.
    addrOffset = 0; // not used
    if (F1Btn.update())
    {
      switch (F1Btn.event())
      {
        case (tap):
          {
            Serial2.println(F1tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F1hold);
            break;
          }
      }
    }

    if (F2Btn.update())
    {
      switch (F2Btn.event())
      {
        case (tap):
          {
            Serial2.println(F2tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F2hold);
          }
      }
    }

    if (F3Btn.update())
    {
      switch (F3Btn.event())
      {
        case (tap):
          {
            Serial2.println(F3tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F3hold);
            break;
          }
      }
    }

    if (F4Btn.update())
    {
      switch (F4Btn.event())
      {
        case (tap):
          {
            Serial2.println(F4tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F4hold);
            break;
          }
      }
    }

    if (F5Btn.update())
    {
      switch (F5Btn.event())
      {
        case (tap):
          {
            Serial2.println(F5tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F5hold);
            break;
          }
      }
    }

    if (F6Btn.update())
    {
      switch (F6Btn.event())
      {
        case (tap):
          {
            Serial2.println(F6tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F6hold);
            break;
          }
      }
    }

    if (F7Btn.update())
    {
      switch (F7Btn.event())
      {
        case (tap):
          {
            Serial2.println(F7tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F7hold);
            break;
          }
      }
    }

    if (F8Btn.update())
    {
      switch (F8Btn.event())
      {
        case (tap):
          {
            Serial2.println(F8tap);
            break;
          }
        case (hold):
          {
            Serial2.println(F8hold);
            break;
          }
      }
    }
  } // end bypassAll = true

  if (not bypassAll) // bypass all the above code, just send/receive to/from computer and KX3
  {
    serial2Check();
    serial3Check();
  } // end bypassAll = false

} // end void loop()
