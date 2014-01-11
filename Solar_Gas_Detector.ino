// Solar Gas Detector //
// Written in Wiring C //

// ***** Coded by Jake Tesler ***** //

/* *************************************************************************
 *    ********* READ THESE INSTRUCTIONS! *********
 *  MOSFET (power control) must be wired to all components except the LCD screen and the Arduino
 *  MOSFET Low = power ON, MOSFET High = power OFF
 *	
 *  TODO: Before this is uploaded to an arduino, all the EEPROM values need to be set to zero...
 *  EEPROM values are a follows: 1 indicates LOW or 1.4v; 2 indicates HIGH or 5v
 *  EEPROM(1) is the current mode.
 *
 *  MQ-7 Citizen Sensor uses a G5V1 Relay
 *
 *  NOTE: When RELAY pin is HIGH, connect to 5v. When the pin is LOW, connect to 1.4v. 
 *
 * *************************************************************************
 */

#include "Arduino.h"
#include "Wire.h"
#include <EEPROM.h> //for methane bit
#include <SoftwareSerial.h>
//#include <lcdCommands.h>
#include "lcdCommands.h"
//#include "cycle.h"
#include <Adafruit_BMP085.h>
#include "MAX1704.h"

#define MOSFET 7
#define BUTTONLED 11
#define RELAY 12

volatile signed int mode = -1; //initial mode
int prevMode = 2; //mode == 0 initially (really) and this will 
                  //##confirm the check for mode accuracy...[0-1-2] cycle
volatile signed int initSet = 0; //initial settings for each mode bit
//double battLevel = 100; //set high initially for no specific reason;
//##replaced with local var below
volatile signed int warningShown = 0;


//boolean curWarnScreen = true; //T = 1; F = 2 //changed to local
//unsigned long previousWarnMillis = 0; //changed to local

unsigned long previousMillis = 0; //= 0; //is unsigned necessary?????
unsigned long MQinterval = 90000; //interval at which to switch relay(ms) [dynamic]
//##do NOT set MQinterval value here...leave at zero
//##why? we should set it to the initial values specified at LOW
int MQrelayState = LOW;

int airInfoSwitch = 0; //air pollution info (LCD line 4) switch var
static int airInfoSwitchMultiplier = 6; //1.5sec

int romAddr; //random # (not 0 or 1 [reserved])...will be set during setup
//SoftwareSerial myLCD(5,4); //5 RX, 4 TX //redefined in lcdCommands.h

signed int animStep = -17; //tracker for confusedAnimation()
static signed int initAnimStep = -17;

Adafruit_BMP085 bar; //I2C - initialize barometric sensor using library
MAX1704 fuelGauge; //I2C - initialize fuel gauge

static int ledLuxLevel = 200; //button led brightness

void setup()
{
  //delay(5000);
//pinMode(2, INPUT); //for interrupt 0 (this is the mode button)
  pinMode(2, INPUT_PULLUP); //for interrupt 0 (this is the mode button)
//pinMode(4, OUTPUT); //LCD RX/TX is on pin 4
  pinMode(5, OUTPUT); //SDA/SCL Relay Control #PWM
  pinMode(MOSFET /*(pin 7)*/, OUTPUT); //MOSFET for power control (pin 7)
  pinMode(9, OUTPUT); //Buzzer #PWM
  pinMode(BUTTONLED /*(pin 11)*/, OUTPUT); //LED for mode button (pin 11) #PWM
  pinMode(RELAY, OUTPUT); //MQ7 (Carbon Monoxide) Relay (pin 12) #PWM
  pinMode(13, OUTPUT); //Builtin LED
  pinMode(A2, INPUT); //MQ4 Data (CH4, CNC) #PWM-S
  pinMode(A3, INPUT); //MQ7 Data (CO, H) #PWM-S
//pinMode(A4, INPUT); //I2C SDA #PWM-S
//pinMode(A5, INPUT); //I2C SCL #PWM-S

//digitalWrite(2, HIGH); //button power (un-float)
  analogWrite(BUTTONLED, ledLuxLevel); //turn on LED initially
  digitalWrite(13, HIGH);
  digitalWrite(RELAY, LOW); //start at collection (1.4v)
  digitalWrite(5, HIGH); //start SDA/SCL batt monitor relay, leave on permanently
  
  myLCD.begin(9600); //initializes LCD at 9600 baud
  Serial.begin(9600); //necessary?
  Serial.println("boot");
  Wire.begin(); //????
  //LCDturnDisplayOn(); //elsewhere
  //LCDclearDisplay();  //elsewhere
  bar.begin();
  Serial.println("bar");
  fuelGauge.reset();
  fuelGauge.quickStart();
  Serial.println("fuelGauge");
  
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  /*
  LCDbrightness(255);
  LCDturnDisplayOn(); //elsewhere
  LCDclearDisplay(); //clears LCD
  LCDsetPosition(1,1); //splash
  myLCD.print(" The COM-BAT System ");
  LCDsetPosition(2,1);
  myLCD.print("Built by Jake Tesler");
  LCDsetPosition(4,1);
  myLCD.print("Loading...");
  */
  
  //update the EEPROM to the current address (1 + previous address, overflow at 255 back to 2)
  //store the new address in addr(0)
  //valid addresses range from 2-255 (0/1 reserved)
  //romZero will store the address used by the previous power cycle, romAddr will store the current address
  //These two variables will remain like this forever...romZero must store the previous address for this to function
  int romZeroAddr = 1  //changed from zero to extend longevity
  int romZero = EEPROM.read(romZeroAddr); //remember that romZero will store the PREVIOUSLY used address
  if (romZero >= 255 || romZero < 2) //if address in 0 is >= 255 (max EEPROM byte value) or less than 2 (our min byte value)
  {
    romAddr = 2; //set EEPROM address to 2 (0/1 resered)
    EEPROM.write(romAddr, 1); //Write to new address value of 1 (relay LOW)...best to start safe
    EEPROM.write(romZeroAddr, 2); //reset address 0 (or zeroaddr) to value 2 (which is now the current address)
  }
  else //if previous address is less than 255 (normal EEPROM value)
  {
    romAddr = romZero+1; //set EEPROM current address to (one + previous address)
    EEPROM.write(romAddr, EEPROM.read(romZero)); //set romAddr to value of previous address
    EEPROM.write(romZeroAddr, romAddr); //increase stored address in (address 0) by one (set to current address)
    Serial.println();
    Serial.println("romstore complete");
  }

  //set the relay based on the EEPROM value
  if(EEPROM.read(romAddr) == 2) //if current value (which was the previous value) is 2 !(was not this value last time)
      {
        EEPROM.write(romAddr, 1);
        MQrelayState = LOW; //set relay to LOW or 1.4v
      }
  else if(EEPROM.read(romAddr) == 1)
  {
    MQrelayState = HIGH; //set relay to HIGH or 5v
    MQinterval = 60000;
    EEPROM.write(romAddr, 2);
  }
  else //if romAddr is not 1 or 2
  {
    MQrelayState = LOW; //proceeding with caution is always a good idea
    EEPROM.write(romAddr, 1);
  }

  
  LCDturnDisplayOn(); //elsewhere
  LCDclearDisplay(); //clears LCD
  LCDsetPosition(1,1); //splash
  myLCD.print(" The COM-BAT System ");
  LCDsetPosition(2,1);
  myLCD.print("Built by Jake Tesler");
  LCDsetPosition(4,1);
  myLCD.print("Loading");
  /*for(int itime = 0; itime <= 1; itime++)
  { 
    LCDsetPosition(4,8);
    myLCD.print("   ");
    int curDot = 0;
    for(int pixeldot = 1; pixeldot < 4; pixeldot++);
    {
      LCDsetPosition(4, curDot+8);
      myLCD.print(".");
      curDot++;
      delay(300);
    }
    
    delay(300);
  }*/
  int dotdelay = 100;
  LCDsetPosition(4,8);
  myLCD.print(".");
  delay(dotdelay);
  LCDsetPosition(4,9);
  myLCD.print(".");
  delay(dotdelay);
  LCDsetPosition(4,10);
  myLCD.print(".");
  delay(dotdelay);
  LCDsetPosition(4,8);
  myLCD.print("   ");
  delay(dotdelay);
  LCDsetPosition(4,8);
  myLCD.print(".");
  delay(dotdelay);
  LCDsetPosition(4,9);
  myLCD.print(".");
  delay(dotdelay);
  LCDsetPosition(4,10);
  myLCD.print(".");
  
  //cyclone(3, 1000, 4, 12);
  delay(500);
  
  
  //A funky loading animation
  LCDsetPosition(4,17);
  myLCD.write(0x4F);
  myLCD.print("_o");
  delay(167);
  LCDsetPosition(3,17);
  myLCD.print("_ _");
  delay(333);
  LCDsetPosition(3,19);
  myLCD.print("-");
  delay(167);
  LCDsetPosition(4,20);
  myLCD.print("?");
  delay(333);
  LCDsetPosition(3,19);
  myLCD.print("_");
  delay(167);
  LCDsetPosition(3,19);
  myLCD.print("-");
  delay(333);
  
  LCDclearDisplay();
  delay(100);
  LCDclearScreenFull();
  
  digitalWrite(MOSFET, LOW);
  
  //mode = EEPROM.read(1); //#### DO NOT!!! ##RE-ENABLE THIS FOR FINAL PRODUCT VERSION //The mode should reset, not pick up from where left off. 
                           //Not worth it to have a permenant mode stored, just start from scratch on reboot.
  
  warningShown = 0;  //initialize warning system
  //attachInterrupt(0, interrupt0, LOW); //digital pin 2 //should be last in setup
  buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
  delay(250);
  buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
  attachInterrupt(0, interrupt0, FALLING); //digital pin 2 //should be last in setup
  //attachInterrupt(0, interrupt0, CHANGE); //digital pin 2
}


void loop() {

//##DEBUG
  Serial.print("MODE ");
  Serial.println(mode);
  //Serial.println(fuelGauge.stateOfCharge());
  Serial.print("INITSET ");
  Serial.println(initSet);
  Serial.println();
  
//if (mode > 2) { mode=0; EEPROM.write(1,0); } //if mode is outside # of modes (currently 3 modes)

  if (mode == -1)
  {
    buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
    delay(200);
//  buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
    mode = 0;
  }
  
  if (mode != 0)
    {
      digitalWrite(MOSFET, LOW); //turn on to power components
      //Serial.println("mosfet low");
    }

  //set title bar...mode titles cannot be longer than 14char

  //GLOBAL BATTERY STAT
  
  double battLevel = fuelGauge.stateOfCharge();
  //Serial.print("global ");
  //Serial.println(battLevel);
  if (battLevel < 100 && battLevel >= 10)
  {
    LCDsetPosition(1,15);
    myLCD.print("B"); //batt logo;
    LCDsetPosition(1,16); //16
    myLCD.print(battLevel,1); //[,1]=.1 (include decimal)
    LCDsetPosition(1,20);
    myLCD.print("%");
  }
  else if (battLevel < 10 && battLevel > -0.01)
  {
    LCDsetPosition(1,15);
    myLCD.print("B0"); //batt logo;
    LCDsetPosition(1,17); //16+1 for B"zero"
    myLCD.print(battLevel,1); //[,1]=.1 (include decimal)
    LCDsetPosition(1,20);
    myLCD.print("%");
  }
  else if(battLevel >= 100 && battLevel < 255)
  {
    LCDsetPosition(1,15);
    myLCD.print("B FULL"); //batt logo;
    /*LCDsetPosition(1,16);
    myLCD.print("FULL");*/
  }
  else if(battLevel >= 255)
  {
    LCDsetPosition(1,15);
    myLCD.print("PW_ERR");
  }
  else if(battLevel <= -0.01)
  {
    LCDsetPosition(1,15);
    myLCD.print("PWE_LW");
  }
  else
  {
    LCDsetPosition(1,15);
    myLCD.print("B_SNSR");
  }

  if (mode == 1) { mode1(); } //gas sensors1
  if (mode == 0); //off-recharge0 //##I think the semicolon was a typo, but the device works great with it, so leave it in.
  { 
    mode0();
  }
  if (mode == 2) { mode2(); } //atmospheric mode2
  if (mode == 3) { mode3(); } //initial science warning

  //THIS IS NOW A GLOBAL CHECK
  //MQ7 (CO) relay switcher
  //noInterrupts();
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > MQinterval) { //if previousMillis diff+ exceeds allowed interval
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    if (MQrelayState == HIGH)
    {
      MQrelayState = LOW; //1.4v
      MQinterval = 90000; //set new interval for LOW period (90sec)
      EEPROM.write(romAddr, 1);
    }
    else
    {
      MQrelayState = HIGH; //5v
      MQinterval = 60000; //set new interval for HIGH period (60sec)
      EEPROM.write(romAddr, 2);
    }
    digitalWrite(RELAY, MQrelayState); //write digital pin 6 (relay control)
    //interrupts();
  }

  delay(500); //delay until next global event  
} //END void loop()




void mode0() //off-recharge
{
  if (mode == 0 || mode == -1) //correction for always displayed
  {
   if (initSet != 1)
   {
     initSet = 1; //should be first because of interrupt possibility
      //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0
   
     if (prevMode == 0 || prevMode == 1) //(prevMode != 2 || prevMode != 3) //if abnormal behavior
     {
       mode = 2; //go back
       initSet = 0;
     }
     else
     {
       prevMode = 0; //else set to current mode
     }
     
      LCDclearDisplay();
                                //delay(100);
      LCDsetPosition(1,1);
      myLCD.print("OFF/CHG");
      //digitalWrite(MOSFET, HIGH); //turn off power to components
      //Serial.println("mosfet high");
      EEPROM.write(1,0);
      animStep = initAnimStep;
      
      
      //delay(50);
      analogWrite(BUTTONLED, 0);
      delay(250);
      interrupts();
      analogWrite(BUTTONLED, ledLuxLevel);
      //interrupts();
      
      if((millis()/1000) > 15) //alive for more than 10 sec
      {
        buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
      }
     
  }
    /*
    LCDsetPosition(3, 8);
    myLCD.print(battLevel0, 1);
    myLCD.print("%");
    */
    
    double battLevel0 = fuelGauge.stateOfCharge();
    
    if(battLevel0 >= 254) //battery disconnected error
    {
      LCDsetPosition(2,1);
      myLCD.print("BATTERY ERROR:");
      LCDsetPosition(3,1);
      myLCD.print("Upper Out of Range");
      Serial.println("Battery Error");
    }
    else if(battLevel0 < 101 && battLevel0 >= 0) //normal battery
    {
      confusedAnimation(animStep);
      animStep++;
    }
    else if (battLevel0 < 0) //unknown sub-zero data reported
    {
      LCDsetPosition(2,1);
      myLCD.print("BATTERY ERROR:");
      LCDsetPosition(3,1);
      myLCD.print("Lower Out of Range");
      Serial.println("Battery Error");
    }
    
    if(MQrelayState == LOW) //1.4v
    {
      LCDsetPosition(4,15);
      myLCD.print(" L1.4v");
    }
    else //if 5.0v
    {
      LCDsetPosition(4,15);
      myLCD.print(" H5.0v");
    }
    
    
    LCDsetPosition(4, 13); //display relay time remaining
    if((MQinterval-(millis() - previousMillis))/1000 <= 9) //timing position correction on LCD for sec0-9
    {
      //LCDsetPosition(4, 13);
      myLCD.print("0");
      //myLCD.print((MQinterval-(millis() - previousMillis))/1000,1);
    }
    else
    {
      //LCDsetPosition(4, 13);
      //myLCD.print((MQinterval-(millis() - previousMillis))/1000,1);
    }
    myLCD.print((MQinterval-(millis() - previousMillis))/1000,1);
    
    //Uptime reporting
    LCDsetPosition(4, 1);
    myLCD.print("Up: ");
    //myLCD.print(millis());
    //LCDsetPosition(4, 4);
    if(millis()/1000 >= 60) {
      myLCD.print((millis()/1000)/60,1);
      myLCD.print("m");
    }
    myLCD.print(((millis()/1000) % 60),1);
    myLCD.print("s");
    
    
      //double curBarTemp = (((bar.readTemperature()) * 1.8) + 32);
    double curBarTemp = bar.readTemperature();
    curBarTemp = ((curBarTemp * 1.8) + 32);
    LCDsetPosition(3,10);
    if (curBarTemp > -200)
    {
      LCDsetPosition(2,15);
      myLCD.print(curBarTemp, 1); //prints the temperature to LCD with one decimal place
      myLCD.write(0b11011111);
      myLCD.print("F");
    }
    else
    {
      myLCD.print("Temp Error");
    }
  }
  /*else
  {
    interrupt0(); //crash protection of some sort?
  }*/  
}


void mode1() //AIR SENSOR
{
  if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility
      //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0()
     
      if (prevMode == 1 || prevMode == 2) //(prevMode != 0 || prevMode != 3)
      {
        mode = 0;
        initSet = 0;
        //return;
      }
      else ////if (prevMode == 0)
      {
        prevMode = 1;
      }
      //there used to be an end curly bracket here ############################################ THIS IS AN INITSET TEST HERE FOR ENCAPSULATING THE ENTIRE LAUNCH INTO THE IF STATEMENT    
      LCDclearDisplay();
      delay(50);
      LCDsetPosition(3,1);
      myLCD.print("                    ");
      /*
      LCDsetPosition(1,1);
      myLCD.print("                    ");
      LCDsetPosition(1,1);
      myLCD.print("Air Sensors   ");

      LCDsetPosition(2,1);
      myLCD.print("CO: ");
      LCDsetPosition(2,10);
      myLCD.print(" ppm");

      LCDsetPosition(3,1);
      myLCD.print("CH4: ");
      LCDsetPosition(3,18);
      myLCD.print("ppm"); 
*/
      LCDsetPosition(4,1);
      myLCD.print("LOADING...");

      //EEPROM.write(1,1);

      analogWrite(BUTTONLED, 0);
      delay(200);
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
      //delay(250);
      analogWrite(BUTTONLED, 0);
      delay(200);
      interrupts();
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms

      //      LCDsetPosition(4,1);
      //      myLCD.print("  20-2k || 200-10k  "); //MQ7 (CO) || MQ4 (CH4);
      
      //interrupts(); //re-enable after interrupt0() interrupts shutdown
      
    

      LCDsetPosition(1,1);
      myLCD.print("Air Sensors   ");

      LCDsetPosition(2,1);
      myLCD.print("CO: ");
      LCDsetPosition(2,10);
      myLCD.print(" ppm");

      LCDsetPosition(3,1);
      myLCD.print("CH4: ");
      LCDsetPosition(3,11);
      myLCD.print("ppm"); 
    }
    
    //cycle air sensor information
    if (airInfoSwitch == (airInfoSwitchMultiplier*3)) { airInfoSwitch = 0; }
    if (airInfoSwitch < (airInfoSwitchMultiplier*3) && airInfoSwitch >= (airInfoSwitchMultiplier*2))
    {
      LCDsetPosition(4,1);
      myLCD.print("CO read at 1.4v ONLY");
      airInfoSwitch += 1;
    }
    if (airInfoSwitch < (airInfoSwitchMultiplier*2) && airInfoSwitch >= airInfoSwitchMultiplier)
    {
      LCDsetPosition(4,1);
      //myLCD.print("ppm: 20-2k | 200-10k"); //MQ7 (CO) | MQ4 (CH4)
      myLCD.print("ppm: 20-2k - 200-10k"); //MQ7 (CO) | MQ4 (CH4)
      airInfoSwitch += 1;
    }
    if (airInfoSwitch < airInfoSwitchMultiplier)
    {
      LCDsetPosition(4,1);
      myLCD.print("MQ7 (CO) - MQ4 (CH4)"); //ppm: 20-2k | 200-10k  
      airInfoSwitch += 1;
    }
    
    //Serial.print("airInfoSwitch ");
    //Serial.println(airInfoSwitch);

    //gas sensors
    
    //Serial.println(val,DEC);//Print the value to serial port

    if(MQrelayState == LOW) //1.4v
    {
      LCDsetPosition(2,6);
      myLCD.print(analogRead(2), DEC); //read/print gas value from analog 2 (CO)
      LCDsetPosition(3,6);
      myLCD.print(analogRead(1), DEC);//Read/print gas value from analog 1 (CH4)
      LCDsetPosition(2,14);
      myLCD.print(" ");
      LCDsetPosition(2,16);
      myLCD.print(" 1.4v");
      LCDsetPosition(3,17);
      myLCD.print("    ");
    }
    else
    {
      /*LCDsetPosition(2,14);
      myLCD.print("X");
      LCDsetPosition(2,16);
      myLCD.print("H+ 5v");*/
      LCDsetPosition(2,14);
      myLCD.print("X H+ 5v");
      LCDsetPosition(3,17);
      myLCD.print("HOLD");
    }
    
    /*
    LCD should read like this:
     12345678901234567890
     1  Air Sensor Data     
     2  CO: xxxxx ppmX H+ 5v   _OR_
     2  CO: xxxxx ppm   1.4v
     3  CH4: xxxxx ppm      
     4  MQ7 (CO) | MQ4 (CH4)   _OR_
     4  ppm: 20-2k | 200-10k   _OR_
     4  CO read at 1.4v ONLY
     */
}




void mode2() //ATMOSPHERIC
{
  if (initSet != 1)
    {
      initSet = 1; //should be first because of interrupt possibility
      //noInterrupts(); //CANNOT BE HERE, ALREADY IN interrupt0
      if (prevMode != 1)
      {
        mode = 1;
        initSet = 0;
      }
      else
      {
        prevMode = 2;
      }
      
      LCDclearDisplay();
      delay(50);
      LCDclearScreen();
      delay(100);
      
      LCDsetPosition(4,1);
      myLCD.print("LOADING...");
      
      //EEPROM.write(1,2);
  
      analogWrite(BUTTONLED, 0);
      delay(200);
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
      //delay(250);
      analogWrite(BUTTONLED, 0);
      delay(200);
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
      //delay(250);
      analogWrite(BUTTONLED, 0);
      delay(200);
      interrupts();
      analogWrite(BUTTONLED, ledLuxLevel);
      buzz(9,400,200);  //buzz on pin 9 at 400hz for 200ms
      
      
      LCDclearScreen();
      delay(50);
      LCDclearScreenFull();
      
      
      LCDsetPosition(1,1);
      myLCD.print("              ");
      LCDsetPosition(1,1);
      //myLCD.write(0x41); //A
      //myLCD.write(0x74); //t
      //myLCD.write(0x6D); //m
      myLCD.print("Atmospheric   ");
      LCDsetPosition(2,1);
      myLCD.print("Temp: ");
      LCDsetPosition(3, 1);
      myLCD.print("Pressure: ");
      LCDsetPosition(4,1);
      myLCD.print("          ");
      LCDsetPosition(4,1);
      //myLCD.write(0x41); //A
      //myLCD.write(0x6C); //l
      myLCD.print("Altitude:  ");
      
      //interrupts();
      //delay(100);
    }
    
    
  //double curBarTemp = (((bar.readTemperature()) * 1.8) + 32);
  double curBarTemp = bar.readTemperature();
  curBarTemp = ((curBarTemp * 1.8) + 32);
  LCDsetPosition(2,7);
  if (curBarTemp > -200)
  {
    LCDsetPosition(2,7);
    myLCD.print(curBarTemp, 1); //prints the temperature to LCD with one decimal place
    myLCD.write(0b11011111);
    myLCD.print("F");
  }
  else { myLCD.print("Temp Error"); }

  
  if(MQrelayState == LOW) //1.4v
  {
    LCDsetPosition(2,17);
    myLCD.print("1.4v");
  }
  else
  {
    LCDsetPosition(2,17);
    myLCD.print("H 5v");
  }
  
  
  double curBarPres = bar.readPressure();
  LCDsetPosition(3,11);
  if (curBarPres < 115000)
  {
    myLCD.print(curBarPres, 0);
    myLCD.print(" Pa ");
  }
  else { myLCD.print(">1k kPa [E"); }


  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  LCDsetPosition(4,11);
  //double curBarAlt = (bar.readAltitude() * 3.2808);
  double curBarAlt = (bar.readAltitude(101325) * 3.2808);
  if (curBarAlt < 10000 && curBarAlt > -2000)
  {
    myLCD.print((curBarAlt), 1);
    myLCD.print(" Ft."); //(" Feet ");
  }
  else { myLCD.print(">10k Feet"); }
  
  
  //Serial.println(curBarTemp);
  //Serial.println(curBarAlt);
}

void mode3()
{
  if (prevMode != 0) //if abnormal behavior
  {
    mode = 0; //go back
    initSet = 0;
  }
  if (prevMode == 0) // else
  {
    //prevMode = 3; //else set to current mode //RECTIFIED, "MODE 3" ISN'T A MODE!!
    warningShown = 1; //to activate next step when interrupt is called
    //interrupts();
    //initSet = 0;
    unsigned long previousWarnMillis = millis();
    //previousWarnMillis = millis();
    boolean curWarnScreen = true; //T = 1; F = 2
    interrupts();
    while(warningShown == 1)
    {
      //interrupts(); //moved up
      if (curWarnScreen)
      {
        LCDsetPosition(1,1);
        myLCD.print("WARNING: Do not use this device for scie-ntific purposes, as");
        LCDsetPosition(4,1);
        myLCD.print("Press to continue...");
      }
      else //if (!(curWarnScreen)) //
      {
        LCDsetPosition(1,1);
        myLCD.print("it may sometimes    ");
        LCDsetPosition(2,1);
        myLCD.print("yield inaccurate or imprecise data.     ");
      }
      
      if (millis() - previousWarnMillis > 2500)
      {
        curWarnScreen = !(curWarnScreen);
        previousWarnMillis = millis();
      }
    }
    
    /*
    while(warningShown == 1)
    {
      interrupts();
      LCDsetPosition(1,1);
      myLCD.print("WARNING: Do not use this device for scie-ntific purposes, as");
      LCDsetPosition(4,1);
      myLCD.print("Press to continue...");
      delay(2500);
      
      LCDsetPosition(1,1);
      myLCD.print("it may sometimes    ");
      LCDsetPosition(2,1);
      myLCD.print("yield inaccurate or imprecise data.     ");
      delay(2500);
    }
    */
  }
}

volatile unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = millis();
void interrupt0()
{
  //debounce protection
  
  interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    noInterrupts();
    if(mode != -1) { buzz(9,500,200); } //buzz on pin 9 at 500hz for 200ms
    
    digitalWrite(13, LOW);
    //digitalWrite(BUTTONLED, LOW);
    analogWrite(BUTTONLED, ledLuxLevel);
  
  if (warningShown == 0) //if just pressed for very first time
  {
    mode = 3; //send to warning screen when loop() will call mode3();
  }
  else if (warningShown == 1) //if TOC just confirmed/accepted [from warning screen]
  {
    mode = 1;
    warningShown = 2;
    prevMode = 0;
    //prevMode = 3; ##previous mode would be 0, since technically "mode 3" isn't really a mode
    Serial.println("switchWarnShown1");
  }
  else if(mode == 1 && warningShown == 2)
  {
    mode = 2;
    Serial.println("switchto2");
  }
  else if(mode == 0 && warningShown == 2)
  {
    mode = 1; 
    Serial.println("switchto1");
  }
  else if(mode == 2 && warningShown == 2)
  {
    mode = 0;
    Serial.println("switchto0");
  }
  else
  {
    mode = 0;
    Serial.println("switchfrom_elseto0");
  } 
  
  initSet = 0;
  Serial.print("INITSET ");
  Serial.println(initSet);
  //digitalWrite(BUTTONLED, HIGH);
  analogWrite(BUTTONLED, ledLuxLevel);
  digitalWrite(13, HIGH);
  
  }/*
  else //THIS CANNOT EXIST UNLESS ACTUALLY PLUGGED INTO USB, CAUSES BUGS OTHERWISE
  {
    Serial.println("buttonPressRegistered");
  }
  Serial.println(last_interrupt_time);*/
  last_interrupt_time = interrupt_time;
}


void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000/frequency/2; // calculate the delay value between transitions
  //## 1 second's worth of microseconds, divided by the frequency, then split in half since
  //## there are two phases to each cycle
  long numCycles = frequency * length/ 1000; // calculate the number of cycles for proper timing
  //## multiply frequency, which is really cycles per second, by the number of seconds to 
  //## get the total number of cycles to produce
  for (long i=0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}





void confusedAnimation(int curStep)
{
  if (curStep == -17)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -16)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -15)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing.");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -14)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing..");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -13)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -12)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
  }
  else if (curStep == -11)
  {
    LCDsetPosition(2,1);
    myLCD.print("Introducing...");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -10)
  {
    LCDsetPosition(2,1);
    myLCD.print("                    ");
    LCDsetPosition(2,1);
    myLCD.print("Big");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -9)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -8)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -7)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
  }
  else if (curStep == -6)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("---");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
    LCDsetPosition(3,10);
    myLCD.print("--");
  }
 else if (curStep == -5)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-----");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,8);
    myLCD.print("----");
  }
  else if (curStep == -4)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
//    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  else if (curStep == -3)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  else if (curStep == -2)
  {
    LCDsetPosition(2,1);
    myLCD.print("Big-Eyed Bill!");
//    LCDsetPosition(3,1);
//    myLCD.print("-------");
    myLCD.write(0x7E);
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);     
    myLCD.print("o ");
    LCDsetPosition(3,5);
    myLCD.write(0x7F);
    myLCD.print("------");
  }
  
  else if (curStep == -1)
  {
    LCDsetPosition(2,1);
    myLCD.print("              ");
    LCDsetPosition(3,1);
    myLCD.print("              ");
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  
  else if(curStep == 0)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 1)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 2)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 3)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 4)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 5)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 6)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 7)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 8)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 9)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 10)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 11)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 12)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 13)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 14)
  {
    LCDsetPosition(2,1);
    myLCD.print("_ -");
  }
  else if (curStep == 15)
  {
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 16)
  {
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 17)
  {
    LCDsetPosition(3,4);
    myLCD.print(" ");
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 18)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 19)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 20)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 21)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x30);
    myLCD.print("_");
    myLCD.write(0xDB);
  }
  else if (curStep == 22)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 23)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 24)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 25)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 26)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 27)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 28)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xF2);
    myLCD.print("_");
    myLCD.write(0xEF);
  }
  else if (curStep == 29)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 30)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 31)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 32)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 33)
  {
    LCDsetPosition(3,1);
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 34)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 35)
  {
    LCDsetPosition(3,1);
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 36)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xFC);
    myLCD.write(0xA3);
    myLCD.write(0x2A);//another one
  }
  else if (curStep == 37)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 38)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x23);//another one
    myLCD.write(0xA3);
    myLCD.write(0x5E); //another one
  }
  else if (curStep == 39)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 40)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x22);//another one
    myLCD.write(0xA3);
    myLCD.write(0xEC); //another one
  }
  else if (curStep == 41)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 42)
  {
    LCDsetPosition(3,1);
    myLCD.write(0xE0);//another one
    myLCD.write(0xA3);
    myLCD.write(0xF4); //another one
  }
  else if (curStep == 43)
  {
    LCDsetPosition(3,1);
    //myLCD.print("-_-");
    myLCD.print("-");
    myLCD.write(0xA3);
    myLCD.print("-");
  }
  else if (curStep == 44)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.write(0xA3);
    myLCD.print("o");
  }
  else if (curStep == 45)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 46)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 47)
  {
    LCDsetPosition(2,1);
    myLCD.print("- -");
  }
  else if (curStep == 48)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 49)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 50)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 51)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 52)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 53)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 54)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print("?");
  }
  else if (curStep == 55)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(3,4);
    myLCD.print(" ");
  }
  else if (curStep == 56)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    LCDsetPosition(2,1);
    myLCD.print("_ _");
  }
  else if (curStep == 57)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 58)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 59)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 60)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 61)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 62)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 63)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 64)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 65)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 66)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_-");
  }
  else if (curStep == 67)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 68)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 69)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 70)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH");
  }
  else if (curStep == 71)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 72)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 73)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 74)
  {
    LCDsetPosition(3,1);
    myLCD.print("-_- SIGH...");
  }
  else if (curStep == 75)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 76)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o SIGH...");
  }
  else if (curStep == 77)
  {
    LCDsetPosition(3,1);
    //myLCD.print("0_o");
    myLCD.print("           ");
    LCDsetPosition(2,1);
    myLCD.print("   ");
  }
  else if (curStep == 78)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
  }
  else if (curStep == 79)
  {
    LCDsetPosition(3,1);
    myLCD.write(0x4F);
    myLCD.print("_o");
    animStep = initAnimStep; //global var
  }
  else
  {
    curStep = initAnimStep;
    animStep = initAnimStep;
  }
}





/* SWITCH HOLD [DEBOUNCE?] CODE?????
 int varSWITCH_PIN = 0;  //variable to store switch presses
 
 if (digitalRead(SWITCH_PIN) == LOW)
 {
 time = millis();
 delay(200); //debounce
 
 // check if the switch is pressed for longer than 1 second.
 if(digitalRead(SWITCH_PIN) == LOW && time - millis() >1000) 
 
 {
 varSWITCH_PIN++;  //add 1 Step to next Mode in setup
 if(varSWITCH_PIN==8){varSWITCH_PIN=0;}       //switch back to 0 after the required modes
 
 // if it is a short press <1000 
 } else
 do something  
 
 }
 */

