#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Si7021.h>

#include "MemoryFree.h"

#define DEBUG 1

//sampling delay
#define CHECK_DELAY 40000

#define HI_MAX_MILLIS 2400000 //40 min
#define MED_MAX_MILLIS 2400000 //40 min

//state values
byte rv_current = 0x60;
byte rv_check   = 0x60;
byte rv_max     = 0x00;
byte rv_delta   = 0x00;
byte rv_start   = 0x00;

//current temperature
byte temp = 0x00;

//error code
byte errn = 0x00;

//switch on and switch off thresholds
#define INC_THRES 2
#define LO_THRES 2
#define MED_THRES 2

//current fan state
byte fanstate = 0;

//current free memory
int mem = 0;

//current state starting time
unsigned long start_millis = 0;

#define STATE_LO    0x01
#define STATE_MED   0x02
#define STATE_HI    0x04

#define RELAY_LO 2
#define RELAY_MED 3
#define RELAY_HI 4

#define LCD_ROWS 4
#define LCD_COLS 20
#define LCD_I2C_ADDR 0x27


/*
Libraries used:
LiquidCrystal_I2C https://github.com/marcoschwartz/LiquidCrystal_I2C
Adafruit_Si7021 https://github.com/adafruit/Adafruit_Si7021
*/

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_Si7021 sensor = Adafruit_Si7021();



void setup()
{
  Serial.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  fanstate |= STATE_LO;
  
  rv_current = sensor.readHumidity();
  //rv_max = rv_current;
  rv_check = rv_current;
  if(!sensor.begin())
  {
    Serial.println(F("Sensor error"));
    lcd.clear();
    lcd.print(F("ERROR: Sensor"));
    errn = 0x01;
  }
}


void toggle(byte pin)
{
  analogWrite(pin, 0xFF);
  delay(200);
  analogWrite(pin,0x00);
}

void fan_hi()
{
  Serial.println(F("FAN HI"));
  fanstate |= STATE_HI;
  //UNSET OTHERS
  fanstate &= ~STATE_LO;
  fanstate &= ~STATE_MED;
  start_millis = millis();
  
  toggle(RELAY_HI);
}

void fan_med()
{
  Serial.println(F("FAN MED"));
  fanstate |= STATE_MED;
  
  fanstate &= ~STATE_LO;
  fanstate &= ~STATE_HI;
  start_millis = millis();
    
  toggle(RELAY_MED);
  
}

void fan_lo()
{
  Serial.println(F("FAN LO"));
  fanstate |= STATE_LO;
  
  fanstate &= ~STATE_MED;
  fanstate &= ~STATE_HI;
  start_millis = millis();
  
  toggle(RELAY_LO);
}


void test_states()
{
  //if current humid is greater than prev humid + threshold and the fan is not in HIGH state
  if((rv_current > (rv_check + INC_THRES)) && (fanstate & ~STATE_HI))
  {
    //if fan is off, set the start humidity
    if(fanstate & STATE_LO)
    {
      rv_start = rv_check;
    }
    //turn on the fan
    fan_hi();
  }

  //if current humidity is greater than maximum measured humidity in this period:
  if((rv_current > rv_max) && (fanstate & (STATE_MED | STATE_HI)))
  {
    //increase the maximum measured humidity to the new maximum
    rv_max = rv_current;
    if(rv_start > 0) rv_delta = (rv_max - rv_start); //set the delta (absolute difference between maximum and start value)
  }

  
  if((rv_current < (rv_start + LO_THRES)) && (fanstate & STATE_MED))
  {
    rv_max = 0;//rv_current;
    fan_lo();
    rv_delta = 0;
    rv_start = 0;
  }
  
  if((rv_current < (rv_max - (rv_delta / 2))) && (fanstate & STATE_HI)) //66% instead of 50%
  {
    rv_max = rv_current;
    fan_med();
  }
  
}

void test_time()
{
  //detect wrap of millis timer
  if(millis() < start_millis)
  {
    Serial.println(F("millis wrap detected"));
    start_millis = 0;
  }
  
  //if in HI state and time expired; go to state MED
  if((fanstate & STATE_HI) && ((millis() - start_millis) >= HI_MAX_MILLIS)) fan_med();

  //if in MED state and time expired; go to state LO
  if((fanstate & STATE_MED) && ((millis() - start_millis) >= MED_MAX_MILLIS)) fan_lo();
}


void print_state()
{

  lcd.clear();
  lcd.setCursor(0,0);
  
  lcd.print(F("RV Actual:"));
  lcd.print(rv_current);
  lcd.setCursor(0,1);
  
  lcd.print(F("RV Start :"));
  lcd.print(rv_start);
  lcd.setCursor(0,2);
  
  lcd.print(F("RV Delta :"));
  lcd.print(rv_delta);
  lcd.setCursor(0,3);
  
  lcd.print(F("RV Max   :"));
  lcd.print(rv_max);
  lcd.setCursor(13,0);
  
  lcd.print(F("Fan:"));
  if(fanstate & STATE_LO) lcd.print(F("Lo"));
  if(fanstate & STATE_MED) lcd.print(F("Mid"));
  if(fanstate & STATE_HI) lcd.print(F("Hi"));
  lcd.setCursor(13,1);
  
  lcd.print(F("Temp:"));
  lcd.print(temp);
  lcd.setCursor(13,2);
  
  lcd.print(F("M:"));
  lcd.print(mem);

  lcd.print(F("E:"));
  lcd.print(errn);
  
  lcd.print(F(" A: "));
  lcd.print(start_millis);

  

}

void serial_dump()
{
  Serial.print(F("RV Actual: "));
  Serial.print(rv_current);
  Serial.print(' ');
  
  Serial.print(F("RV Start:"));
  Serial.print(rv_start);
  Serial.print(' ');
  
  Serial.print(F("RV Delta: "));
  Serial.print(rv_delta);
  Serial.print(' ');
  
  Serial.print(F("RV Max: "));
  Serial.print(rv_max);
  Serial.print(' ');
  
  Serial.print(F("Fan: "));
  if(fanstate & STATE_LO) Serial.print(F("Lo"));
  if(fanstate & STATE_MED) Serial.print(F("Mid"));
  if(fanstate & STATE_HI) Serial.print(F("Hi"));
  Serial.print(' ');
  
  Serial.print(F("Temp: "));
  Serial.print(temp);
  Serial.print(' ');
  
  Serial.print(F("Memory: "));
  Serial.print(mem);
  Serial.print(F(" bytes free"));
  
  Serial.print(F(" E:"));
  Serial.print(errn);
  
  Serial.print(F(" A: "));
  Serial.print(start_millis);
  
  Serial.println();
  
}

void process_serial_command(byte command, unsigned char *buf, byte len)
{
  switch(command)
  {
    case 1:
      rv_delta = atoi((const char *)buf);
      break;
    case 2:
      fan_hi();
      break;
    case 3:
      fan_med();
      break;
    case 4:
      fan_lo();
      break;
    case 5:
    case 6:
    default:
      break;
    
  }
}

void process_serial_input(char input)
{
  static byte command = 0;
  static byte pos = 0;
  static unsigned char buf[5];
  switch(input)
  {
    case 'd':
      command = 1;
      break;

    //fan commands
    case 'h':
      command = 2;
      break;
    case 'm':
      command = 3;
      break;
    case 'l':
      command = 4;
      break;
      
    case ';':
      process_serial_command(command, buf, pos);
      memset(buf, 0, 5);
      pos = 0;
      break;
    default:
      if(pos < 5) buf[pos++] = input;
      break;
  }
}

void delay_loop(unsigned long milliseconds)
{
  //time counter for loop delay
  unsigned long thres = millis() + milliseconds;
  while(millis() < thres)
  {
    //loop until threshold value is met
    if(Serial.available() > 0)
    {
      //process Serial events
      process_serial_input(Serial.read());
    }
  }
}

void loop()
{ 

  rv_current = sensor.readHumidity();
  temp = sensor.readTemperature();
  mem = freeMemory(); //test heap memory
  
  test_states();
  test_time();
  
  //lcd.clear();
  print_state();
  serial_dump();
  
  rv_check = rv_current;
  delay_loop(CHECK_DELAY);
}
