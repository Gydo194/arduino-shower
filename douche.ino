#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Si7021.h>

#include "MemoryFree.h"

#define DEBUG 1

//sampling delay
#define CHECK_DELAY 40000//15000

//state values
byte rv_current = 0x60;
byte rv_check   = 0x60;
byte rv_max     = 0x00;
byte rv_delta   = 0x00;
byte rv_start   = 0x00;

//current temperature
byte temp = 0x00;

//switch on and switch off thresholds
#define INC_THRES 2
#define LO_THRES 2

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
    while(1) ; //stop
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
  
  toggle(RELAY_HI);
}

void fan_med()
{
  Serial.println(F("FAN MED"));
  fanstate |= STATE_MED;
  
  fanstate &= ~STATE_LO;
  fanstate &= ~STATE_HI;
  
  toggle(RELAY_MED);
  
}

void fan_lo()
{
  Serial.println(F("FAN LO"));
  fanstate |= STATE_LO;
  
  fanstate &= ~STATE_MED;
  fanstate &= ~STATE_HI;
  
  toggle(RELAY_LO);
}


void test_states()
{
  //Serial.println(F("test"));
  if((rv_current > (rv_check + INC_THRES)) && (fanstate & ~STATE_HI))
  {
    if(fanstate & STATE_LO)
    {
      rv_start = rv_check;
    }
    fan_hi();
  }
  
  if((rv_current > rv_max) && (fanstate & (STATE_MED | STATE_HI)))
  {
    rv_max = rv_current;
    if(rv_start > 0) rv_delta = (rv_max - rv_start);
  }
  
  if((rv_current < (rv_start + LO_THRES)) && (fanstate & STATE_MED))
  {
    rv_max = 0;//rv_current;
    fan_lo();
    rv_delta = 0;
    rv_start = 0;
  }
  
  if((rv_current < (rv_max - (rv_delta / 2))) && (fanstate & STATE_HI))
  {
    rv_max = rv_current;
    fan_med();
  }
  
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
  Serial.println();
}


void loop()
{ 
  rv_current = sensor.readHumidity();
  temp = sensor.readTemperature();
  mem = freeMemory(); //test heap memory
  
  test_states();
  
  //lcd.clear();
  print_state();
  serial_dump();
  
  rv_check = rv_current;
  delay(CHECK_DELAY);
}
