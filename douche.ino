#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Si7021.h>

#define DEBUG 1

#define CHECK_DELAY 30000//15000

byte rv_current = 0x60;
byte rv_check   = 0x60;
byte rv_max     = 0x00;
byte rv_delta   = 0x00;
byte rv_start   = 0x00;

byte temp = 0x00;

#define INC_THRES 2
#define LO_THRES 2

byte fanstate;

#define STATE_LO    0x01
#define STATE_MED   0x02
#define STATE_HI    0x04

#define RELAY_LO 2
#define RELAY_MED 3
#define RELAY_HI 4

#define LCD_ROWS 4
#define LCD_COLS 20
#define LCD_I2C_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_Si7021 sensor = Adafruit_Si7021();

void setup()
{
  Serial.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  fanstate |= STATE_LO;
  
  rv_current = sensor.readHumidity();
  rv_max = rv_current;
  rv_check = rv_current;
  if(!sensor.begin())
  {
    Serial.println(F("Sensor error"));
    lcd.clear();
    lcd.print(F("Error 1"));
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
  Serial.println(F("test"));
  if((rv_current > (rv_check + INC_THRES)) && (fanstate & ~STATE_HI))
  {
    rv_start = rv_check;
    fan_hi();
  }
  
  if(rv_current > rv_max)
  {
    rv_max = rv_current;
    if(rv_start > 0) rv_delta = (rv_max - rv_start);
  }
  
  if((rv_current < (rv_max - (rv_delta / 2))) && (fanstate & STATE_HI))
  {
    rv_max = rv_current;
    fan_med();
  }
  
  if((rv_current < (rv_start + LO_THRES)) && (fanstate & STATE_MED))
  {
    rv_max = 0;//rv_current;
    fan_lo();
    rv_delta = 0;
    rv_start = 0;
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
  
  lcd.print('I');

}



void loop()
{ 
  /*
  print_state();
  lcd.setCursor(0,1);
  lcd.print(F("Sampling..."));
  rv_check = sensor.readHumidity();
  delay(CHECK_DELAY);
  rv_current = sensor.readHumidity();
  lcd.setCursor(0,1);
  //lcd.clearLine(1);
  //lcd.print(F("Processing.."));
  //lcd.clear();
  test_states();
  print_state();
  lcd.setCursor(0,1);
  lcd.print(F("Done"));
  */
  rv_current = sensor.readHumidity();
  temp = sensor.readTemperature();
  

  //lcd.setCursor(0,1); //next line
  //lcd.print(F("Done"));
  
  test_states();
  
  //lcd.clear();
  print_state();
 // lcd.setCursor(0,1);
  //lcd.print(F("Sampling..."));
  rv_check = rv_current;
  delay(CHECK_DELAY);
}
