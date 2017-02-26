//*******************************//
// Soldering Station
// https://github.com/jlegen/Solderstation
// based on work from Matthias Wagner
// https://debugginglab.wordpress.com/2014/10/30/soldering-station/
// Modified by jleg99@gmail.com:
// - code cleanup
// - new shutdown mode
// - proper LED and Fan support
// - measure Vcc and Vin
//*******************************//

// tested on Arduino Pro mini with standard boot loader
// external libraries used are all available by using the IDE library mananger:
// TimerOne, ClickEncoder, FastLED, Adafruit_GFX, Adafruit_ST7735

// more Ideas:
// - replace pot by rotary encoder 
// - enter menu with long click: show 3-4 temp. presets, shutdown temp, shutdown time
// - select presets with short lick (with little timeout/delay for skipping presets)
// - replace logo(s) (2-3 blue boxes on black with white text?) PCB-Background? + simpler tip in main screen?

// D12: Standby=0V, 5V = normal  

//#define __PROG_TYPES_COMPAT__ 
#include <SPI.h>
#include <EEPROM.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

#include "icons.h"
#include "stationLOGO.h"
#define ST7735_GREY 0x632C
#define BARCOLOR ST7735_MAGENTA
#define BARHEIGHT 8
#define BACKGROUND ST7735_BLACK

// experimenting with other display sizes
#define PIXELS_X 128
#define PIXELS_Y 160
#define SCALE_X (PIXELS_X/128)
#define SCALE_Y (PIXELS_Y/160)

#define VERSION "2.0"

// comment in/out to (de)activate
//#define DEBUG
#define INTRO
#define HAVE_LED  // control a WS2812 RGB LED 
#define HAVE_FAN  // fan connected to FAN_PIN
#define HAVE_VOLT // also measure Vin - needs voltage divider: Vin <-22k-> VCCpin <-5.6k-> GND (max. 25V with this config)

//
// Pin assignments
//

//#define sclk  13 // Don't change
//#define mosi  11 // Don't change

#ifdef HAVE_LED
 #include <FastLED.h>
 #define NUM_LEDS 1
 #define LED_PIN 6    // Pin of WS2812
 #define BRIGHTNESS 200
 CRGB leds[NUM_LEDS];
#endif

#ifdef HAVE_FAN  
 #define FAN_PIN A0   // Pin of fume fan
#endif

#ifdef HAVE_VOLT
 #define VCCpin A6  //Voltage divider pin for Vin
// volatile double v_in;
 double v_in = 0.0;
#endif

#define tft_bl   7  // TFT Backlight
#define tft_dc   9	 
#define tft_cs  10   
#define tft_rst 12	// use 0 when unused (or wired to Arduino RST)

#define STANDBYin A4  // connect to tip stand
#define POTI   	A5
#define TEMPin 	A7
#define PWMpin 	3

#define CNTRL_GAIN      10 // default is 10
#define DELAY_MAIN_LOOP 0 // default is 10
#define DELAY_MEASURE 	50 // default is 50
#define ADC_TO_TEMP_GAIN 	0.53 // mit original Weller Station verglichen
#define ADC_TO_TEMP_OFFSET 13.0 
//#define STANDBY_TEMP	  150  // decrease to this temp. when tip is in stand
//#define SHUTDOWN_TIME 300000 // set temp to 0 after these ms in stand
#define READ_INTVAL     1000 // intervall to measure voltages
#define EEPROM_ADDR 10  // start address for avr eeprom 

#define OVER_SHOT 			2
#define MAX_PWM_LOW			180
#define MAX_PWM_HI			210		//254
#define MAX_TEMP				400		//400Grad C

#define r_button 17                //rotary encoder pushbutton, PB1
#define r_pha 16                   //rotary encoder phase A, PB2
#define r_phb 15                   //rotary encoder phase B, PB3

#define PWM_DIV 1024						//default: 64   31250/64 = 2ms

Adafruit_ST7735 tft = Adafruit_ST7735(tft_cs, tft_dc, tft_rst);
ClickEncoder Encoder(r_pha, r_phb, r_button, 1); // 2,3,4 = clicks per step (notches)


// state machine: modes
#define OP_NORMAL 0
#define OP_STANDBY 1
#define OP_SHUTDOWN 2

int op_state = -1;

int pwm = 0;             // pwm Out Val 0.. 255
int soll_temp = 260;
int LastPercent = 0;
unsigned long StbyMillis = 0;
unsigned long CurMillis;
unsigned long PrevMillis = 0;
bool is_error = false;
double vcc = 0.0;
int16_t encMovement;
int16_t encAbsolute;
//int16_t encLastAbsolute = -1;
namespace State {  
  typedef enum SystemMode_e {
    None      = 0,
    Default   = (1<<0),
    Settings  = (1<<1),
    Edit      = (1<<2)
  } SystemMode;
};

uint8_t systemState = State::Default;
uint8_t previousSystemState = State::None;
//bool updateMenu = false;

// define parameter's index
#define STARTTEMP 0
#define STANDBYTEMP 5
#define SHUTDOWNTIME 4
uint16_t params[6] = {260, 250, 300, 350, 5, 150};
#define MAX_PRESETS 3 // how many temp. presets to store
uint8_t act_preset = 0;

void timerIsr()
{
  Encoder.service();
}

void access_eeprom(bool do_write = false) {
int i;
uint16_t eeadr = EEPROM_ADDR;
int len = sizeof(params) / sizeof(int16_t);
 for (i=0; i<len; i++) {
   if (do_write) {
    EEPROM.put(eeadr, params[i]);
   } else {
    EEPROM.get(eeadr, params[i]);
   }
   eeadr += sizeof(int16_t);
 }
}


void setup(void) {

#ifdef HAVE_LED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
	FastLED.setBrightness(BRIGHTNESS);
  fill_solid( leds, NUM_LEDS, CRGB::White );
#endif

#ifdef HAVE_FAN
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
#endif 

  pinMode(tft_rst,OUTPUT);
  digitalWrite(tft_rst, HIGH);
  
	pinMode(tft_bl, OUTPUT);
	digitalWrite(tft_bl, LOW);
	
	pinMode(STANDBYin, INPUT_PULLUP);
	
	pinMode(PWMpin, OUTPUT);
	digitalWrite(PWMpin, LOW);
	setPwmFrequency(PWMpin, PWM_DIV);
	digitalWrite(PWMpin, LOW);
	
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // 8MHz
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  Encoder.setAccelerationEnabled(true);

  tft.initR(INITR_BLACKTAB);
	tft.setRotation(0);	// 0 - Portrait, 1 - Landscape
	tft.setTextWrap(false);
	
#ifdef INTRO
  tft.fillScreen(ST7735_GREEN);
  tft.drawBitmap(0,0,splash,128,149,BACKGROUND);
  tft.fillRect(0,149,128,11,BACKGROUND); // image lacks some y width...
	tft.setTextSize(1);
	tft.setTextColor(ST7735_YELLOW);
	tft.setCursor(40,50);
	tft.print("AVR");
  tft.setCursor(40,60);
	tft.print("Soldering");
  tft.setCursor(40,70);
	tft.print("Station");
  tft.setCursor(40,80);
  tft.print("V");
	tft.print(VERSION);
	
  //Backlight on
  digitalWrite(tft_bl, HIGH);

  ClickEncoder::Button b = Encoder.getButton(); // check button on startup - force write defaults to eeprom
  uint16_t eetest;
  EEPROM.get(EEPROM_ADDR, eetest);
  access_eeprom(eetest < 10 || eetest > 400 || (b != ClickEncoder::Open)); // read conf from eeprom, or write defaults

#ifdef DEBUG
  Serial.begin(56700);
  Serial.print(F("SolderStation V "));
  Serial.println(VERSION);
  Serial.print(F("EEProm Standby Temp.: "));
  Serial.println(params[STANDBYTEMP]);
  Serial.print(F("EEProm Shutdown Time: "));
  Serial.println(params[SHUTDOWNTIME]);
  Serial.print(F("EEProm Prog Temp 1: "));
  Serial.println(params[1]);
  Serial.print(F("EEProm Prog Temp 2: "));
  Serial.println(params[2]);
  Serial.print(F("EEProm Prog Temp 3: "));
  Serial.println(params[3]);
  Serial.print(F("Display scale factor X: "));
  Serial.println(SCALE_X);
  Serial.print(F("Display scale factor Y: "));
  Serial.println(SCALE_Y);
 #ifdef HAVE_LED
   Serial.print(F("WS2812 LED activated Pin: "));
   Serial.println(LED_PIN);
 #endif
 #ifdef HAVE_FAN
   Serial.print(F("FAN activated Pin: "));
   Serial.println(FAN_PIN);
 #endif
#endif

	delay(2000);
#endif

  //Print station Logo
  tft.fillScreen(BACKGROUND);
  tft.drawBitmap(2,1,stationLOGO1,124,47,ST7735_GREY);
  tft.drawBitmap(3,3,stationLOGO1,124,47,ST7735_YELLOW);    
  tft.drawBitmap(3,3,stationLOGO2,124,47,tft.Color565(254,147,52)); 
  tft.drawBitmap(3,3,stationLOGO3,124,47,tft.Color565(255,78,0));
  
#ifndef INTRO
  //Backlight on
  digitalWrite(tft_bl, HIGH);
#endif
  
  tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(1,46);
  tft.print("v");
  tft.print(VERSION);

	tft.setTextColor(ST7735_WHITE);

	tft.setCursor(1,84);
	tft.print("Temp");

  tft.setCursor(1,129);
  tft.print("Set");

  tft.setCursor(1,151);
  tft.print("PWM");
  
  tft.setCursor(122,58);
  tft.print("O");
  tft.setCursor(122,104);
  tft.print("O");
  tft.setCursor(122,145);
  tft.print("%");
}

void loop() {
int soll_temp_tmp;

  CurMillis = millis();

  // handle encoder
  encMovement = Encoder.getValue();
  if (encMovement) {
    //PrevMillis = CurMillis;  // INTERVAL time for showing data
    encAbsolute += encMovement;
    switch (systemState) {
      case State::Settings:
        //engine->navigate((encMovement > 0) ? engine->getNext() : engine->getPrev());
        //updateMenu = true;
        break;

      // scroll through data to show
      case State::Default:
       if ((encMovement > 0 && soll_temp < MAX_TEMP) || (encMovement < 0 && soll_temp >  0)) soll_temp += encMovement;
       if (soll_temp < 0) soll_temp = 0;
       if (soll_temp > MAX_TEMP) soll_temp = MAX_TEMP;
       //update_screen(show_screen);
      break;
    }
  }

  // handle button
  switch (Encoder.getButton()) {
    case ClickEncoder::Clicked:
      switch (systemState) {
        // navigate menu structure
        case State::Settings:
          //engine->invoke();
          //updateMenu = true;
          break;

        // exit show data mode
        case State::Default:
          if (act_preset < MAX_PRESETS) {
            act_preset++;
          } else {
            act_preset = 0; // 0 = start/default preset
          }
          soll_temp = params[act_preset];
          tft_show_preset(act_preset, false);
          break;
        
        // save value
        case State::Edit:
          //DEBUG_PRINTLN(F("Save changes!"));
          //write_to_eeprom();
          //Encoder.setAccelerationEnabled(false);
          //engine->navigate(&miSettings1);
          previousSystemState = systemState;
          systemState = State::Settings;
          //updateMenu = true;
          break;
      }
      break;

    case ClickEncoder::DoubleClicked:
      // menu: goto one level up
      /*
      if (systemState == State::Settings) {
        //DEBUG_PRINTLN(F("Doubleclick in Settings"));
        //engine->navigate(engine->getParent());
        //updateMenu = true;
      }
      // escaped from edit mode
      if (systemState == State::Edit) {
        //DEBUG_PRINTLN(F("Doubleclick in Edit"));
        //Encoder.setAccelerationEnabled(false);
        // Reset parameter to previous value
        //getItemValuePointer(engine->currentItem, &iValue);
        //*iValue = last_value;
        //previousSystemState = systemState;
        //systemState = State::Settings;
        //engine->navigate(engine->currentItem);
        //updateMenu = true;
      }
      */
      break;

    case ClickEncoder::Held:
      params[act_preset] = soll_temp;
      tft_show_preset(act_preset, true);
      // enter menu
      /*if (systemState != State::Settings) {
        //Encoder.setAccelerationEnabled(false);
        //engine->navigate(&miSettings1);
        previousSystemState = systemState;
        systemState = State::Settings;
        //updateMenu = true;
      }
      */
      break;
  }
  
  
  int actual_temperature = getTemperature();

  if (digitalRead(STANDBYin) == true) { // tip not in stand
    if (op_state != OP_NORMAL) {
      op_state = OP_NORMAL;
      #ifdef HAVE_FAN
        digitalWrite(FAN_PIN, true);
         #ifdef DEBUG
            Serial.println(F("FAN switched on."));
         #endif
      #endif
      tft.fillRect(5, 57 , 20, 22, BACKGROUND); // erase icon
    }
  } else {   // tip in stand
    if (op_state == OP_NORMAL || op_state == -1) {
     op_state = OP_STANDBY;
     #ifdef HAVE_FAN
        digitalWrite(FAN_PIN, false);
        #ifdef DEBUG
           Serial.println(F("FAN switched off."));
        #endif
     #endif
     StbyMillis = CurMillis; //start timer
     tft.drawBitmap(5,57,sb_icon,20,20,ST7735_GREEN);
    } else if (op_state == OP_STANDBY && ((CurMillis - StbyMillis) >= (params[SHUTDOWNTIME] * 60L * 1000L))) {
     op_state = OP_SHUTDOWN;
     tft.fillRect(5, 57 , 20, 22, BACKGROUND);
     tft.drawBitmap(5,57,shut_icon,20,22,ST7735_BLUE); 
    }
  }

  switch (op_state) {
    case OP_NORMAL:
      soll_temp_tmp = soll_temp;
    break;
    case OP_STANDBY:
      soll_temp_tmp = soll_temp >= params[STANDBYTEMP] ? params[STANDBYTEMP] : soll_temp;
    break;
    case OP_SHUTDOWN:
      soll_temp_tmp = 0;
    break;
  }

#ifdef DEBUG
  Serial.print(F("Status:    "));
  Serial.println(op_state);
  Serial.print(F("Encoder:   "));
  Serial.println(encAbsolute);
  Serial.print(F("Temp Soll: "));
  Serial.println(soll_temp);
  Serial.print(F("Temp Akt.: "));
  Serial.println(actual_temperature);
  Serial.print(F("soll_tmp:  "));
  Serial.println(soll_temp_tmp );
  Serial.print(F("Volt INT:  "));
  Serial.println(CurMillis % READ_INTVAL);
  if (op_state == OP_STANDBY) {
    Serial.print(F("Standby time: "));
    Serial.println((long)(CurMillis - StbyMillis)/1000);
  }
#endif

	int diff = (soll_temp_tmp + OVER_SHOT)- actual_temperature;
	pwm = diff * CNTRL_GAIN;

#ifdef DEBUG
  Serial.print(F("diff: "));
  Serial.println(diff );
  Serial.print(F("pwm:  "));
  Serial.println(pwm );
  Serial.println(F("########################################################"));
#endif
  
	int MAX_PWM;

	//Set max heating Power 
	MAX_PWM = actual_temperature <= params[STANDBYTEMP] ? MAX_PWM_LOW : MAX_PWM_HI;
	
	//8 Bit Range
	pwm = pwm > MAX_PWM ? pwm = MAX_PWM : pwm < 0 ? pwm = 0 : pwm;
	
  // tip removed?
	if (actual_temperature > 540){
		pwm = 0;
		actual_temperature = 0;
    if (!is_error) tft_message("Tip!", true);
    is_error = true;
	} else if (is_error){
    tft_message("Tip!", false);
    is_error = false;
  }

	analogWrite(PWMpin, pwm);

  writeHEATING(soll_temp, actual_temperature, pwm);
	
#ifdef HAVE_LED
	//update LED
	FastLED.show();
#endif

  if ((CurMillis % READ_INTVAL) < 50) {
    if (read_voltage()) print_voltage();
  }

	delay(DELAY_MAIN_LOOP);		//wait for some time
}


int getTemperature()
{
	analogWrite(PWMpin, 0);		//switch off heater
	delay(DELAY_MEASURE);			//wait for some time (to get low pass filter in steady state)
	int adcValue = analogRead(TEMPin); // read the input on analog pin 
#ifdef DEBUG
	Serial.print(F("ADC Value "));
	Serial.println(adcValue);
#endif
	analogWrite(PWMpin, pwm);	//switch heater back to last value
	return round(((float) adcValue)*ADC_TO_TEMP_GAIN+ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}

void writeHEATING(int tempSOLL, int tempVAL, int pwmVAL){
	static int d_tempSOLL = 0;		//Tiefpass für Anzeige (2 f. Potizittern)
	static int tempSOLL_OLD = 	10;
	static int tempVAL_OLD	= 	10;
	static int pwmVAL_OLD	= 	10;
  static int col;

	pwmVAL = map(pwmVAL, 0, 254, 0, 100);
	
	tft.setTextSize(5);

	if (tempVAL_OLD != tempVAL){
		int tempDIV = round(float(tempSOLL - tempVAL)*8.5);
		tempDIV = tempDIV > 254 ? tempDIV = 254 : tempDIV < 0 ? tempDIV = 0 : tempDIV;

    switch (op_state) {
      case OP_NORMAL:
        col = tft.Color565(tempDIV, 255-tempDIV, 0);
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB( tempDIV, 255-tempDIV, 0) );
        #endif
      break;
      case OP_STANDBY:
        col = ST7735_CYAN;
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB::Cyan );
        #endif
      break;
      case OP_SHUTDOWN:
        col = ST7735_BLUE;
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB::Blue );
        #endif
      break;
    }
		tft_print(tempVAL_OLD, tempVAL, 30, 57, col);
		tempVAL_OLD = tempVAL;
	}
	
	if ((tempSOLL_OLD+d_tempSOLL < tempSOLL) || (tempSOLL_OLD-d_tempSOLL > tempSOLL)){
    tft_print(tempSOLL_OLD, tempSOLL, 30, 102, ST7735_WHITE);
		tempSOLL_OLD = tempSOLL;
	}

	if (pwmVAL_OLD != pwmVAL){
    drawPWMBar(pwmVAL);
    tft.setTextSize(2);
    tft_print(pwmVAL_OLD, pwmVAL, 80, 144, ST7735_WHITE);

		pwmVAL_OLD = pwmVAL;
	}
}

// display current preset
void tft_show_preset(uint8_t preset, bool new_preset) {
  tft.setTextSize(2);
  if (new_preset) {
    tft.drawRect(4,101,13,17,BACKGROUND);
    tft.drawChar(5,102,(char)preset+48, ST7735_MAGENTA, BACKGROUND, 2);
    delay(200);
    tft.drawChar(5,102,(char)preset+48,BACKGROUND, ST7735_MAGENTA, 2);
    delay(200);
    access_eeprom(true); // write changed values 
  }
  tft.drawRect(4,101,13,17,ST7735_MAGENTA);
  tft.drawChar(5,102,(char)preset+48,BACKGROUND, ST7735_MAGENTA, 2); 
}

// print formatted numbers on tft 
void tft_print(int oldval, int newval, int x, int y, uint16_t col) {
  if (oldval != newval) {
    tft.setCursor(x,y);
    tft.setTextColor(BACKGROUND);
    if ((oldval/100) != (newval/100)) {
      tft.print(oldval/100);
    } else {
      tft.print(" ");
    }
    if ((oldval/10)%10 != (newval/10)%10) {
      tft.print((oldval/10)%10);
    } else {
      tft.print(" ");
    }
    if ((oldval%10) != (newval%10)) {
      tft.print(oldval%10);
    } 
    
    /*
    //tft.print((oldval/100) != (newval/100) ? (char*)(oldval/100) : " ");
    //tft.print(((oldval/10)%10) != ((newval/10)%10) ? (char*)((oldval/10)%10) : " ");
    //tft.print((oldval%10) != (newval%10) ? (char*)(oldval%10) : " ");
    */
  }

  tft.setCursor(x,y);
  tft.setTextColor(col);
  tft.print(newval < 10 ? "  " : (newval < 100) ? " " : "");
  tft.print(newval);
}

// display error/alert icon with short message
void tft_message(char* msg, bool dowrite) {
  if (dowrite) {
    tft.drawBitmap(26, 56, alert_icon, 20, 20, tft.Color565(255,0,90)); 
    tft.setTextColor(ST7735_RED);
  } else {
    tft.fillRect(26,56,20,20,BACKGROUND);
    tft.setTextColor(BACKGROUND,BACKGROUND);
  }
  tft.setTextSize(1);
  tft.setCursor(48,55);
  tft.print(msg);
}

// draw horizonal bar for PWM value
void drawPWMBar (int nPer){
  if(nPer < LastPercent){
    // erase only diff bar
    tft.fillRect(20 + nPer , PIXELS_Y - BARHEIGHT - 2 , LastPercent - nPer, BARHEIGHT, BACKGROUND); //x,y,width,height,color
  }
  else{
    tft.fillRect(20 , PIXELS_Y - BARHEIGHT - 2, nPer, BARHEIGHT, BARCOLOR);
  }    
  LastPercent = nPer;  
}

void setPwmFrequency(int pin, int divisor) {
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
		switch(divisor) {
			case 1: 		mode = 0x01; break;
			case 8: 		mode = 0x02; break;
			case 64: 		mode = 0x03; break;
			case 256: 	mode = 0x04; break;
			case 1024: 	mode = 0x05; break;
			default: return;
		}
		
		if(pin == 5 || pin == 6) {
			TCCR0B = TCCR0B & 0b11111000 | mode;
		} else {
			TCCR1B = TCCR1B & 0b11111000 | mode;
		}
	} else if(pin == 3 || pin == 11) {
		switch(divisor) {
			case 1: 		mode = 0x01; break;
			case 8: 		mode = 0x02; break;
			case 32: 		mode = 0x03; break;
			case 64: 		mode = 0x04; break;
			case 128: 	mode = 0x05; break;
			case 256: 	mode = 0x06; break;
			case 1024: 	mode = 0x07; break;
			default: return;
		}
		
		TCCR2B = TCCR2B & 0b11111000 | mode;
	}
}

#ifdef HAVE_VOLT
double measureVoltage(void) {
unsigned int ADCValue;
  analogRead(VCCpin); // settle ADC
  ADCValue = analogRead(VCCpin);
  return ADCValue * (25 / 1023.0); // range: max. 25V
}
#endif

double readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2);                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);             // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result/1000.0;       // Vcc in volts
}

void print_voltage() {
  tft.setTextSize(1);
  tft.fillRect(50,1,18,8,BACKGROUND);
  tft.setTextColor(ST7735_MAGENTA);
  tft.setCursor(30,1);
  tft.print("Vcc ");
  tft.setCursor(50,1);
  tft.print(vcc,1);
  tft.print("V");
  #ifdef DEBUG
    Serial.print(F("*********************** Vcc: "));
    Serial.println(vcc,2);
  #endif

#ifdef HAVE_VOLT
  tft.fillRect(97,1,24,8,BACKGROUND);
  tft.setCursor(78,1);
  tft.print("Vin ");
  tft.setCursor(97,1);
  tft.print(v_in,1);
  tft.print("V");
  #ifdef DEBUG
    Serial.print(F("*********************** Vin: "));
    Serial.println(v_in,2);
  #endif
#endif
}

// read voltages 
bool read_voltage() {
  bool volt_changed;
  double last_val1 = vcc;
  vcc = readVcc();
  volt_changed = ((int)(last_val1*10) != (int)(vcc*10)); // only check 1 decimal
#ifdef HAVE_VOLT
  double last_val2 = v_in;
  v_in = measureVoltage();
  volt_changed = volt_changed || ((int)(last_val2*10) != (int)(v_in*10));
#endif
  return volt_changed;
}


