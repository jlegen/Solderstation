//*******************************//
// Soldering Station
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
// TimerOne, FastLED, Adafruit_GFX, Adafruit_ST7735

// more Ideas:
// - replace pot by rotary encoder 
// - enter menu with long click: show 3-4 temp. presets, shutdown temp, shutdown time
// - select presets with short lick (with little timeout/delay for skipping presets)
// - replace logo(s) (2-3 blue boxes on black with white text?) PCB-Background? + simpler tip in main screen?

// D12: Standby=0V, 5V = normal  

#define __PROG_TYPES_COMPAT__ 
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#define BIGFONT FreeSansBold18pt7b
#include <Fonts/FreeSansBold18pt7b.h>
#define MEDFONT FreeSans9pt7b
#include <Fonts/FreeSans9pt7b.h>

#include "icons.h"
#include "stationLOGO.h"
#define ST7735_GREY 0x632C
#define BARCOLOR ST7735_MAGENTA
#define BARHEIGHT 8
#define BACKGROUND ST7735_BLACK

// experimenting with other displays
#define PIXELS_X 128
#define PIXELS_Y 160
#define SCALE_X (PIXELS_X/128)
#define SCALE_Y (PIXELS_Y/160)

#define VERSION "1.7"

// comment in/out to (de)activate
#define INTRO
//#define DEBUG
#define HAVE_LED  // control a WS2812 RGB LED 
#define HAVE_FAN  // fan connected to FAN_PIN
#define HAVE_VOLT // measure Vin - needs voltage divider Vin<-22k->analog pin<-5.6k->GND (max. 25V with this config)

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
#define STANDBY_TEMP	  150  // decrease to this temp. when tip is in stand
#define SHUTDOWN_TIME 300000 // set temp to 0 after these ms in stand
#define READ_INTVAL     1000 // intervall to measure voltages

#define OVER_SHOT 			2
#define MAX_PWM_LOW			180
#define MAX_PWM_HI			210		//254
#define MAX_POTI				400		//400Grad C

#define PWM_DIV 1024						//default: 64   31250/64 = 2ms

Adafruit_ST7735 tft = Adafruit_ST7735(tft_cs, tft_dc, tft_rst);

// state machine: modes
#define OP_NORMAL 0
#define OP_STANDBY 1
#define OP_SHUTDOWN 2
int op_state = -1;

int pwm = 0;             // pwm Out Val 0.. 255
int soll_temp = 300;
int val_poti = 0;
int LastPercent = 0;
unsigned long StbyMillis = 0;
unsigned long CurMillis;
bool is_error = false;
double vcc = 0.0;

void setup(void) {
#ifdef DEBUG
  Serial.begin(56700);
  Serial.print(F("SolderStation V "));
  Serial.println(VERSION);
  Serial.println(F("Parameter: "));
  Serial.print(F("Standby Temp.: "));
  Serial.println(STANDBY_TEMP);
  Serial.print(F("Shutdown Time: "));
  Serial.println(SHUTDOWN_TIME);
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
  
	//tft.fillRect(0,47,128,125,BACKGROUND);

  tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(1,46);
  tft.print("v");
  tft.print(VERSION);

	//tft.setFont(&FreeSansBold9pt7b);
	tft.setTextColor(ST7735_WHITE);

	tft.setCursor(1,84);
	tft.print("Temp");

  tft.setCursor(1,129);
  tft.print("Set");

  tft.setCursor(1,151);
  tft.print("PWM");

  
  //tft.setFont();
  tft.setCursor(122,58);
  tft.print("O");
  tft.setCursor(122,104);
  tft.print("O");
  tft.setCursor(122,145);
  tft.print("%");

  //Timer1.initialize(1000000); // 1000000 = 1sec
  //Timer1.attachInterrupt(timerIsr); 

/*
  
  tft.setCursor(115,47);
  tft.print("o");
  tft.setCursor(115,92);
  tft.print("o");
*/
  //tft.setTextSize(2);
  //tft.setCursor(90,144); //115,144
	//tft.print("%");
}

void loop() {
int soll_temp_tmp;
//double vcc;
//double v_in;

  CurMillis = millis();
  int actual_temperature = getTemperature();

  val_poti = analogRead(POTI);
	soll_temp = map(val_poti, 0, 1024, 0, MAX_POTI);

  if (digitalRead(STANDBYin) == true) {
    if (op_state != OP_NORMAL) {
      op_state = OP_NORMAL;
      #ifdef HAVE_FAN
        digitalWrite(FAN_PIN, true);
         #ifdef DEBUG
            Serial.println(F("FAN on."));
         #endif
      #endif
      tft.fillRect(5, 57 , 20, 22, BACKGROUND); // erase icon
    }
  } else {
    if (op_state == OP_NORMAL || op_state == -1) {
     op_state = OP_STANDBY;
     #ifdef HAVE_FAN
        digitalWrite(FAN_PIN, false);
        #ifdef DEBUG
           Serial.println(F("FAN off."));
        #endif
     #endif
     StbyMillis = CurMillis; //start timer
     tft.drawBitmap(5,57,sb_icon,20,20,ST7735_GREEN);
    } else if (op_state == OP_STANDBY && ((CurMillis - StbyMillis) >= SHUTDOWN_TIME)) {
     op_state = OP_SHUTDOWN;
     #ifdef HAVE_FAN
        digitalWrite(FAN_PIN, false);
        #ifdef DEBUG
           Serial.println(F("FAN off."));
        #endif
     #endif
     tft.fillRect(5, 57 , 20, 22, BACKGROUND);
     tft.drawBitmap(5,57,shut_icon,20,22,ST7735_BLUE); 
    }
  }

  switch (op_state) {
    case OP_NORMAL:
      soll_temp_tmp = soll_temp;
    break;
    case OP_STANDBY:
      soll_temp_tmp = soll_temp >= STANDBY_TEMP ? STANDBY_TEMP : soll_temp;
    break;
    case OP_SHUTDOWN:
      soll_temp_tmp = 0;
    break;
  }

#ifdef DEBUG
  Serial.print(F("Status: "));
  Serial.println(op_state);
  Serial.print(F("Poti:       "));
  Serial.println(val_poti);
  Serial.print(F("soll_temp:  "));
  Serial.println(soll_temp);
  Serial.print(F("akt. Temp.: "));
  Serial.println(actual_temperature);
  Serial.print(F("soll_temp_tmp: "));
  Serial.println(soll_temp_tmp );
  Serial.print("Voltdisplay-Int.: ");
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
	MAX_PWM = actual_temperature <= STANDBY_TEMP ? MAX_PWM_LOW : MAX_PWM_HI;
	
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
  //if (volt_changed) print_voltage();

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
	static int d_tempSOLL = 2;		//Tiefpass für Anzeige (Poti zittern)
	static int tempSOLL_OLD = 	10;
	static int tempVAL_OLD	= 	10;
	static int pwmVAL_OLD	= 	10;
  static int col;

	pwmVAL = map(pwmVAL, 0, 254, 0, 100);
	
	tft.setTextSize(1);
  tft.setFont(&BIGFONT);

	if (tempVAL_OLD != tempVAL){
		int tempDIV = round(float(tempSOLL - tempVAL)*8.5);
		tempDIV = tempDIV > 254 ? tempDIV = 254 : tempDIV < 0 ? tempDIV = 0 : tempDIV;

    switch (op_state) {
      case OP_NORMAL:
        tft.setTextColor(tft.Color565(tempDIV, 255-tempDIV, 0));
        col = tft.Color565(tempDIV, 255-tempDIV, 0);
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB( tempDIV, 255-tempDIV, 0) );
        #endif
      break;
      case OP_STANDBY:
        tft.setTextColor(ST7735_CYAN);
        col = ST7735_CYAN;
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB::Cyan );
        #endif
      break;
      case OP_SHUTDOWN:
        tft.setTextColor(ST7735_BLUE);
        col = ST7735_BLUE;
        #ifdef HAVE_LED
          fill_solid( leds, NUM_LEDS, CRGB::Blue );
        #endif
      break;
    }
		tft_print(tempVAL_OLD, tempVAL, 60, 90, col);
		tempVAL_OLD = tempVAL;
	}
	
	if ((tempSOLL_OLD+d_tempSOLL < tempSOLL) || (tempSOLL_OLD-d_tempSOLL > tempSOLL)){
    tft_print(tempSOLL_OLD, tempSOLL, 60, 135, ST7735_WHITE);
		tempSOLL_OLD = tempSOLL;
	}

	if (pwmVAL_OLD != pwmVAL){
    drawPWMBar(pwmVAL);
    //tft.setCursor(79,144);
    //tft.setTextColor(ST7735_WHITE); 
/*    
    const char * blanks2 = pwmVAL < 10 ? "  " : ((pwmVAL < 100) ? " " : "");
    tft.print(blanks2);
    tft.print(pwmVAL);
    */
   
    //tft.setFont(&FreeSansBold9pt7b);
    tft.setFont(&MEDFONT);
    tft_print(pwmVAL_OLD, pwmVAL, 60, 156, ST7735_WHITE);

		pwmVAL_OLD = pwmVAL;
	}
  tft.setFont();
}

// print formatted numbers on tft 
void tft_print(int oldval, int newval, int x, int y, uint16_t col) {
  int16_t  x1, y1;
  uint16_t w, h;
  char buf[3];
  
  if (oldval != newval) {
    itoa(oldval,buf,10);
    tft.getTextBounds(buf, x, y, &x1, &y1, &w, &h);
    tft.fillRect(119-w,y1,w+3,h,BACKGROUND);
  }

  itoa(newval,buf,10);
  tft.getTextBounds(buf, x, y, &x1, &y1, &w, &h);
  tft.setCursor(119-w,y);
  tft.setTextColor(col);
  tft.print(newval);
  
#ifdef DEBUG
  Serial.print(F("w: "));
  Serial.println(w);
  Serial.print(F("x1: "));
  Serial.println(x1 );
#endif  

    //if (newval < 100) tft.print("  ");
    //if (newval <  10) tft.print("  ");
//    tft.setTextColor(col);
//    tft.print(newval);
}

// display error/alert icon with short message
void tft_message(char* msg, bool dowrite) {
  //tft.fillRect(10,60,108,40,ST7735_RED);
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
  analogRead(VCCpin);
  ADCValue = analogRead(VCCpin);
  //v_in = v_in*.9+(analogRead(VCCpin)*25/1024.0)*.1; //maximum measurable is ~24.5V
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

// ISR 
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


