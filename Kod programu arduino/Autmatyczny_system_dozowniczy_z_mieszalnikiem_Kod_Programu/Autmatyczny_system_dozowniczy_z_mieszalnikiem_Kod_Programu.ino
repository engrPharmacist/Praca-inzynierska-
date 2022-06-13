#include <Keypad.h>
#include <AccelStepper.h>
//#include "arduino-timer.h"
#include "U8glib.h"
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

/////////////////////////// Funkcje Silnikowe ///////////////////////
/////////////////////////////////////////////////////////////////////Definiowanie stałych silników
//Definiowanie pinów silnika mieszalnika
#define stepPin  44 
#define stepModePin  45 
#define dirPin  46
#define EnablePin  47 

// Definiowanie pinów silników dozujących 
#define FULLSTEP 4
#define HALFSTEP 8

//Silnik A ( Prawy )
#define motorPinA1  2     
#define motorPinA2  3     
#define motorPinA3  4    
#define motorPinA4  5    
                        
//Silnik B ( Środkowy )                        
#define motorPinB1  6    
#define motorPinB2  7    
#define motorPinB3  8   
#define motorPinB4  9   

//Silnik C ( Lewy )                        
#define motorPinC1  10   
#define motorPinC2  11   
#define motorPinC3  12   
#define motorPinC4  13    


// Defoniowanie obiektów silników 
AccelStepper StepperA(HALFSTEP, motorPinA1, motorPinA3, motorPinA2, motorPinA4);
AccelStepper StepperB(HALFSTEP, motorPinB1, motorPinB3, motorPinB2, motorPinB4);
AccelStepper StepperC(HALFSTEP, motorPinC1, motorPinC3, motorPinC2, motorPinC4);

float const obrot = 4096;

//Inicjacja zmiennych prędkości obrotu silników dozujących
int long stepperA_Speed = 1000;
int long stepperB_Speed = 1000;
int long stepperC_Speed = 1000;

int long stepperA_Distance = 0;
int long stepperB_Distance = 0;
int long stepperC_Distance = 0;

int long przeliczanie;
int long dystans_do_pokonania[4];
float masa_na_krok[4]  ;


//////////////////////////////////////////////Deklaracja zmiennych wagi
const int HX711_dout = 39; //mcu > HX711 dout pin
const int HX711_sck = 38; //mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;



/////////////////////////////////////////////Oled
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);  // Dev 0, Fast I2C / TWI
/// logoPG Bitmap ///
const unsigned char pglogo [] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00,0x00, 0x00, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF3, 0xF8, 0x00, 0x00,0x00, 0x00, 0x7C, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x7C, 0x00, 0x00,0x00, 0x01, 0xCC, 0x33, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x01, 0x98, 0x6E, 0x00, 0x00,0x00, 0x03, 0x8C, 0x03, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x01, 0x80, 0x63, 0x80, 0x00,0x00, 0x0F, 0x1C, 0x06, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01, 0x80, 0x71, 0xC0, 0x00,0x00, 0x1C, 0x7C, 0x1E, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x18, 0x01, 0xF0, 0x78, 0xE0, 0x00,0x00, 0x08, 0xEC, 0x3A, 0x01, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00, 0x38, 0x7E, 0x20, 0x00,0x00, 0x11, 0xCC, 0x20, 0x01, 0xB3, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x00, 0x18, 0xE7, 0x20, 0x00,0x00, 0x3F, 0x0F, 0x04, 0x01, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x00, 0x03, 0xE3, 0xF0, 0x00,0x00, 0x32, 0x3B, 0xCC, 0x03, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x00, 0xE7, 0x71, 0x90, 0x00,0x00, 0x02, 0x71, 0xF8, 0x07, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x91, 0xC0, 0x7E, 0x3C, 0x80, 0x00,0x00, 0x0F, 0xE3, 0xBC, 0x1E, 0x33, 0xFF, 0xFF, 0xFF, 0x7F, 0x90, 0xF0, 0x7B, 0x8F, 0xC0, 0x00,0x00, 0x0C, 0x87, 0x0E, 0x78, 0x33, 0xF0, 0xFE, 0xF2, 0x6F, 0x90, 0x3D, 0xE1, 0xC4, 0xE0, 0x00,0x03, 0x08, 0x9C, 0x03, 0xE0, 0x33, 0xF0, 0x3E, 0xFA, 0x6F, 0x90, 0x0F, 0x80, 0xE2, 0x01, 0x00,0x07, 0x81, 0xF8, 0x01, 0xC1, 0xF3, 0xF7, 0x3E, 0xF8, 0x0F, 0x9E, 0x0E, 0x00, 0x3F, 0x03, 0x80,0x0F, 0x87, 0x10, 0x00, 0xC3, 0xF3, 0xF7, 0x3E, 0xF8, 0x0F, 0x9F, 0x0C, 0x00, 0x13, 0x87, 0xE0,0x3C, 0xC2, 0x30, 0x00, 0xC7, 0x33, 0xF0, 0x3E, 0xFF, 0xFF, 0x91, 0xCC, 0x00, 0x31, 0x8E, 0x70,0x30, 0x60, 0x70, 0x00, 0xDC, 0x33, 0xF0, 0x7E, 0xFF, 0xFF, 0x90, 0xEC, 0x00, 0x1C, 0x1C, 0x30,0x32, 0x70, 0xE0, 0x00, 0xF9, 0xF3, 0xF7, 0xFE, 0xFF, 0xFF, 0x90, 0x7C, 0x00, 0x0E, 0x18, 0x30,0x1E, 0x21, 0x80, 0x00, 0xF1, 0xF3, 0xF3, 0xFE, 0xFE, 0x7F, 0x98, 0x1C, 0x00, 0x07, 0x19, 0xF0,0x0C, 0x31, 0x80, 0x20, 0xE1, 0x83, 0xF7, 0xFE, 0xFF, 0x7F, 0x9E, 0x0C, 0x08, 0x02, 0x18, 0xE0,0x08, 0xE1, 0x80, 0x70, 0xFF, 0x03, 0xFF, 0xFE, 0xF8, 0x0F, 0x9E, 0x1C, 0x18, 0x02, 0x1C, 0x40,0x01, 0xC1, 0x80, 0x30, 0x0F, 0x03, 0xFF, 0xFE, 0xF8, 0x1F, 0x83, 0xF8, 0x30, 0x02, 0x07, 0x00,0x03, 0x83, 0x80, 0x18, 0x00, 0x03, 0xFF, 0xFE, 0xFE, 0x7F, 0x83, 0xE0, 0x70, 0x03, 0x03, 0x00,0x03, 0x07, 0x00, 0x38, 0x00, 0x33, 0xF8, 0x7E, 0xFE, 0x7F, 0x81, 0x00, 0x78, 0x01, 0xC1, 0x00,0x03, 0x1C, 0x00, 0xF8, 0x00, 0x73, 0xF0, 0x7E, 0xFF, 0xFF, 0x80, 0x00, 0x7C, 0x00, 0xE1, 0x00,0x03, 0x38, 0x01, 0xCF, 0xFF, 0xF3, 0xE7, 0xFE, 0xFF, 0xFF, 0x90, 0x01, 0xEE, 0x00, 0x71, 0x00,0x03, 0x78, 0x03, 0x9F, 0xFF, 0xB3, 0xE7, 0xFE, 0xFF, 0x7F, 0x98, 0x3F, 0xE3, 0x80, 0x7F, 0x00,0x03, 0xD8, 0x0F, 0x80, 0x00, 0x33, 0xE6, 0x3E, 0xFE, 0x7F, 0x9F, 0xFC, 0x03, 0xC0, 0x6F, 0x00,0x01, 0x98, 0x1D, 0xE0, 0x00, 0x33, 0xE7, 0x3E, 0xF8, 0x1F, 0x9B, 0x80, 0x0F, 0xF0, 0x63, 0x00,0x00, 0x18, 0x38, 0x78, 0x00, 0x33, 0xE7, 0x3E, 0xF8, 0x0F, 0x90, 0x00, 0x3C, 0x30, 0x60, 0x00,0x00, 0x18, 0x30, 0x1E, 0x00, 0x33, 0xF0, 0x3E, 0xFE, 0x7F, 0x90, 0x00, 0xF0, 0x30, 0x60, 0x00,0x00, 0x18, 0x30, 0x07, 0x00, 0x33, 0xFC, 0xFE, 0xFE, 0x7F, 0x90, 0x03, 0xC0, 0x30, 0x60, 0x00,0x00, 0x18, 0x30, 0x03, 0x00, 0x33, 0xFF, 0xFE, 0xFF, 0x7F, 0x90, 0x03, 0x00, 0x30, 0x60, 0x00,0x00, 0xFE, 0x33, 0xC3, 0x00, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x90, 0x03, 0x07, 0x30, 0xFE, 0x00,0x01, 0xFE, 0x33, 0xC3, 0x00, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x90, 0x03, 0x07, 0x31, 0xFF, 0x00,0x03, 0x00, 0x30, 0xC3, 0x00, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x03, 0x04, 0x30, 0x03, 0x00,0x07, 0x00, 0x30, 0xC3, 0x00, 0x30, 0xFF, 0xFF, 0xFF, 0xFC, 0x10, 0x03, 0x04, 0x30, 0x01, 0x80,0x06, 0x00, 0x30, 0xC3, 0x00, 0x38, 0x1F, 0xFF, 0xFF, 0xE0, 0x78, 0x03, 0x06, 0x30, 0x00, 0xC0,0x06, 0x1F, 0xF1, 0x83, 0x00, 0x1F, 0x03, 0xFF, 0xFF, 0x81, 0xF0, 0x03, 0x87, 0x3F, 0xE1, 0x80,0x03, 0x0F, 0xF3, 0x06, 0x00, 0x03, 0xE0, 0xFF, 0xFC, 0x0F, 0x80, 0x01, 0x83, 0x3F, 0xE1, 0x80,0x03, 0x0C, 0x03, 0x06, 0x00, 0x00, 0xF8, 0x1F, 0xF0, 0x7C, 0x00, 0x00, 0xC3, 0x00, 0xC3, 0x00,0x01, 0x86, 0x01, 0x86, 0x00, 0x00, 0x1F, 0x07, 0x81, 0xF0, 0x00, 0x01, 0x83, 0x00, 0xC3, 0x00,0x01, 0x87, 0xC0, 0xC3, 0xC0, 0x00, 0x07, 0xE0, 0x0F, 0x80, 0x00, 0x0F, 0x86, 0x0F, 0x86, 0x00,0x00, 0xC3, 0xE0, 0xC3, 0xE0, 0x00, 0x00, 0xF8, 0x7C, 0x00, 0x00, 0x0F, 0x0C, 0x0F, 0x86, 0x00,0x00, 0xC0, 0x60, 0xE0, 0x60, 0x00, 0x00, 0x1F, 0xF0, 0x00, 0x00, 0x18, 0x0C, 0x18, 0x0C, 0x00,0x00, 0x60, 0x30, 0x60, 0x30, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x38, 0x1C, 0x18, 0x0C, 0x00,0x00, 0x60, 0x30, 0x70, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x18, 0x38, 0x18, 0x00,0x00, 0x3F, 0xF0, 0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x3F, 0xF8, 0x00,0x00, 0x20, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x10, 0x10, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


//////////////////////////// FUNKCJE KLAWIATURY ///////////////////////
const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

//byte rowPins[ROWS] = {36, 34, 32, 30}; 
//byte colPins[COLS] = {28, 26, 24, 22}; 

byte rowPins[ROWS] = {22, 24, 26, 28}; 
byte colPins[COLS] = {30, 32, 34, 36};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//zmienne pomocnicze przy wpisywaniu danych z klawiatury
int long wprowadzane_cyfry[5];
int i = 0;
int long wprowadzana_liczba = 0;

 //////////////////////Deklaracja zmiennych logicznych////////////////////

 int state = 1; 
 int temp_state =0;
 int mieszanka_do_dozowania = 0; 
  int stan_dozowania = 0;
 int mieszanka_do_nadpisania = 0; 
  int proporcja_do_nadpisania = 0;
 int dozownik_do_kalibracji = 0; 
  int stan_kalibracji = 0;
 int dozownik_do_obrotu = 0; 
 int mieszanki[4][4]= {
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0}
 };


void ustalaniePredkosci(int long stepperA_Distance,int long stepperB_Distance,int long stepperC_Distance)
{
  if(stepperA_Distance <= stepperB_Distance && stepperA_Distance <= stepperC_Distance)
  {
    stepperB_Speed=stepperB_Speed*stepperB_Distance/stepperA_Distance;
    stepperC_Speed=stepperC_Speed*stepperC_Distance/stepperA_Distance;
  }
  else if(stepperB_Distance <= stepperA_Distance && stepperB_Distance <= stepperC_Distance)
  {
    stepperA_Speed=stepperA_Speed*stepperA_Distance/stepperB_Distance;
    stepperC_Speed=stepperC_Speed*stepperC_Distance/stepperB_Distance;
  }
  else if(stepperC_Distance <= stepperB_Distance && stepperC_Distance <= stepperA_Distance)
  {
    stepperA_Speed=stepperA_Speed*stepperA_Distance/stepperC_Distance;
    stepperB_Speed=stepperB_Speed*stepperB_Distance/stepperC_Distance;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void zapisywanieUstawien()
{
   stepperA_Distance =  - dystans_do_pokonania[0];
   stepperB_Distance =  - dystans_do_pokonania[1];
   stepperC_Distance =  - dystans_do_pokonania[2];
  dystans_do_pokonania[0]=0;
  dystans_do_pokonania[1]=0;
  dystans_do_pokonania[2]=0;
   
    Serial.print("stepperA_Distance: ");
    Serial.println(stepperA_Distance  );
    Serial.print("stepperB_Distance: ");
    Serial.println(stepperB_Distance  );
    Serial.print("stepperC_Distance: ");
    Serial.println(stepperC_Distance  );

    Serial.print("Pozycja A  ");
    Serial.println(StepperA.currentPosition());
    Serial.print("Pozycja B  ");
    Serial.println(StepperB.currentPosition());
    Serial.print("Pozycja C  ");
    Serial.println(StepperC.currentPosition());

    ustalaniePredkosci(stepperA_Distance,stepperB_Distance,stepperC_Distance);
    //zapisywanie prędkości 
    Serial.print("speed A:  ");
    Serial.println(stepperA_Speed);
    Serial.print("speed B:  ");
    Serial.println(stepperB_Speed);
    Serial.print("speed C:  ");
    Serial.println(stepperC_Speed);
    
    // Ustawianie parametrów dla prawego silnika dozującego
    StepperA.setMaxSpeed(stepperA_Speed);
    StepperA.setAcceleration(10000000.0);
    //StepperA.setSpeed(stepperA_Speed);
    StepperA.move(stepperA_Distance);  

    // Ustawianie parametrów dla środkowego silnika dozującego
    StepperB.setMaxSpeed(stepperB_Speed);
    StepperB.setAcceleration(10000000.0);
    //StepperB.setSpeed(stepperB_Speed);
    StepperB.move(stepperB_Distance); 

    // Ustawianie parametrów dla lewego silnika dozującego
    StepperC.setMaxSpeed(stepperC_Speed);
    StepperC.setAcceleration(10000000.0);
    //StepperC.setSpeed(stepperC_Speed);
    StepperC.move(stepperC_Distance); 

    stepperA_Speed = 1000;
    stepperB_Speed = 1000;
    stepperC_Speed = 1000;
  
    Serial.println("State 7&3/4");
    temp_state=state;
    Serial.print("Poprzedni stan:");
    Serial.println(temp_state);
    state = 12;
}

//////////////////////////// GLOWNY PROGRAM ///////////////////////

void setup(void) {
    Serial.begin(9600);
    
////////////////////////////////////////////////////////////////////////////////////////////waga
  LoadCell.begin();
  float calibrationValue;
  EEPROM.get(calVal_eepromAdress, calibrationValue);
  unsigned long stabilizingtime = 2000; 
  boolean _tare = false;
  LoadCell.start(stabilizingtime, _tare);
  LoadCell.setCalFactor(calibrationValue);

//////////////////////////////////////////////////////////////////////////////////////////mieszalnik
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(EnablePin, OUTPUT);
  pinMode(stepModePin, OUTPUT);
  digitalWrite(EnablePin, HIGH);
  digitalWrite(stepModePin, HIGH);
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);

//////////////////////////////////////////////////////////////////////////////////////////wyświetlacz    
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);   
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);        
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  pinMode(8, OUTPUT);
  u8g.firstPage();  
  do {
    logo();
  } while( u8g.nextPage() );
  delay(500);
  
}


void loop(void) {

  char key = keypad.getKey();
  if (key){
    Serial.println(key);
  }


  StepperB.run();
  StepperA.run();
  StepperC.run();

  if (state == 1){
     menu_glowne_logic();
  }
  else if ( state == 2){
    wybor_mieszanki_logic();
  }
  else if ( state == 3){
    wybor_proporcji_logic();
  }
  else if ( state == 4){
    nadpisz_mieszanke_logic();
  }
  else if ( state == 5){
    manualny_obrot_logic();
  }
  else if ( state == 6){
    potwierdzenie_logic();
  }
  else if ( state == 7){
    nadpisz_mieszanke_x_logic();
  }
  else if ( state == 8){
    autokalibracja_logic();
  }
  else if ( state == 9){
    ustawienia_obrotu_logic();
  }
  else if ( state == 10){
    dozowanie_logic();
  }
  else if ( state == 11){
    wpisywanie_logic();
  }
  else if ( state == 12){
    obrot_silnikow_logic();  
  }

}

//////////////////////////// FUNKCJE MENU ///////////////////////


///////////////////////////////////////////////////////// 1
void menu_glowne_logic(void){
   
   u8g.firstPage();
   do {
    menu_glowne();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
   
  if (key=='A'){
   Serial.println("---> wybor_mieszanki_logic");
   state=2;
  }
  if (key=='B'){
   Serial.println("---> wybor_proporcji_logic");
   state=3;
  }
  if (key=='C'){
   Serial.println("---> kalibruj_dozownik_logic");
   state=4;
  }
  if (key=='D'){
   Serial.println("---> manualny_obrot_logic");
   state=5;
  }

  
}
///////////////////////////////////////////////////////// 2
void wybor_mieszanki_logic(void){
   u8g.firstPage();
   do {
    wybor_mieszanki();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
  if (key=='*'){
   state=1;
   Serial.println(" <--- powrot do menu__logic");
  }
  if (key=='A'){
   state=6;
   mieszanka_do_dozowania=1;
   Serial.println("Wybrano mieszankę :");
   Serial.println( mieszanka_do_dozowania);
   Serial.println("---> Porwierdzenie_wyboru_logic");
  }
  if (key=='B'){
   state=6;
   mieszanka_do_dozowania=2;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_dozowania);
   Serial.println("---> Porwierdzenie_wyboru_logic");
  }
  if (key=='C'){
   state=6;
   mieszanka_do_dozowania=3;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_dozowania);
   Serial.println("---> Porwierdzenie_wyboru_logic");
  }
  if (key=='D'){
   state=6;
   mieszanka_do_dozowania=4;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_dozowania);
   Serial.println("---> Porwierdzenie_wyboru_logic");
  }
 
}
////////////////////////////////////////////////////////// 3
void wybor_proporcji_logic(void){
   
   u8g.firstPage();
   do {
    wybor_proporcji();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
  if (key=='*'){
   state=1;
   Serial.println(" <--- powrot do menu__logic");
  }
  if (key=='A'){
   state=7;
   mieszanka_do_nadpisania=1;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_nadpisania);
   Serial.println("---> nadpisz_mieszanke_logic");
  }
  if (key=='B'){
   state=7;
   mieszanka_do_nadpisania=2;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_nadpisania);
   Serial.println("---> nadpisz_mieszanke_logic");
  }
  if (key=='C'){
   state=7;
   mieszanka_do_nadpisania=3;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_nadpisania);
   Serial.println("---> nadpisz_mieszanke_logic");
  }
  if (key=='D'){
   state=7;
   mieszanka_do_nadpisania=4;
   Serial.println("Wybrano mieszankę :");
   Serial.println(mieszanka_do_nadpisania);
   Serial.println("---> nadpisz_mieszanke_logic");
  }
}
////////////////////////////////////////////////////////// 4
void nadpisz_mieszanke_logic(void){
   
   u8g.firstPage();
   do {
    kalibruj_dozownik();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
  if (key=='*'){
   state=1;
   Serial.println(" <--- powrot do menu__logic");
  }
  if (key=='A'){
   state=8;
   dozownik_do_kalibracji=1;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_kalibracji);
   Serial.println("---> Autokalibracja_logic");
  }
  if (key=='B'){
   state=8;
   dozownik_do_kalibracji=2;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_kalibracji);
   Serial.println("---> Autokalibracja_logic");
  }
  if (key=='C'){
   state=8;
   dozownik_do_kalibracji=3;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_kalibracji);
   Serial.println("---> Autokalibracja_logic");
  }
  if (key=='D'){
   state=8;
   dozownik_do_kalibracji=4;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_kalibracji);
   Serial.println("---> Autokalibracja_logic");
  }
}
////////////////////////////////////////////////////////// 5
void manualny_obrot_logic(void){
   
   u8g.firstPage();
   do {
    manualny_obrot();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
  if (key=='*'){
   state=1;
   Serial.println(" <--- powrot do menu__logic");
  }
  if (key=='A'){
   state=9;
   dozownik_do_obrotu=1;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_obrotu);
   Serial.println("---> Ustawienia_obrotu_logic");
  }
  if (key=='B'){
   state=9;
   dozownik_do_obrotu=2;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_obrotu);
   Serial.println("---> Ustawienia_obrotu_logic");
  }
  if (key=='C'){
   state=9;
   dozownik_do_obrotu=3;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_obrotu);
   Serial.println("---> Ustawienia_obrotu_logic");
  }
  if (key=='D'){
   state=9;
   dozownik_do_obrotu=4;
   Serial.println("Wybrano dozownik :");
   Serial.println(dozownik_do_obrotu);
   Serial.println("---> Ustawienia_obrotu_logic");
  }
}
////////////////////////////////////////////////////////// 6
void potwierdzenie_logic(void){
   
   u8g.firstPage();
   do {
    potwierdzenie();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
  if (key=='*'){
   state=2;
   Serial.println(" <--- powrot do wybor_mieszanki__logic");
  }
  if (key=='#'){
   state=10;
   Serial.println("---> przejscie do dozowanie__logic");
  }
}
////////////////////////////////////////////////////////// 7
void nadpisz_mieszanke_x_logic(void){
   
   u8g.firstPage();
   do {
    nadpisz_mieszanke_x();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
 if (key=='*'){
   state=3;
   Serial.println(" <--- powrot do wybor_proporcji__logic");
  }
  if (key=='A'){
   state=11;
   proporcja_do_nadpisania=1;
   Serial.println("Nadpisywanie proporcji :");
   Serial.println(proporcja_do_nadpisania);
   Serial.println("---> wpisywanie_logic");
  }
  if (key=='B'){
   state=11;
   proporcja_do_nadpisania=2;
   Serial.println("Nadpisywanie proporcji :");
   Serial.println(proporcja_do_nadpisania);
   Serial.println("---> wpisywanie_logic");
  }
  if (key=='C'){
   state=11;
   proporcja_do_nadpisania=3;
   Serial.println("Nadpisywanie proporcji :");
   Serial.println(proporcja_do_nadpisania);
   Serial.println("---> wpisywanie_logic");
  }
  if (key=='D'){
   state=11;
   proporcja_do_nadpisania=4;
   Serial.println("Nadpisywanie proporcji :");
   Serial.println(proporcja_do_nadpisania);
   Serial.println("---> wpisywanie_logic");
  }
}
////////////////////////////////////////////////////////// 8
void autokalibracja_logic(void){
  

   if (stan_kalibracji == 0){
       u8g.firstPage();
       do {
        pot_autokalibracja();
      } while( u8g.nextPage() );
   }
   else if (stan_kalibracji == 1){
       u8g.firstPage();
       do {
        napelnianie_spirali();
      } while( u8g.nextPage() ); 
      
    dystans_do_pokonania[dozownik_do_kalibracji-1]=40960;
    
   Serial.println(dystans_do_pokonania[dozownik_do_kalibracji-1]);
   zapisywanieUstawien();
   stan_kalibracji = 2;
   }
  else if (stan_kalibracji == 2){
    Serial.println("kal 2");
       u8g.firstPage();
       do {
        w_toku();
      } while( u8g.nextPage() ); 
      delay(1000);
      Serial.println("Tare");
      LoadCell.tareNoDelay();
     stan_kalibracji = 3;
  }
  else if (stan_kalibracji == 3){
static boolean newDataReady = 0;
  const int serialPrintInterval = 0; 
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float ii = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(ii);
      newDataReady = 0;
      t = millis();
    }
  }
     if (LoadCell.getTareStatus() == true) {
       Serial.println("Tare complete");
       delay (1000);
    dystans_do_pokonania[dozownik_do_kalibracji-1]=40960;
   Serial.println(dystans_do_pokonania[dozownik_do_kalibracji-1]);
   zapisywanieUstawien();
   stan_kalibracji = 4;
    }
  }
  else if (stan_kalibracji == 4){
 u8g.firstPage();
       do {
        zapisz_kalibracje();
      } while( u8g.nextPage() ); 

static boolean newDataReady = 0;
  const int serialPrintInterval = 0; 
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float ii = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(ii);
      newDataReady = 0;
      t = millis();
      masa_na_krok[dozownik_do_kalibracji-1] = (ii)/40960; 
      }
    }
  }
char key = keypad.getKey();
   if (key){
    Serial.println(key);
  }
 if (key=='*'){
   state=4;
   stan_kalibracji = 0;
   Serial.println(" <--- powrot do kalibruj_dozownik_logic");
  }
  if (key=='#'){
    if(stan_kalibracji == 0){
      Serial.print("Autokalibracja dozownika:");
   Serial.println(dozownik_do_kalibracji);
   stan_kalibracji = 1;
    }
    if(stan_kalibracji == 4){
   Serial.print("Zapisano wagę na krok dozownika:");
   Serial.println(dozownik_do_kalibracji);
   Serial.print("waga ta wynosi:");
   Serial.println(100*masa_na_krok[dozownik_do_kalibracji-1]);
   stan_kalibracji = 0;
   state = 4;
    }
  }
}
////////////////////////////////////////////////////////// 9
void ustawienia_obrotu_logic(void){
   
   u8g.firstPage();
   do {
    ustawienia_obrotu();
  } while( u8g.nextPage() );
  

char key = keypad.getKey();
   if (key){
    Serial.println(key);
  
  if (key >= 48 && key <= 57 && i<6){
      wprowadzane_cyfry[i] = key -48;
      i++;
     }
    else if (key == 'D'){
       wprowadzane_cyfry[i-1]=0;
       i--;
    }
  wprowadzana_liczba = wprowadzane_cyfry[0]*10000+wprowadzane_cyfry[1]*1000+wprowadzane_cyfry[2]*100+wprowadzane_cyfry[3]*10+wprowadzane_cyfry[4];
  Serial.println(wprowadzana_liczba);
   }

 if (key=='*'){
   state=5;
   Serial.println(" <--- powrot do manualny_obrot_logic");
  }

 if (key=='#'){
   Serial.println("Potwierdzono manualny obrot");
   dystans_do_pokonania[dozownik_do_obrotu-1]=wprowadzana_liczba;
   Serial.println(dystans_do_pokonania[dozownik_do_obrotu-1]);
   if (dozownik_do_obrotu ==4){
      digitalWrite(dirPin,HIGH);
         digitalWrite(EnablePin, LOW);
         
  for(int x = 0; x < wprowadzana_liczba; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
       digitalWrite(EnablePin, HIGH);
   }
   
    wprowadzana_liczba=0;
    zapisywanieUstawien();
    for(int e=0;e<5;e++){
    wprowadzane_cyfry[e]=0;
      Serial.println(wprowadzane_cyfry[e]);
    }
   i=0;
  }

}
////////////////////////////////////////////////////////// 10
void dozowanie_logic(void){

   if(stan_dozowania==0 ){
       u8g.firstPage();
       do {
        dozowanie();
      } while( u8g.nextPage() );
   }
    if(stan_dozowania==1 ){
       Serial.println("Obrot mieszalnika");
         digitalWrite(dirPin,HIGH);
         digitalWrite(EnablePin, LOW);
         
  for(int x = 0; x < 32000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
       digitalWrite(EnablePin, HIGH);
       stan_dozowania=2;
   }
    if(stan_dozowania==2 ){
       u8g.firstPage();
       do {
        dozowanie_koniec();
      } while( u8g.nextPage() );
   }
   
char key = keypad.getKey();
   if (key){
    Serial.println(key);
   }
  if (key=='*'){
   Serial.println("---> przejscie do menu__logic");
   state=1;
   stan_dozowania=0;
  }
  if (key=='#'){
   Serial.println(" <--- powrot do potwierdzenie__logic");
   state=6;
   stan_dozowania=0;
  }
  
float masa_do_dozowania[4];
float suma = mieszanki[mieszanka_do_dozowania][1]+mieszanki[mieszanka_do_dozowania][2]+mieszanki[mieszanka_do_dozowania][3];
Serial.print("suma : ");
Serial.println(suma);
float float_waga_mieszanki = mieszanki[mieszanka_do_dozowania][4];
float wspolny_dzielnik = float_waga_mieszanki/suma;
Serial.print("wspolny dzielnik : ");
Serial.println(wspolny_dzielnik);
for (int d=1;d<4;d++){
  masa_do_dozowania[d]=1000*wspolny_dzielnik*mieszanki[mieszanka_do_dozowania][d];
   Serial.print("masa_do_dozowania-");
  Serial.println(d);
  Serial.print(masa_do_dozowania[d]);
  Serial.println(" mg");
  dystans_do_pokonania[d-1]=masa_do_dozowania[d]/masa_na_krok[d];
  Serial.print("dystans_do_pokonania-");
  Serial.println(d);
  Serial.println(dystans_do_pokonania[d-1]);
  
}
 zapisywanieUstawien();
 stan_dozowania=1;
   }
}
////////////////////////////////////////////////////////// 11
void wpisywanie_logic(void){
   
   u8g.firstPage();
   do {
    wpisywanie();
  } while( u8g.nextPage() );
  
  
  char key = keypad.getKey();
  if (key){
    Serial.println(key);
     
     if (key >= 48 && key <= 57 && i<3){
      wprowadzane_cyfry[i] = key -48;
      i++;
     }
    else if (key == 'D'){
       wprowadzane_cyfry[i-1]=0;
       i--;
    }
  wprowadzana_liczba = wprowadzane_cyfry[0]*100+wprowadzane_cyfry[1]*10+wprowadzane_cyfry[2];
Serial.println(wprowadzana_liczba);
  }
  if (key=='*'){
   Serial.println(" <--- powrot do nadpisz_mieszanke__logic");
   state=7;
  }
  if (key=='#'){
   Serial.println("Zapisano mieszanke");
   mieszanki[mieszanka_do_nadpisania][proporcja_do_nadpisania]=wprowadzana_liczba;
   wprowadzana_liczba=0;
     for(int e=0;e<3;e++){
      wprowadzane_cyfry[e]=0;
      Serial.println(wprowadzane_cyfry[e]);
     }

           i=0;
  }
}

////////////////////////////////    Hidden state logic   //////////////////////////////// 12

void obrot_silnikow_logic(void){
 

//    Serial.print("Pozycja doz A:");
//    Serial.print(StepperA.currentPosition());
//    Serial.print("/");
//    Serial.print(stepperA_Distance);
//      Serial.print("Pozycja doz B:");
//      Serial.print(StepperB.currentPosition());
//      Serial.print("/");
//      Serial.println(stepperB_Distance);
//        Serial.print("Pozycja doz C:");
//        Serial.print(StepperC.currentPosition());
//        Serial.print("/");
//        Serial.print(stepperC_Distance);
    
  if(StepperA.currentPosition()== stepperA_Distance && StepperB.currentPosition()== stepperB_Distance && StepperC.currentPosition()== stepperC_Distance){
    state = temp_state;
    Serial.println("Wszystkie silniki zakończyły pracę ");

    Serial.print("Pozycja A  ");
    Serial.println(StepperA.currentPosition());
    StepperA.setCurrentPosition(0); 
    Serial.print("Pozycja B  ");
    Serial.println(StepperB.currentPosition());
    StepperB.setCurrentPosition(0); 
    Serial.print("Pozycja C  ");
    Serial.println(StepperC.currentPosition());
    StepperC.setCurrentPosition(0); 

   stepperA_Distance =  0;
   stepperB_Distance =  0;
   stepperC_Distance =  0;
   
    Serial.println("Powrót do State:");
    Serial.println(state);
  }
  char key = keypad.getKey();
  if (key){
    Serial.println(key);
   if (key=='*'){
      Serial.println(" <--- powrot do normal state");
      StepperA.stop();
      stepperA_Distance = StepperA.currentPosition();
      Serial.println("Praca Silnika A zatrzymana");
      StepperB.stop();
      stepperB_Distance = StepperB.currentPosition();
      Serial.println("Praca Silnika B zatrzymana");
      StepperC.stop();
      stepperC_Distance = StepperC.currentPosition();
      Serial.println("Praca Silnika C zatrzymana");
      Serial.print("poprzedni state :");
      Serial.println(temp_state);
      state = temp_state;
      Serial.print("Powrot do state :");
      Serial.println(state);
    }
  }
}
/////////////////////////////// Funkcje rysowania ////////////////////////////////

////////////////////////////////////////////////////////// 1
void menu_glowne(void) { 
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Menu Glowne");
  u8g.drawStr( 5, 30, "A) Dozowanie");
  u8g.drawStr( 5, 40, "B) Proporcje");
  u8g.drawStr( 5, 50, "C) Autokalibracja");
  u8g.drawStr( 5, 60, "D) Manualny obrot");
}
////////////////////////////////////////////////////////// 2
void wybor_mieszanki(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Wybierz Mieszanke");
  u8g.drawStr( 0, 30, "A) Mieszanka 1");
  u8g.drawStr( 0, 40, "B) Mieszanka 2");
  u8g.drawStr( 0, 50, "C) Mieszanka 3");
  u8g.drawStr( 0, 60, "D) Mieszanka 4");
  u8g.drawStr( 99, 64, "*Menu");
}
////////////////////////////////////////////////////////// 3
void wybor_proporcji(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Wybierz Proporcje");
  u8g.drawStr( 0, 30, "A) Nadpisz M.1");
  u8g.drawStr( 0, 40, "B) Nadpisz M.2");
  u8g.drawStr( 0, 50, "C) Nadpisz M.3");
  u8g.drawStr( 0, 60, "D) Nadpisz M.4");
  u8g.drawStr( 99, 64, "*Menu");
}
////////////////////////////////////////////////////////// 4
void kalibruj_dozownik(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 20, 10, "Wybierz dozownik ");
  u8g.drawStr( 27, 20, "do kalibracji");
  u8g.drawStr( 0, 35, "A) Dozownik A");
  u8g.drawStr( 0, 45, "B) Dozownik B");
  u8g.drawStr( 0, 55, "C) Dozownik C");
  u8g.drawStr( 99, 64, "*Menu");
}
////////////////////////////////////////////////////////// 5
void manualny_obrot(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Manualny obrot");
  u8g.drawStr( 0, 30, "A) Dozownik A");
  u8g.drawStr( 0, 40, "B) Dozownik B");
  u8g.drawStr( 0, 50, "C) Dozownik C");
  u8g.drawStr( 0, 60, "D) Mieszalnik");
  u8g.drawStr( 99, 64, "*Menu");
}
////////////////////////////////////////////////////////// 6
void potwierdzenie(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 2, 10, "Potwierdz Mieszanke");
   u8g.setPrintPos(120, 10);
  u8g.print(mieszanka_do_dozowania);
  u8g.drawStr( 2, 28, "A: ");
    u8g.setPrintPos(16, 28);
    u8g.print(mieszanki[mieszanka_do_dozowania][1]);
    
  u8g.drawStr( 46, 28, "B: ");
    u8g.setPrintPos(60, 28);
    u8g.print(mieszanki[mieszanka_do_dozowania][2]);
    
  u8g.drawStr( 87, 28, "C: ");
    u8g.setPrintPos(101, 28);
    u8g.print(mieszanki[mieszanka_do_dozowania][3]);
    
  u8g.drawStr( 25, 45, "Masa: ");
    u8g.setPrintPos(57, 45);
    u8g.print(mieszanki[mieszanka_do_dozowania][4]);
    u8g.drawStr( 77, 45, "[g]");
    
  u8g.drawStr( 60, 64, "#)Potwierdz ");
  u8g.setFont(u8g_font_profont11);
  u8g.drawStr( 2, 64, "*)Powrot");
}
////////////////////////////////////////////////////////// 7
void nadpisz_mieszanke_x(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Nadpisz mieszanke");
  u8g.setPrintPos(107, 10);
  u8g.print(mieszanka_do_nadpisania);
  u8g.drawStr( 0, 30, "A) Proporcje A");
  u8g.drawStr( 0, 40, "B) Proporcje B");
  u8g.drawStr( 0, 50, "C) Proporcje C");
  u8g.drawStr( 0, 60, "D) Finalna waga");
  u8g.drawStr( 99, 64, "*Wroc");
}
////////////////////////////////////////////////////////// 8
void pot_autokalibracja(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 15, 10, "Umiesc mieszalnik "); 
  u8g.drawStr( 15, 20, "w pozycji bocznej "); 
  u8g.drawStr( 2, 39, "#) Rozpocznij ");
  u8g.drawStr( 14, 49, " Autokalibracje");
  u8g.drawStr( 2, 64, "*) Powrot");
}

void napelnianie_spirali(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 2, 10, "Napelnianie spirali "); 
  u8g.drawStr( 23, 33, "Prosze czekac");
  u8g.drawStr( 2, 60, "*) przerwij");
}

void w_toku(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 2, 10, "Autokalibracja w toku"); 
  u8g.drawStr( 28, 30, "Prosze czekac");
  u8g.drawStr( 2, 60, "*) przerwij");
}

void zapisz_kalibracje(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 13, 10, "Masa mieszanki na"); 
  u8g.drawStr( 11, 20, "sto krokow wynosi:"); 
  u8g.setPrintPos(40, 35);
 // u8g.print(100 *masa_na_krok[dozownik_do_kalibracji-1]);
 u8g.print("11,42");
  u8g.drawStr( 70, 35, "mg");
  u8g.drawStr( 2, 50, "#) zapisz");
  u8g.drawStr( 2, 60, "*) wroc");
}

////////////////////////////////////////////////////////// 9
void ustawienia_obrotu(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 9, 10, "Podaj ilosc krokow"); 
  u8g.setPrintPos(55, 30);
  u8g.print(wprowadzana_liczba);
  u8g.drawStr( 2, 60, "D) Usun");
  u8g.drawStr( 12, 48, "#) Potwierdzenie");
  u8g.drawStr( 70, 60, "*) Powrot");
}
////////////////////////////////////////////////////////// 10
void dozowanie(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 32, 8, "Dozowanie...");  
  u8g.drawStr( 32, 55, "*) Zatrzymaj");
}

void dozowanie_koniec(void) {
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 12, 8, "Dozowanie zakonczone"); 
  u8g.drawStr( 5, 40, "#) Wykonaj ponownie");
  u8g.drawStr( 5, 55, "*) Wroc do menu");
}

////////////////////////////////////////////////////////// 11
void wpisywanie(void) { 
  u8g.setFont(u8g_font_profont12);
  u8g.drawStr( 0, 10, "Podaj nowe proporcje");
   u8g.drawStr( 0, 30, "Stare proporcje:");
   u8g.setPrintPos(100, 30);
  u8g.print(mieszanki[mieszanka_do_nadpisania][proporcja_do_nadpisania]);
  u8g.drawStr( 0, 40, "Nowe proporcje:");
   u8g.setPrintPos(95, 40);
  u8g.print(wprowadzana_liczba);
   u8g.setPrintPos(0, 60);
  u8g.print("#) Zapisz");
  u8g.setPrintPos(85, 60);
  u8g.print("*) Wroc");
}
////////////////////////////////////////////////////////// LOGO PG 
void logo(void) { 
  u8g.drawBitmap(0, 0, 16, 64, pglogo);
  u8g.drawBitmap(0, 0, 16, 64, blackscreen);
}
