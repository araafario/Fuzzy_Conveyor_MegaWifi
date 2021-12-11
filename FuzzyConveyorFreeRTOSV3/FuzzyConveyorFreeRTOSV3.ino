#include <MemoryFree.h>
#include <EEPROM.h>
#include <Arduino_FreeRTOS.h>
#include <Fuzzy.h>
#include <LiquidCrystal_I2C.h>

#define PIN_LED 13    // вывод светодиода
String inString;
boolean enablingWeb = false;


#define BARIS_LCD 2
#define KOLOM_LCD 16
LiquidCrystal_I2C lcd(0x27,KOLOM_LCD,BARIS_LCD);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//#define PRINT
#define SIMPLEPRINT
#define WAKTUTUNGGU 300 //Dalam satuan TICK, untuk konversi ke detiknya silahkan dicoba manual

const int pinIR_1 = 4; //IR di pintu masuk
const int pinIR_2 = 3; //IR di tempat berhenti
const int pinTrig = 6;          
const int pinEcho = 5;
const int pinIN1 = 11;
const int pinIN2 = 10;
const int pinIN3 = 9;
const int pinIN4 = 8;
const int pinENA = 12;
const int pinENB = 7;

bool onRead1,onRead2;

Fuzzy *fuzzy = new Fuzzy();

//Inputan untuk US
FuzzySet *near = new FuzzySet(0, 0, 15, 45); //cm
FuzzySet *far = new FuzzySet(15, 45, 500, 500); //cm

//Inputan untuk Counting
FuzzySet *few = new FuzzySet(0, 0, 2, 4); //pcs
FuzzySet *fair = new FuzzySet(2, 4, 4, 6); //pcs
FuzzySet *many = new FuzzySet(4, 6, 10, 10); //pcs

//Output PWM MINValue = 50 , MAXValue = 150
FuzzySet *slow = new FuzzySet(50, 50, 75, 100);
FuzzySet *medium = new FuzzySet(75, 100, 100, 125);
FuzzySet *fast = new FuzzySet(100, 125, 150, 150);

int countingIR(){
  static int jumlahBotol = 0;
  int readIR_1 = digitalRead(pinIR_1);
  int readIR_2 = digitalRead(pinIR_2);
  
  if (readIR_1 == 0 && onRead1 == false){
     onRead1 = true;
  }
  
  else if (readIR_1 == 1 && onRead1 == true){
     jumlahBotol++;
     #ifdef PRINT
      Serial.println(jumlahBotol);
     #endif
     onRead1 = false;
  }

  if (readIR_2 == 0 && onRead2 == false){
     onRead2 = true;
  }
  
  else if (readIR_2 == 1 && onRead2 == true){
     jumlahBotol--;
     #ifdef PRINT
      Serial.println(jumlahBotol);
     #endif
     onRead2 = false;
  }

  if (jumlahBotol < 0) jumlahBotol = 0;

  return jumlahBotol;
}

float bacaUS(){
  digitalWrite(pinTrig, LOW);
  vTaskDelay(1/portTICK_PERIOD_MS);
  digitalWrite(pinTrig, HIGH);
  vTaskDelay(1/portTICK_PERIOD_MS);
  digitalWrite(pinTrig, LOW);
  vTaskDelay( 1 / portTICK_PERIOD_MS );

  float durasi = pulseIn(pinEcho, HIGH); // menerima suara ultrasonic
  float jarak = (durasi / 2) / 29.1;  // mengubah durasi menjadi jarak (cm)
  return jarak;
}

float calculateFuzzy(float inputDistance,float inputCounting){
  fuzzy->setInput(1, inputDistance);
  fuzzy->setInput(2, inputCounting);
  fuzzy->fuzzify();
  
  float output = fuzzy->defuzzify(1);

  #ifdef PRINT
    Serial.println("Entrance: ");
    Serial.print("\tDistance: ");
    Serial.print(inputDistance);
    Serial.print(", Counting: ");
    Serial.println(inputCounting);
    
    Serial.println("Input: ");
    Serial.print("\tDistance: Near-> ");
    Serial.print(near->getPertinence());
    Serial.print(", Far-> ");
    Serial.print(far->getPertinence());
  
    Serial.print("Counting: Few-> ");
    Serial.print(few->getPertinence());
    Serial.print(",  Fair-> ");
    Serial.print(fair->getPertinence());
    Serial.print(",  Many-> ");
    Serial.println(many->getPertinence());
  
    Serial.println("Output: ");
    Serial.print("\tSpeed: Slow-> ");
    Serial.print(slow->getPertinence());
    Serial.print(",  Normal-> ");
    Serial.print(medium->getPertinence());
    Serial.print(",  Fast-> ");
    Serial.println(fast->getPertinence());
  
    Serial.println("Result: ");
    Serial.print("\tSpeed: ");
    Serial.print(output);
    Serial.print("\n\n\n");
  #endif
  
  #ifdef SIMPLEPRINT
    Serial.print("Distance: ");
    Serial.print(inputDistance);
    Serial.print(", Counting: ");
    Serial.print(inputCounting);
    Serial.print(", Output: ");
    Serial.println(output);
  #endif
  return output;
}

void motorA(int x){
  if (x > 255) x = 255;
  else if (x < -255) x = -255;

  if (x == 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }
  else if (x < 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    x = -x;
  }
  else if (x > 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  }
  analogWrite(pinENA, x);
}

void motorB(int x){
  if (x > 255) x = 255;
  else if (x < -255) x = -255;

  if (x == 0) {
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, LOW);
  }
  else if (x < 0) {
    digitalWrite(pinIN3, HIGH);
    digitalWrite(pinIN4, LOW);
    x = -x;
  }
  else if (x > 0) {
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, HIGH);
  }
  analogWrite(pinENB, x);
}

void serialEvent3() {
  
}

void TaskFuzzy( void *pvParameters );
void TaskRead( void *pvParameters );
void TaskUS( void *pvParameters );

void setup() {
  
  Serial.begin(115200);
  Serial3.begin(9600);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  Serial.println("INIT . . .  ");
  delay(1000);
  pinMode(pinIR_1, INPUT_PULLUP);
  pinMode(pinIR_2, INPUT_PULLUP);
  pinMode(pinTrig, OUTPUT);    
  pinMode(pinEcho, INPUT);  
  pinMode(pinIN1,OUTPUT);
  pinMode(pinIN2,OUTPUT);
  pinMode(pinIN3,OUTPUT);
  pinMode(pinIN4,OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinENB, OUTPUT);
  lcd.init();lcd.init();
  lcd.backlight();
  
  // FuzzyInput
  FuzzyInput *distance = new FuzzyInput(1);
  FuzzyInput *counting = new FuzzyInput(2);

  distance->addFuzzySet(near);
  distance->addFuzzySet(far);
  fuzzy->addFuzzyInput(distance);

  counting->addFuzzySet(few);
  counting->addFuzzySet(fair);
  counting->addFuzzySet(many);
  fuzzy->addFuzzyInput(counting);
  
  // FuzzyOutput
  FuzzyOutput *motorspeed = new FuzzyOutput(1);

  motorspeed->addFuzzySet(slow);
  motorspeed->addFuzzySet(medium);
  motorspeed->addFuzzySet(fast);
  fuzzy->addFuzzyOutput(motorspeed);

  // FuzzyRule - IF DISTANCE IS NEAR OR COUNTING IS MANY THEN SPEED SLOW
  FuzzyRuleAntecedent *ifDistanceNearAndCountingMany = new FuzzyRuleAntecedent();
  ifDistanceNearAndCountingMany->joinWithOR(near, many);
  
  FuzzyRuleConsequent *thenSpeedSlow = new FuzzyRuleConsequent();
  thenSpeedSlow->addOutput(slow);

  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifDistanceNearAndCountingMany, thenSpeedSlow);
  fuzzy->addFuzzyRule(fuzzyRule1);
    
  // FuzzyRule2 - IF DISTANCE IS FAR AND COUNTING IS FAIR THEN SPEED MEDIUM
  FuzzyRuleAntecedent *ifdistanceFarAndCountingFair = new FuzzyRuleAntecedent();
  ifdistanceFarAndCountingFair->joinWithAND(far, fair);
  
  FuzzyRuleConsequent *thenSpeedMedium = new FuzzyRuleConsequent();
  thenSpeedMedium->addOutput(medium);

  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifdistanceFarAndCountingFair, thenSpeedMedium);
  fuzzy->addFuzzyRule(fuzzyRule2);

   // FuzzyRule3 - IF DISTANCE IS FAR AND COUNTING IS FEW THEN SPEED FAST
  FuzzyRuleAntecedent *ifdistanceFarAndCountingFew = new FuzzyRuleAntecedent();
  ifdistanceFarAndCountingFew->joinWithAND(far, few);
  
  FuzzyRuleConsequent *thenSpeedFast = new FuzzyRuleConsequent();
  thenSpeedFast->addOutput(fast);

  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifdistanceFarAndCountingFew, thenSpeedFast);
  fuzzy->addFuzzyRule(fuzzyRule3);
  Serial.println("Fuzzy Init");
  
  xTaskCreate(
    TaskFuzzy
    ,  "Fuzzy"   
    ,  256  // Stack size
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  xTaskCreate(
    TaskUS
    ,  "US"   
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  xTaskCreate(
    TaskRead
    ,  "Read"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

}

void loop()
{

}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

float jarak;
bool show;
int counterIR,pengisian,state;
int firstT,timer;

void TaskFuzzy(void *pvParameters)  
{
  (void) pvParameters;

  vTaskDelay(1000 / portTICK_PERIOD_MS ); //delay 1 detik
  for (;;) 
  { 
    if(enablingWeb == true){
         if(state == 0){
            float output = calculateFuzzy(jarak,counterIR);
            motorA(output);motorB(output);
            //motorA(-output);motorB(-output); //Gunakan ini jika arah terbalik
            
            if (show == true) {
              lcd.setCursor(11,0);
              lcd.print("       ");
              lcd.setCursor(0,0);
              lcd.print("Distance : ");lcd.print(jarak);
              lcd.setCursor(10,1);
              lcd.print("       ");
              lcd.setCursor(0,1);
              lcd.print("Counter : ");lcd.print(counterIR);
              show = false;
            }  
            
            if (pengisian == 0) {
              lcd.clear();show = false;
              firstT = xTaskGetTickCount();
              state = 1;
            }        
         }  
         else if (state == 1){
             motorA(0);motorB(0);
             if (show == true) {
               lcd.setCursor(0,0);
               lcd.print("Tunggu Pengisian . . . ");
               lcd.setCursor(0,1);
               lcd.print("Counting ");lcd.print(xTaskGetTickCount() - firstT);
               show = false;
             }  
             
             Serial.println(xTaskGetTickCount() - firstT);
             if (xTaskGetTickCount() - firstT >= 300){
                 lcd.clear();show = false;
                 firstT = xTaskGetTickCount();
                 state = 2;              
             }
         }  
         else if(state == 2){
          if (show == true) {
               lcd.setCursor(0,0);
               lcd.print("Pengisian Selesai, Tunggu . . .");           
               show = false;
             }  
             Serial.println("CONVEYOR MOVING");
             motorA(60);
             motorB(60);
             if (pengisian == 1) {lcd.clear();show = false;state = 0;}
         }
        
        vTaskDelay(1); 
        if (xTaskGetTickCount() - timer > 10 && show == false) {show = true;timer = xTaskGetTickCount();}  
    }
    else if (enablingWeb == false){
      motorA(0);
      motorB(0);
      vTaskDelay(1);  
    }
  }
}

void TaskRead(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    while (Serial3.available()) {
    char inChar = Serial3.read();
    Serial.write(inChar);
    inString += inChar;
    if (inChar == ']') {
      if (inString.indexOf("[ON]")>0) {
        enablingWeb = true;
        digitalWrite(PIN_LED, HIGH);
      }
      else if (inString.indexOf("[OFF]")>0) {
        enablingWeb = false;
        digitalWrite(PIN_LED, LOW);
      }
      else
      {
        Serial.println("Wrong command");
      }
      inString = "";
    }
  }
   }
}

void TaskUS(void *pvParameters)  
{
  (void) pvParameters;

  for (;;) 
  {
    pengisian = digitalRead(pinIR_2);    
    counterIR = countingIR();
    jarak = bacaUS();
    vTaskDelay(1); 
  }
}
