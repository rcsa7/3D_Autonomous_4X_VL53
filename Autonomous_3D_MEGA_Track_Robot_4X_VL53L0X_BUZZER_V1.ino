/*** 
 *  Autonomous Arduino Mega Track Robot
 *  Track Robot w/ Arduino Mega 2560, motor shield v3, OLED, and Sharp IR sensors
 *  https://create.arduino.cc/projecthub/MyPartsChest/autonomous-arduino-mega-track-robot-10699d
 *  1-19-2020
 *  by My Parts Chest
 *  https://mypartschest.com
 *  https://mypartschest.blogspot.com/
 *  https://github.com/pololu/vl53l0x-arduino
*/

#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define BUZZER_PIN  49   // AX-1205-2 5V

/* Model :
  VL53L0X
*/

VL53L0X frontSensor;// SENSOR DA FRENTE--PIN1
VL53L0X driverSensor;// SENSOR LADO ESQUERDO--MOTORISTA--PIN2
VL53L0X passengerSensor;// SENSOR LADO DIREITO--PASSAGEIRO--PIN3
VL53L0X rearSensor;// SENSOR RE--PIN4

//--usando este pins para alterar o endereço I2C do sensor
#define PIN1 22 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define PIN2 24 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define PIN3 26 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define PIN4 28 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define led1 35//LED D35
#define led2 37//LED D37
#define led3 39//LED D39
#define led4 41//LED D41
#define led5 43//LED D43
#define led6 45//LED D45
#define led7 7//LED D7
#define led8 8//LED D8

#define ledALARM9 47//LED D47---FUTURO ALARME BATERIAS

//global vars
bool startup = false;
bool firstLoop = false;
bool motorsOn = false;
int stopTimer = 0;
String heading = "Nao iniciado";

//declar OLED pins
const int SDA_PIN = 20;
const int SDC_PIN = 21;

//Declare pins for motor control
const int dirA = 12;// motor-1
const int dirB = 13;// motor-2
const int speedA = 3;//pin-pwm- motor-1
const int speedB = 11;//pin-pwm- motor-2
const int brakeA = 9;// motor-1
const int brakeB = 8;// motor-2
///const int highSpeed = 255;
///const int lowSpeed = 240;
const int highSpeed = 185;
const int lowSpeed = 180;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
 // Begin serial communication at a baudrate of :
Wire.begin();
Serial.begin(115200);
pinMode(BUZZER_PIN, OUTPUT); 
tone(BUZZER_PIN, 200, 200); delay(200); 
tone(BUZZER_PIN, 500, 400); delay(500); 
pinMode(led1, OUTPUT);
digitalWrite(led1, LOW); 
pinMode(led2, OUTPUT);
digitalWrite(led2, LOW);
pinMode(led3, OUTPUT);
digitalWrite(led3, LOW);
pinMode(led4, OUTPUT);
digitalWrite(led4, LOW); 
pinMode(led5, OUTPUT);
digitalWrite(led5, LOW);
pinMode(led6, OUTPUT);
digitalWrite(led6, LOW);
pinMode(led7, OUTPUT);
digitalWrite(led7, LOW);
pinMode(led8, OUTPUT);
digitalWrite(led8, LOW);
pinMode(ledALARM9, OUTPUT);//LED D47---FUTURO ALARME BATERIAS
digitalWrite(ledALARM9, LOW);
  //--------------------------------------
  //-----sensores---vl53l0x-----------------------------
pinMode(PIN1, OUTPUT);
pinMode(PIN2, OUTPUT);
pinMode(PIN3, OUTPUT);
pinMode(PIN4, OUTPUT);
digitalWrite(PIN1, LOW);
digitalWrite(PIN2, LOW);
digitalWrite(PIN3, LOW);
digitalWrite(PIN4, LOW);

//delay(500);
///Wire.begin();


//--------------------------------------
pinMode(PIN1, INPUT);
///delay(150);
///Serial.println("00");
frontSensor.init(true);

///Serial.println("01");
///delay(100);
frontSensor.setAddress((uint8_t)24); //0X16 novo endereço frontSensor
//---------------------------------------------
///Serial.println("02");

pinMode(PIN2, INPUT);
///delay(150);
driverSensor.init(true);
///Serial.println("03");
///delay(100);
driverSensor.setAddress((uint8_t)28);//0X19 novo endereço driverSensor
///Serial.println("04");
//----------------------------------------------------------
//-------------------------------------------------------------
pinMode(PIN3, INPUT);
///delay(150);
passengerSensor.init(true);

///Serial.println("----");
///delay(100);
passengerSensor.setAddress((uint8_t)32);// 0x17 novo endereço passengerSensor
//-------------------------------------------------------------
pinMode(PIN4, INPUT);
///delay(150);
rearSensor.init(true);

///Serial.println("----");
///delay(100);
rearSensor.setAddress((uint8_t)36);// 0x18  novo endereço rearDistance_4
//---------------------------------------------------------------------------
///Serial.println("addresses set");

// end configuration
frontSensor.setTimeout(500);
driverSensor.setTimeout(500);
frontSensor.startContinuous();
driverSensor.startContinuous();

passengerSensor.setTimeout(500);
rearSensor.setTimeout(500);
passengerSensor.startContinuous();
rearSensor.startContinuous();

 callI2Cscanner();

//-------------------------------------------

  // Setup Channel A & B
  pinMode(dirA, OUTPUT); //Init Motor A (rear-driver's side)
  pinMode(dirB, OUTPUT); //Init Motor B (front-passenger's side)
  pinMode(brakeA, OUTPUT); //Init Brake A
  pinMode(brakeB, OUTPUT); //Init Brake B

  // Init display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    ///Serial.println(F("SSD1306 allocation failed"));
    ///for(;;);
  }
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("Inicializando...");
  display.display(); 
  
  delay(1000);
  startup = false;
  firstLoop = true;
  motorsOn = true;
}

void callI2Cscanner()
{
  // scan i2c
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++) 
  {  
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    }             // end of good response
  }               // end of for loop
  
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");  
}

void loop() {

//----------sensor---vl53l0x
  ///int distance1 =frontSensor.readRangeContinuousMillimeters();// get distance for sensor 1
  ///int distance2 =driverSensor.readRangeContinuousMillimeters();// get distance for sensor 2
  ///int distance3 =passengerSensor.readRangeContinuousMillimeters();// get distance for sensor 3
  ///int distance4 =rearDistance_4.readRangeContinuousMillimeters();// get distance for sensor 4
  
  float distance_1= frontSensor.readRangeContinuousMillimeters();// SENSOR DA FRENTE
  float distance_2= driverSensor.readRangeContinuousMillimeters();// SENSOR LADO ESQUERDO--MOTORISTA
  float distance_3= passengerSensor.readRangeContinuousMillimeters();// SENSOR LADO DIREITO--PASSAGEIRO
  float distance_4= rearSensor.readRangeContinuousMillimeters();// SENSOR RE
  
  /*
Serial.print(frontSensor.readRangeContinuousMillimeters());
if (frontSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S1"); }
Serial.print("\t");
Serial.print(driverSensor.readRangeContinuousMillimeters());
if (driverSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S2"); }

Serial.print(passengerSensor.readRangeContinuousMillimeters());
if (passengerSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S3"); }
Serial.print("\t");

Serial.print(sensor4.readRangeContinuousMillimeters());
if (sensor4.timeoutOccurred()) { Serial.print(" TIMEOUT S4"); }
Serial.print("\t");

Serial.println();

*/


 /*
  //-----local vars  sensor sharp-----------------------------------------
  int frontDistance;
  int driverDistance;
  int passengerDistance;
  int rearDistance;
   
  // Obtenha distância dos sensores
  frontDistance = frontSensor.distance();
  driverDistance = driverSensor.distance();
  passengerDistance = passengerSensor.distance();
  rearDistance = rearSensor.distance();
//---------------------------------------------------------------------------
*/
 //-----local vars sensor vl53l0x----------------------------------------------

  uint16_t frontDistance_1; // SENSOR DA FRENTE
  uint16_t driverDistance_2;// SENSOR LADO ESQUERDO--MOTORISTA
  uint16_t passengerDistance_3;// SENSOR LADO DIREITO--PASSAGEIRO
  uint16_t rearDistance_4;// SENSOR RE
   
  // Obtenha distância dos sensores
  frontDistance_1 = frontSensor.readRangeContinuousMillimeters();
  driverDistance_2 = driverSensor.readRangeContinuousMillimeters();
  passengerDistance_3 = passengerSensor.readRangeContinuousMillimeters();
  rearDistance_4 = rearSensor.readRangeContinuousMillimeters();
  
  
//----------------------------------------------------------------------------------
  //update OLED

//updateDisplay(tensao_A2,tensao_A3,tensao_A4,tensao_A5,distance_1,distance_2,distance_3,distance_4,"Stopped!");

  if (startup)
  {      
 ////////   if (frontSensor.distance() > 30) // para sensor sharp
   if (frontSensor.readRangeContinuousMillimeters() > 600) //500sensor da frente--- para sensor vl53l0x : VALORES EM MILIMETROS
    {
      firstLoop = false;
      heading = "Forward";

      //--forward B (front motor)
      //--frente B (motor dianteiro)
      move("B", "FWD", highSpeed); 
        Serial.println ("B- FWD- highSpeed-");            
      //forward A (rear motor)
      move("A", "FWD", lowSpeed); //rear motor is running a little faster than the front motor; so slow it down
      Serial.println ("A- FWD- highSpeed-");                           // o motor traseiro está funcionando um pouco mais rápido que o motor dianteiro; então diminua a velocidade
      delay(100);
      stopTimer = 0;
    }
             // menos de 30cm; Algo está no caminho
    else    //less than 30cm; something is in the way
    {
      if (heading == "Forward" && !firstLoop) //make sure this isn't at startup and I'm not already turning
                                             // certifique-se de que isso não esteja na inicialização e eu já não esteja girando 
      {
        fullStop(100); //stop during testing to read sensors
                      // pare durante o teste para ler os sensores
      
        //---back up if too close to something
        // --- backup se estiver muito perto de algo
        
  ////////      if (frontSensor.distance() < 10) // para sensor sharp
      if (frontSensor.readRangeContinuousMillimeters() < 300) //sensor da frente-- para sensor vl53l0x : VALORES EM MILIMETROS
        {
          //--forward B (front motor)
          //-- forward B (motor dianteiro)
          move("B", "REV", highSpeed);   
         Serial.println ("B- REV- highSpeed-");             
          //--forward A (rear motor)
          //-- forward A (motor traseiro)
          move("A", "REV", lowSpeed); 
          Serial.println ("A- REV- highSpeed-");       
          delay(100);          
        }
                  
   ////////     if (passengerSensor.distance() > driverSensor.distance())// para sensor sharp--   // o que tem mais distância, esquerda ou direita? LADO MOTORISTA/PASSAGEIRO
             if (passengerSensor.readRangeContinuousMillimeters() > driverSensor.readRangeContinuousMillimeters())//sensor lado passageiro--- para sensor vl53l0x           
        {
          heading = "Turning Right";
            Serial.println ("Turning Right-");   
       ///////   int lastFront = frontSensor.distance();// para sensor sharp
          int lastFront = frontSensor.readRangeContinuousMillimeters();// //sensor da frente-- para sensor vl53l0x
          
        //////  while (driverSensor.distance() != lastFront)//LADO MOTORISTA---- para sensor sharp
                while (driverSensor.readRangeContinuousMillimeters() != lastFront)// para sensor vl53l0x
          {
         
         updateDisplay(frontDistance_1,driverDistance_2,passengerDistance_3,rearDistance_4, "Stopped!");
            turnRight(500);
         /////////   if (frontSensor.distance() > 30)// SENSOR DA FRENTE-- para sensor sharp
                     if (frontSensor.readRangeContinuousMillimeters() > 500)// para sensor vl53l0x : VALORES EM MILIMETROS
            {
              break;      
            }         
          }
        }
        else
        {
          heading = "Turning Left";
           Serial.println ("Turning Left-");
       /////////   int lastFront = frontSensor.distance();//SENSOR DA FRENTE--- para sensor sharp
                   int lastFront = frontSensor.readRangeContinuousMillimeters();// para sensor vl53l0x
      ///////////    while (passengerSensor.distance() != lastFront)//SENSOR PASSAGEIRO---- para sensor sharp
                   while (passengerSensor.readRangeContinuousMillimeters() != lastFront)// para sensor vl53l0x
          {
           
           updateDisplay(frontDistance_1,driverDistance_2,passengerDistance_3,rearDistance_4, "Stopped!");
            turnLeft(500);
        ////////////    if (frontSensor.distance() > 30)//SENSOR DA FRENTE--- para sensor sharp
                  if (frontSensor.readRangeContinuousMillimeters() > 200)// 500 para sensor vl53l0x : VALORES EM MILIMETROS
            {
              break;  
            
            }              
          }
        }                      
      }
      else
      {
        stopTimer++;
        if (stopTimer > 100)
        {
          heading = "Stopped";
          Serial.println ("Stopped-");
          digitalWrite(led1, HIGH); 
          digitalWrite(led2, HIGH); 
          digitalWrite(led3, HIGH); 
          digitalWrite(led4, HIGH); 
          digitalWrite(led5, HIGH); 
          digitalWrite(led6, HIGH); 
          digitalWrite(led7, HIGH); 
          digitalWrite(led8, HIGH); 
          tone(BUZZER_PIN, 200, 400); delay(500); 
          fullStop(100);
          //display stopped
        
         updateDisplay(frontDistance_1,driverDistance_2,passengerDistance_3,rearDistance_4, "Stopped!");
          exit(0);
        }        
      }
    }
  }
  else //loop forever until I put my hand in front of the forward sensor
      // loop para sempre até que eu coloque minha mão na frente do sensor de avanço
  {
 //////////   if (frontDistance < 11)//SENSOR DA FRENTE--- para sensor sharp
        if (frontDistance_1 < 100)// para sensor vl53l0x : VALORES EM MILIMETROS
      {
        delay(1000);
        startup = true;
    }
    else
    {
      delay(20);
    }
  }
}

void fullStop(int duration)
{
    move("A", "FWD", 0);
    move("B", "FWD", 0);   
    delay(duration);
}

void turnRight(int duration)
{
    move("A", "FWD", highSpeed);
    move("B", "REV", highSpeed);
    delay(duration);
}

void turnLeft(int duration)
{
    move("A", "REV", highSpeed);
    move("B", "FWD", highSpeed);
    delay(duration);
}

void move(String channel, String direction, int speed )
{
  int motor = dirA;
  int pwm = speedA;
  int brake = brakeA;
  bool highLow = HIGH;

  if (motorsOn)
  {
    if (direction == "REV")
      highLow = LOW;
      
    if (channel == "B")
    {
      motor = dirB;
      pwm = speedB;
      brake = brakeB;
      //reverse directions for motor B
      // direção reversa para motor B
      highLow = !highLow;
    }
  
    if (speed == 0) //brake
    {
      digitalWrite(brake, HIGH);
    }
    else
    {
      digitalWrite(motor, highLow);
      digitalWrite(brake, LOW);
      analogWrite(pwm, speed);
    }    
  }
}




//void updateDisplay(int fd, int dd, int pd, int rd, float tensao_A2, float tensao_A3,float tensao_A4, float tensao_A5,float distance_1,float distance_2,float distance_3,float distance_4, String hd)
void updateDisplay(uint16_t frontDistance_1,uint16_t driverDistance_2,uint16_t passengerDistance_3,uint16_t rearDistance_4, String hd)

{
  //write to OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("4X_VL53L0X 3D-");
  display.setCursor(0, 5);
  display.print("...............");  
 //--- Display sensor readings---SENSOR--VL53L0X
  //-----sensor---frente
  display.setCursor(0, 16);
  display.print("Frente: ");
  display.setCursor(45, 16);
  display.print(frontDistance_1);
  display.print(" mm-");
  display.setCursor(84, 16);
  
///  display.print("-");
///  display.print(" v");
  //--------sensor esquerdo
  display.setCursor(0, 26);  
  display.print("esquerdo: ");
  display.setCursor(48, 26);  
  display.print(driverDistance_2);
  display.print(" mm-");
  display.setCursor(84, 26);
///  display.print("-");
///  display.print(" v");
  //----------sensor direito
  display.setCursor(0, 36);  
  display.print("direito: ");
  display.setCursor(45, 36);  
  display.print(passengerDistance_3);
  display.print(" mm-");
  display.setCursor(84, 36);
///  display.print("-");
///  display.print(" v");
 //-------sensor re 
  display.setCursor(0, 46);  
  display.print("Re: ");
  display.setCursor(45, 46);  
  display.print(rearDistance_4);
  display.print(" mm-");
  display.setCursor(84, 46);
///  display.print("-");
///  display.print(" v");
//------------------
  display.setCursor(0, 56);  
  display.print("Moving: ");
  display.setCursor(45, 56);  
  display.print(hd);  

  
 /* 
  //--- Display sensor readings---SENSOR--SHARP
  //-----sensor---frente
  display.setCursor(0, 16);
  display.print("Front: ");
  display.setCursor(45, 16);
  display.print(fd);
  display.print(" cm-");
  display.setCursor(84, 16);
  
  display.print(tensao_A4);
  display.print(" v");
  //--------sensor esquerdo
  display.setCursor(0, 26);  
  display.print("Left: ");
  display.setCursor(45, 26);  
  display.print(dd);
  display.print(" cm-");
  display.setCursor(84, 26);
  display.print(tensao_A5);
  display.print(" v");
  //----------sensor direito
  display.setCursor(0, 36);  
  display.print("Right: ");
  display.setCursor(45, 36);  
  display.print(pd);
  display.print(" cm-");
  display.setCursor(84, 36);
  display.print(tensao_A2);
  display.print(" v");
 //-------sensor re 
  display.setCursor(0, 46);  
  display.print("Rear: ");
  display.setCursor(45, 46);  
  display.print(rd);
  display.print(" cm-");
  display.setCursor(84, 46);
  display.print(tensao_A3);
  display.print(" v");
//------------------
  display.setCursor(0, 56);  
  display.print("Moving: ");
  display.setCursor(45, 56);  
  display.print(hd);  
 */

  
  display.display(); 
}
