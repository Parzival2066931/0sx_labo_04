#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>



#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 5
#define IN_2 6
#define IN_3 7
#define IN_4 8

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);

//Variables
unsigned long currentTime = 0;
int dist = 0;
const int stepPerTurn = 2038;
int currentPosition = 0;
int maxSpeed = 1000;
int speed = 200;
int acceleration = 200;

enum Distance {TROP_PRES, PARFAITE, TROP_LOIN};

Distance distance = PARFAITE;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.begin();
  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setSpeed(speed);
  myStepper.setAcceleration(acceleration);

  lcdSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  
  lcdTask(dist, currentTime);
  distance = distanceTask(currentTime, distance, dist);

  switch(distance) {

  case TROP_PRES:
    lcd.print("trop pres");
    myStepper.stop();
    break;
  case PARFAITE:
    pointeurTask(dist, stepPerTurn, currentPosition, currentTime);
    lcd.print(currentPosition); //position en degrés
    lcd.print(" deg    ");
    break;
  case TROP_LOIN:
    lcd.print("trop loin");
    myStepper.stop();
    break;
  }
  serialTask(dist, currentPosition);

  myStepper.run();
  
}
void lcdSetup() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2066931");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4B");
  delay(2000);
  lcd.clear();
}
Distance distanceTask(unsigned long ct, Distance distance, int &dist) {
  static unsigned long previousTime = 0;
  int rate = 100;
  int minDist = 30;
  int maxDist = 60;

  if(ct - previousTime >= rate) {

    dist = hc.dist();

    previousTime = ct;
  }
  if(dist < minDist) {
    distance = TROP_PRES;
  }
  else if(dist > maxDist) {
    distance = TROP_LOIN;
  }
  else {
    distance = PARFAITE;
  }
  
  return distance;
}
void lcdTask(int dist, unsigned long ct) {
  static unsigned long previousTime = 0;
  int rate = 100;

  if(ct - previousTime >= rate) {
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(dist);
    lcd.print(" cm  ");
    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");
  }

  
  
  
}
void pointeurTask(int dist, const int stepPerTurn, int &currentPosition, unsigned long ct) {
  static unsigned long previousTime = 0;
  int rate = 100;
 
  int minDist = 30;
  int maxDist = 60;
  int minAngle = 10;
  int maxAngle = 170;
  int stepsToMove = 0;

  if(ct - previousTime >= rate) {
    currentPosition = map(dist, minDist, maxDist, minAngle, maxAngle);
    stepsToMove = map(dist, minDist, maxDist, 0, stepPerTurn);
  }

  myStepper.moveTo(stepsToMove);
  
}
void serialTask(int dist, int deg) {
  Serial.print("etd:2066931,dist:");
  Serial.print(dist);
  Serial.print(",deg:");
  Serial.println(deg);
}
