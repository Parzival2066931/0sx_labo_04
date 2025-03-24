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
unsigned long previousTime = 0;
int deltaTime = 0;
int dist = 0;
const int stepPerTurn = 1019;
int currentPosition = 0;
int maxSpeed = 500;
int speed = 500;
int acceleration = 200;

char msg1[16] = { };
char msg2[16] = { };

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
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  lcdTask(dist, currentTime);
  distance = distanceTask(currentTime, distance, dist);

  switch(distance) {

  case TROP_PRES:
    trop_pres(currentTime);
    break;

  case PARFAITE:
    parfaite(dist, stepPerTurn, currentPosition, currentTime);
    break;

  case TROP_LOIN:
    trop_loin(currentTime);
    break;
  }
  serialTask(dist, currentPosition, currentTime);

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
  static unsigned long lastTime = 0;
  int rate = 100;

  int minDist = 30;
  int maxDist = 60;

  if(ct - lastTime >= rate) {

    dist = hc.dist();

    lastTime = ct;
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
  static int count = 0;
  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(dist);
    lcd.print(" cm  ");
    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");

    lastTime = ct;
  }
}
void trop_pres(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop pres");
    myStepper.disableOutputs();

    lastTime = ct;
  }
  
  
}
void trop_loin(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {

    lcd.setCursor(6, 1);
    lcd.print(" trop loin");
    myStepper.disableOutputs();

    lastTime = ct;
  }
}
void parfaite(int dist, const int stepPerTurn, int &currentPosition, unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  pointeurTask(dist, stepPerTurn, currentPosition, ct);

  if(ct - lastTime >= rate) {

    lcd.print(currentPosition); //position en degrés
    lcd.print(" deg    ");

    lastTime = ct;
  }
}
void pointeurTask(int dist, const int stepPerTurn, int &currentPosition, unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;
 
  int minDist = 30;
  int maxDist = 60;
  int minAngle = 10;
  int maxAngle = 170;
  static int stepsToMove = 0;

  if(ct - lastTime >= rate) {
    currentPosition = map(dist, minDist, maxDist, minAngle, maxAngle);
    stepsToMove = map(dist, minDist, maxDist, 0, stepPerTurn);

    lastTime = ct;
  }

  strcpy(msg2, "");
  strcat(msg2, "Obj  :");
  strcat(msg2, currentPosition);

  myStepper.moveTo(stepsToMove);
  
}
void serialTask(int dist, int deg, unsigned long ct) {

  static unsigned long lastTime = 0;
  int rate = 100;

  if(ct - lastTime >= rate) {

    Serial.print("etd:2066931,dist:");
    Serial.print(dist);
    Serial.print(",deg:");
    Serial.print(deg);

    Serial.print("\t");
    Serial.println(deltaTime);
    lastTime = ct;
  }
  

  

}
