#include <EnableInterrupt.h>
#include <NewPing.h>
#include <Adafruit_MLX90614.h>

/*--------------------------------------
*
*
*         GLOBAL VARIABLES
*
*
----------------------------------------
*/

volatile int currentState = LOW;
volatile int lastStateA  = LOW;
volatile bool interruptActive = false;
volatile bool firstTurn90 = true;

volatile long motor1Pos = 0;
volatile long motor2Pos = 0;
volatile long motor3Pos = 0;
volatile long motor4Pos = 0;

/*--------------------------------------
*
*
*         SENSORS
*
*
----------------------------------------
*/

/*--------------------------------------
  Class LimitSwitch
*/
class LimitSwitch
{
  public:
    LimitSwitch(int);
    bool isPushed();
  private:
    int pin;
};

LimitSwitch::LimitSwitch(int addPin)
{
  pin = addPin;
  pinMode(pin, INPUT);
}

bool LimitSwitch::isPushed()
{
  return digitalRead(pin);
}
//--------------------------------------

/*--------------------------------------
  Class Thermometer
*/
class Thermometer
{
  public:
    Thermometer();
    double getTemperature();

  private:
    double temperature;
    Adafruit_MLX90614 mlx;
};

Thermometer::Thermometer()
{
  mlx.begin();
}

double Thermometer::getTemperature()
{
  return mlx.readObjectTempC();
}
//---------------------------------------

/*---------------------------------------
  Class DigitalSharp
*/

class DigitalSharp
{
  public:
    DigitalSharp(int);

    bool isInRange();

  private:
    int pin;
};

DigitalSharp::DigitalSharp(int addPin)
{
  pin = addPin;
  pinMode(pin, INPUT);
}

bool DigitalSharp::isInRange()
{
  return !digitalRead(pin);
}

//--------------------------------------

/*---------------------------------------
  Class Ultrasonic
*/

class Ultrasonic
{
  public:
    Ultrasonic(int);

    float getDistance();

  private:
    int pingPin;
    NewPing sonar;

};

Ultrasonic::Ultrasonic(int pin) :
  sonar(pin, pin)
{
  pingPin = pin;
}

float Ultrasonic::getDistance()
{
  unsigned int microSec = sonar.ping();
  return float(microSec) / (US_ROUNDTRIP_CM);
}


//--------------------------------------

/*--------------------------------------
*
*
*         MOTORS AND CONTROL
*
*
----------------------------------------
*/

/*---------------------------------------
  Class Motor
*/

class Motor
{
  public:
    Motor(int, int, int, int);

    int getEncoderA() {
      return encoderA;
    }
    int getEncoderB() {
      return encoderB;
    }
    unsigned long long getOldPos() {
      return oldPos;
    }
    int getLastStateA() {
      return lastStateA;
    }
    unsigned long long getEncoderPos() {
      return encoderPos;
    }

    void setEncoderPos(unsigned long long pos) {
      encoderPos = pos;
    }
    void setLastStateA(int state) {
      lastStateA = state;
    }

    void increaseEncoderPos() {
      encoderPos++;
    }


    float calculateSpeed();
    void forward(int intensity);
    void backward(int intensity);
    void stopMotor();

  private:
    int pinAdelante;
    int pinAtras;
    int encoderA;
    int encoderB;
    float speed;
    volatile unsigned long long encoderPos;
    volatile int currentState = LOW;
    volatile int lastStateA  = LOW;

    unsigned long long oldTime;
    unsigned long long oldPos;
};

Motor::Motor(int pin1, int pin2, int eA, int eB)
{
  pinAdelante = pin1;
  pinAtras = pin2;

  encoderA = eA;
  encoderB = eB;

  pinMode(pinAdelante, OUTPUT);
  pinMode(pinAtras, OUTPUT);

  pinMode(encoderA, INPUT);
  digitalWrite(encoderA, HIGH);

  pinMode(encoderB, INPUT);
  digitalWrite(encoderB, HIGH);

  speed = 0;
  encoderPos = 0;

  oldTime = 0;
  oldPos;
}

float Motor::calculateSpeed()
{
  unsigned long long newTime = millis();

  //newTime = newTime % 10000007;
  speed = ((encoderPos - oldPos) * 1000) / ((float)(newTime - oldTime));
  oldTime = newTime;
  oldPos = encoderPos;
  return speed;
}

void Motor::forward(int intensity)
{
  analogWrite(pinAdelante, intensity);
  analogWrite(pinAtras, 0);
}

void Motor::backward(int intensity)
{
  analogWrite(pinAdelante, 0);
  analogWrite(pinAtras, intensity);
}

void Motor::stopMotor()
{
  analogWrite(pinAdelante, 0);
  analogWrite(pinAtras, 0);
}


//------------------------------------------------

/*-----------------------------------------------
Class Control
*/

class Control
{
public:
  Control();

  Motor getMotor1() {return motor1;}
  Motor getMotor2() {return motor2;}
  Motor getMotor3() {return motor3;}
  Motor getMotor4() {return motor4;}

  void forwardTile();

private:
  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;

  Ultrasonic ultraSensorRight;
  Ultrasonic ultraSensorLeft;

};

Control::Control() :
motor1(5,4,19,25), motor2(6,7,18,24), motor3(8,9,3,23), motor4(11,10,2,22),
ultraSensorRight(A5), ultraSensorLeft(A9)
{

}

void Control::forwardTile()
{
  
}

/*--------------------------------------
*
*
*         NAVIGATION, WALLS, AND TILES
*
*
----------------------------------------
*/
/*--------------------------------------
Class Wall
*/

class Wall
{
public:
  Wall();

  void setWallExists(bool value) {wallExists = value;}
  void setHeatedVictim(bool value) {heatedVictim = value;}
  void setHarmedVictim(bool value) {harmedVictim = value;}
  void setStableVictim(bool value) {stableVictim = value;}
  void setUnharmedVictim(bool value) {unharmedVictim = value;}

  bool getWallExists() {return wallExists;}
  bool getHeatedVictim() {return heatedVictim;}
  bool getHarmedVictim() {return harmedVictim;}
  bool getStableVictim() {return stableVictim;}
  bool getUnharmedVictim() {return unharmedVictim;}

private:

  bool wallExists;
  bool heatedVictim, harmedVictim, stableVictim, unharmedVictim;
};

Wall::Wall()
{
  wallExists = false;
  heatedVictim = false;
  harmedVictim = false;
  stableVictim = false;
  unharmedVictim = false;
}
//--------------------------------------

/*--------------------------------------
Class Tile
*/
class Tile
{
public:
  Tile();

  byte getX() {return x;}
  byte getY() {return y;}

  void setX(byte nx) {x = nx;}
  void setY(byte ny) {y = ny;}
  void setIsVisited(bool value) {isVisited = value;}
  void setIsRobotPresent(bool value) {isRobotPresent = value;}
  void setIsNode(bool value) {isNode = value;}

  bool getIsVisited() {return isVisited;}
  bool getIsNode() {return isNode;}
  bool getIsRobotPresent() {return isRobotPresent;}

  Wall wallFront;
  Wall wallRight;
  Wall wallLeft;
  Wall wallBack;

private:
  byte x;
  byte y;

  bool isVisited, isNode, isRobotPresent;
  byte floorType;

};

Tile::Tile()
{
  isVisited = false;
  isNode = false;
  isRobotPresent = false;

  byte floorType = 0;
}
//--------------------------------------

/*--------------------------------------
Class Navigation
*/
class Navigation
{
public:

  static void start(byte matSize);

  static void scanSides();
  static void moveToNextTile();
  static Control getControl() {return control;}

private:
  static Control control;

  static byte robotPosX;
  static byte robotPosY;
  static byte robotOrientation;

  static Tile tiles[30][30];

};

byte Navigation::robotOrientation = 0;

byte Navigation::robotPosX = 0;
byte Navigation::robotPosY = 0;
Tile Navigation::tiles[30][30];

void Navigation::start(byte matSize)
{
  for (byte i = 0; i < matSize; ++i)
  {
    for (byte j = 0; j < matSize; ++j)
    {
      tiles[i][j].setX(i);
      tiles[i][j].setX(j);
    }
  }

  robotPosX = matSize/2;
  robotPosY = matSize/2;

  tiles[robotPosX][robotPosY].setIsRobotPresent(true);

  robotOrientation = 0;
}

void Navigation::scanSides()
{
  switch (robotOrientation)
  {
    case 0:
      if (ultraSensorRight.getDistance() < 20)
      {
        tile[robotPosX][robotPosY].wallRight.setWallExists(true);
      }

      if (ultraSensorLeft.getDistance() < 20)
      {
        tile[robotPosX][robotPosY].wallRight.setWallExists(true);
      }

      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
  }
}

void Navigation::moveToNextTile()
{

}


/*--------------------------------------
*
*
*         INTERRUPTS
*
*
----------------------------------------
*/

//Interrupts for Motors

void encodeInterruptM1()
{
  currentState = digitalRead(Navigation::getControl().getMotor1().getEncoderA());
  if (currentState == HIGH)
  {
    Navigation::getControl().getMotor1().increaseEncoderPos();
    encoder1TotalTurnPos++;
  }

}

void encodeInterruptM2()
{
  currentState = digitalRead(Navigation::getControl().getMotor2().getEncoderA());
  if (currentState == HIGH)
  {
    Navigation::getControl().getMotor2().increaseEncoderPos();
    interruptActive = true;
    encoder2TotalTurnPos++;
  }
}

void encodeInterruptM3()
{

  currentState = digitalRead(Navigation::getControl().getMotor3().getEncoderA());
  if (currentState == HIGH)
  {
    Navigation::getControl().getMotor3().increaseEncoderPos();
    encoder3TotalTurnPos++;
  }
}

void encodeInterruptM4()
{
  currentState = digitalRead(Navigation::getControl().getMotor4().getEncoderA());
  if (currentState == HIGH)
  {
    Navigation::getControl().getMotor4().increaseEncoderPos();
    encoder4TotalTurnPos++;
  }
}

/*
//VARIABLES
volatile int currentState = LOW;
volatile int lastStateA  = LOW;
volatile bool interruptActive = false;
volatile bool firstTurn90 = true;

int turn90amount = 785;
int forward30amount = 1620;

long encoder1TotalTurnPos = 0;
long encoder2TotalTurnPos = 0;
long encoder3TotalTurnPos = 0;
long encoder4TotalTurnPos = 0;

byte gintensity = 250;
*/
//Class declarations
/*
Motor motor1(5, 4, 19, 25);
Motor motor2(6, 7, 18, 24);
Motor motor3(8, 9, 3, 23);
Motor motor4(11, 10, 2, 22);

volatile long motor1Pos = 0;
volatile long motor2Pos = 0;
volatile long motor3Pos = 0;
volatile long motor4Pos = 0;

float speedMotor1 = 0;
float speedMotor2 = 0;
float speedMotor3 = 0;
float speedMotor4 = 0;

Ultrasonic ultraSensor(A9);
DigitalSharp sharpSensor(A6);
LimitSwitch limitSensor(A2);
*/

//---------------------------------------------
//Motor move functions

/*
void stopMotors()
{
  motor1.stopMotor();
  motor2.stopMotor();
  motor3.stopMotor();
  motor4.stopMotor();
}

void forward(int intensity)
{
  motor1.forward(intensity);
  motor2.forward(intensity);
  motor3.forward(intensity);
  motor4.forward(intensity);

  speedMotor1 = motor1.calculateSpeed();
  speedMotor2 = motor2.calculateSpeed();
  speedMotor3 = motor3.calculateSpeed();
  speedMotor4 = motor4.calculateSpeed();
}

void backward(int intensity)
{
  motor1.backward(intensity);
  motor2.backward(intensity);
  motor3.backward(intensity);
  motor4.backward(intensity);

  speedMotor1 = motor1.calculateSpeed();
  speedMotor2 = motor2.calculateSpeed();
  speedMotor3 = motor3.calculateSpeed();
  speedMotor4 = motor4.calculateSpeed();
}

void rotate(bool isRight, int intensity)
{
  if (isRight)
  {
    motor1.forward(intensity);
    motor4.forward(intensity);
    motor3.backward(intensity);
    motor2.backward(intensity);
  }
  else
  {
    motor1.backward(intensity);
    motor2.backward(intensity);
    motor3.forward(intensity);
    motor4.forward(intensity);
  }
}

void turnLeft(int intensity) {

  motor1.backward(intensity);
  motor2.backward(intensity);
  motor3.forward(intensity);
  motor4.forward(intensity);
}

void turnRight(int intensity) {

  motor1.forward(intensity);
  motor4.forward(intensity);
  motor3.backward(intensity);
  motor2.backward(intensity);
}

void turn90degRight(int intensity) {

  if (firstTurn90)
  {
    turnRight(intensity);
    encoder2TotalTurnPos = 0;
    encoder3TotalTurnPos = 0;
    encoder4TotalTurnPos = 0;
    firstTurn90 = false;
  }


  if (encoder2TotalTurnPos > turn90amount && encoder3TotalTurnPos > turn90amount && encoder4TotalTurnPos > turn90amount)
  {
    turnRight(intensity);
    firstTurn90 = true;
  }
}

void turn90degLeft(int intensity) {

  if (firstTurn90)
  {
    turnLeft(intensity);
    encoder2TotalTurnPos = 0;
    encoder3TotalTurnPos = 0;
    encoder4TotalTurnPos = 0;
    firstTurn90 = false;
  }


  if (encoder2TotalTurnPos > turn90amount && encoder3TotalTurnPos > turn90amount && encoder4TotalTurnPos > turn90amount)
  {
    turnRight(intensity);
    firstTurn90 = true;
  }
}

void conciousTurn90Left(int intensity)
{
  do
  {
    turn90degLeft(intensity);

    speedMotor1 = motor1.calculateSpeed();
    speedMotor2 = motor2.calculateSpeed();
    speedMotor3 = motor3.calculateSpeed();
    speedMotor4 = motor4.calculateSpeed();

    Serial.println("Vel Motor1: ");
    Serial.println(speedMotor1);
    Serial.println("Vel Motor2: ");
    Serial.println(speedMotor2);
    Serial.println("Vel Motor3: ");
    Serial.println(speedMotor3);
    Serial.println("Vel Motor4: ");
    Serial.println(speedMotor4);

  }
  while (!firstTurn90);

  stopMotors();
  delay(500);
}

void conciousTurn90Right(int intensity)
{
  do
  {
    turn90degRight(intensity);

    speedMotor1 = motor1.calculateSpeed();
    speedMotor2 = motor2.calculateSpeed();
    speedMotor3 = motor3.calculateSpeed();
    speedMotor4 = motor4.calculateSpeed();

    Serial.println("Vel Motor1: ");
    Serial.println(speedMotor1);
    Serial.println("Vel Motor2: ");
    Serial.println(speedMotor2);
    Serial.println("Vel Motor3: ");
    Serial.println(speedMotor3);
    Serial.println("Vel Motor4: ");
    Serial.println(speedMotor4);

  }
  while (!firstTurn90);

  stopMotors();
  delay(500);
}

void forwardAmount(int intensity)
{
  if (firstTurn90)
  {
    forward(intensity);
    encoder2TotalTurnPos = 0;
    encoder3TotalTurnPos = 0;
    encoder4TotalTurnPos = 0;
    firstTurn90 = false;
  }


  if (encoder2TotalTurnPos > forward30amount && encoder3TotalTurnPos > forward30amount && encoder4TotalTurnPos > forward30amount)
  {
    forward(intensity);
    firstTurn90 = true;
  }

  Serial.println(encoder2TotalTurnPos);
}

void forward30(int intensity)
{
  do
  {
    forwardAmount(intensity);

  }
  while (!firstTurn90);

  stopMotors();

}
*/
void setup() {

  enableInterrupt(Navigation::getControl().getMotor1().getEncoderA(), encodeInterruptM1, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor1().getEncoderB(), encodeInterruptM1, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor2().getEncoderA(), encodeInterruptM2, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor2().getEncoderB(), encodeInterruptM2, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor3().getEncoderA(), encodeInterruptM3, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor3().getEncoderB(), encodeInterruptM3, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor4().getEncoderA(), encodeInterruptM4, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor4().getEncoderB(), encodeInterruptM4, CHANGE);

  Navigation::start(20);

  Serial.begin(9600);
}

void loop() {


}
