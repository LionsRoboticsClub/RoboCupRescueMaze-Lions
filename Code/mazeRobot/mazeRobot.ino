#include <EnableInterrupt.h>
#include <NewPing.h>
#include <Adafruit_MLX90614.h>

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


//VARIABLES
volatile int currentState = LOW;
volatile int lastStateA  = LOW;
volatile bool interruptActive = false;
volatile bool firstTurn90 = true;

int turn90amount = 785;

long encoder1TotalTurnPos = 0;
long encoder2TotalTurnPos = 0;
long encoder3TotalTurnPos = 0;
long encoder4TotalTurnPos = 0;

byte gintensity = 250;


//Class declarations
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

//--------------------------------------------------
//INTERRUPTS
void encodeInterruptM1()
{
  currentState = digitalRead(motor1.getEncoderA());
  if (currentState == HIGH)
  {
    motor1Pos++;
    encoder1TotalTurnPos++;
  }

  motor1.setEncoderPos(motor1Pos);
  motor1.setLastStateA(currentState);
}

void encodeInterruptM2()
{
  currentState = digitalRead(motor2.getEncoderA());
  if (currentState == HIGH)
  {
    motor2Pos++;
    interruptActive = true;
    encoder2TotalTurnPos++;
  }

  motor2.setEncoderPos(motor2Pos);
  motor2.setLastStateA(currentState);
}

void encodeInterruptM3()
{

  currentState = digitalRead(motor3.getEncoderA());
  if (currentState == HIGH)
  {
    motor3Pos++;
    encoder3TotalTurnPos++;
  }

  motor3.setEncoderPos(motor3Pos);
  motor3.setLastStateA(currentState);
}

void encodeInterruptM4()
{
  currentState = digitalRead(motor4.getEncoderA());
  if (currentState == HIGH)
  {
    motor4Pos++;
    encoder4TotalTurnPos++;
  }

  motor4.setEncoderPos(motor4Pos);
  motor4.setLastStateA(currentState);
}
//----------------------------------------------

//---------------------------------------------
//Motor move functions

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

//void forward30(gintensity)
//{
   //forwardAmount
//}

void setup() {

  enableInterrupt(motor1.getEncoderA(), encodeInterruptM1, CHANGE);
  enableInterrupt(motor1.getEncoderB(), encodeInterruptM1, CHANGE);
  enableInterrupt(motor2.getEncoderA(), encodeInterruptM2, CHANGE);
  enableInterrupt(motor2.getEncoderB(), encodeInterruptM2, CHANGE);
  enableInterrupt(motor3.getEncoderA(), encodeInterruptM3, CHANGE);
  enableInterrupt(motor3.getEncoderB(), encodeInterruptM3, CHANGE);
  enableInterrupt(motor4.getEncoderA(), encodeInterruptM4, CHANGE);
  enableInterrupt(motor4.getEncoderB(), encodeInterruptM4, CHANGE);

  Serial.begin(9600);
}

void loop() {

  forward(gintensity);
  delay(5000);

  stopMotors();
  delay(5000);

  //conciousTurn90Right(gintensity);

}
