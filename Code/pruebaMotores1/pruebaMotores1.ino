#include <EnableInterrupt.h>
#include <NewPing.h>


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
sonar(pin,pin)
{
     pingPin = pin;      
}

float Ultrasonic::getDistance()
{
    unsigned int microSec = sonar.ping();
    return float(microSec)/(US_ROUNDTRIP_CM);  
}


//--------------------------------------

/*---------------------------------------
Class Motor
*/

class Motor
{
public:
    Motor(int, int, int, int);
  
    int getEncoderA() {return encoderA;}
    int getEncoderB() {return encoderB;}
    unsigned long long getOldPos() {return oldPos;}
    int getLastStateA() {return lastStateA;}
    unsigned long long getEncoderPos() {return encoderPos;}
    
    void setEncoderPos(unsigned long long pos) {encoderPos = pos;}
    void setLastStateA(int state) {lastStateA = state;}
  
    float calcularVelocidad();
    void forward();
    void backward();
  
private:
    int pinAdelante;
    int pinAtras;
    int encoderA;
    int encoderB;
    float velocidad;
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
  
    velocidad = 0;
    encoderPos = 0;
    
    oldTime = 0;
    oldPos;
}

float Motor::calcularVelocidad()
{
   unsigned long long newTime = millis();
  
   //newTime = newTime % 10000007;
   velocidad = ((encoderPos - oldPos)*1000)/((float)(newTime-oldTime));
   oldTime = newTime;
   oldPos = encoderPos;
   return velocidad; 
}

void Motor::forward()
{
    analogWrite(pinAdelante, 150);
    analogWrite(pinAtras, 0);
}

void Motor::backward()
{
    analogWrite(pinAdelante, 0);
    analogWrite(pinAtras, 150);
}

//------------------------------------------------


//VARIABLES
volatile int currentState = LOW;
volatile int lastStateA  = LOW;

Motor motor1(5,4,19,25);
Motor motor2(6,7,18,24);
Motor motor3(8,9,3,23);
Motor motor4(11,10,2,22);

volatile long motor1Pos = 0;
volatile long motor2Pos = 0;
volatile long motor3Pos = 0;
volatile long motor4Pos = 0;

float velocidadMotor1 = 0;
float velocidadMotor2 = 0;
float velocidadMotor3 = 0;
float velocidadMotor4 = 0;

Ultrasonic ultraSensor(A9);
DigitalSharp sharpSensor(A6);

//INTERRUPTS
void encodeInterruptM1()
{   
    currentState = digitalRead(motor1.getEncoderA());
    if(currentState == HIGH)
    {
        motor1Pos++;
    }

    motor1.setEncoderPos(motor1Pos);
    motor1.setLastStateA(currentState);
}

void encodeInterruptM2()
{
    currentState = digitalRead(motor2.getEncoderA());
    if(currentState == HIGH)
    {
        motor2Pos++;
    }

    motor2.setEncoderPos(motor2Pos);
    motor2.setLastStateA(currentState);
}

void encodeInterruptM3()
{
    currentState = digitalRead(motor3.getEncoderA());
    if(currentState == HIGH)
    {
        motor3Pos++;
    }

    motor3.setEncoderPos(motor3Pos);
    motor3.setLastStateA(currentState);
}

void encodeInterruptM4()
{
    currentState = digitalRead(motor4.getEncoderA());
    if(currentState == HIGH)
    {
        motor4Pos++;
    }

    motor4.setEncoderPos(motor4Pos);
    motor4.setLastStateA(currentState);
}
//----------------------------------------------

//--------------------------------------------
//Motor Move functions
void forward()
{
    motor1.forward();
    motor2.forward();
    motor3.forward();
    motor4.forward();

    velocidadMotor1 = motor1.calcularVelocidad();
    velocidadMotor2 = motor2.calcularVelocidad();
    velocidadMotor3 = motor3.calcularVelocidad();
    velocidadMotor4 = motor4.calcularVelocidad();
}

void backward()
{
    motor1.backward();
    motor2.backward();
    motor3.backward();
    motor4.backward();
}

void rotate(bool isRight)
{
    if(isRight)
    {
        motor1.forward();
        motor4.forward();
        motor3.backward();
        motor2.backward();
    }
    else
    {
        motor1.backward();
        motor2.backward();
        motor3.forward();
        motor4.forward();
    }
}

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
 
    if (sharpSensor.isInRange())
    {
        Serial.println("En rango");    
    }
    else
    {
        Serial.println("Fuera de rango");
    }
    
}

