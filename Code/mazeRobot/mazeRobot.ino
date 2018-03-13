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
    LimitSwitch(byte);
    bool isPushed();
  private:
    int pin;
};

LimitSwitch::LimitSwitch(byte addPin)
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
    float getTemperature();

  private:
    float temperature;
    Adafruit_MLX90614 mlx;
};

Thermometer::Thermometer()
{
  mlx.begin();
}

float Thermometer::getTemperature()
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
    DigitalSharp(byte);

    bool isInRange();

  private:
    byte pin;
};

DigitalSharp::DigitalSharp(byte addPin)
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
    Ultrasonic(byte);

    float getDistance();

  private:
    int pingPin;
    NewPing sonar;

};

Ultrasonic::Ultrasonic(byte pin) :
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
    Motor(byte, byte, byte, byte);

    int getEncoderA() {
      return encoderA;
    }
    int getEncoderB() {
      return encoderB;
    }
    unsigned long getOldPos() {
      return oldPos;
    }
    int getLastStateA() {
      return lastStateA;
    }
    unsigned long getEncoderPos() {
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

    void increaseEncoderTotalTurnPos() {
      encoderTotalTurnPos++;
    }

    void resetEncoderTotalTurnPos() {
      encoderTotalTurnPos = 0;
    }

    unsigned long getEncoderTotalTurnPos() {
      return encoderTotalTurnPos;
    }

    float calculateSpeed();
    void forward(int intensity);
    void backward(int intensity);
    void stopMotor();

  private:
    int pinForward;
    int pinBackward;
    int encoderA;
    int encoderB;
    float speed;
    volatile unsigned long encoderPos;
    volatile int currentState = LOW;
    volatile int lastStateA  = LOW;

    unsigned long oldTime;
    unsigned long oldPos;
    unsigned long encoderTotalTurnPos;
};

Motor::Motor(byte pin1, byte pin2, byte eA, byte eB)
{
  pinForward = pin1;
  pinBackward = pin2;

  encoderA = eA;
  encoderB = eB;

  pinMode(pinForward, OUTPUT);
  pinMode(pinBackward, OUTPUT);

  pinMode(encoderA, INPUT);
  digitalWrite(encoderA, HIGH);

  pinMode(encoderB, INPUT);
  digitalWrite(encoderB, HIGH);

  speed = 0;
  encoderPos = 0;

  oldTime = 0;
  oldPos;

  encoderTotalTurnPos = 0;
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
  analogWrite(pinForward, intensity);
  analogWrite(pinBackward, 0);
}

void Motor::backward(int intensity)
{
  analogWrite(pinForward, 0);
  analogWrite(pinBackward, intensity);
}

void Motor::stopMotor()
{
  analogWrite(pinForward, 0);
  analogWrite(pinBackward, 0);
}


//------------------------------------------------

/*-----------------------------------------------
Class Control
*/

class Control
{
public:
  Control();

  Motor& getMotor1() {return motor1;}
  Motor& getMotor2() {return motor2;}
  Motor& getMotor3() {return motor3;}
  Motor& getMotor4() {return motor4;}

  Ultrasonic getUltraSensorRight() {return ultraSensorRight;}
  Ultrasonic getUltraSensorLeft() {return ultraSensorLeft;}
  Ultrasonic getUltraSensorFront() {return ultraSensorFront;}

  void forwardMotors(byte);
  void stopMotors();

  //Simple motor rotation only
  void rotateRight(byte);
  void rotateLeft(byte);

  //90 degree rotation
  void turnRight(byte);
  void turnLeft(byte);

  void forwardTile(byte);

  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;
private:
  

  Ultrasonic ultraSensorRight;
  Ultrasonic ultraSensorLeft;
  Ultrasonic ultraSensorFront;

  int turn90amount;
  int forward30amount;

};

Control::Control() :
motor1(5,4,19,25), motor2(6,7,18,24), motor3(8,9,3,23), motor4(11,10,2,22),
ultraSensorRight(A5), ultraSensorLeft(A9), ultraSensorFront(A8)
{
  turn90amount = 785;
  forward30amount = 1620;
}

void Control::forwardMotors(byte intensity)
{
  motor1.forward(intensity);
  motor2.forward(intensity);
  motor3.forward(intensity);
  motor4.forward(intensity);
}

void Control::stopMotors()
{
  motor1.stopMotor();
  motor2.stopMotor();
  motor3.stopMotor();
  motor4.stopMotor();
}

void Control::rotateRight(byte intensity)
{
  motor1.forward(intensity);
  motor4.forward(intensity);
  motor3.backward(intensity);
  motor2.backward(intensity);
}

void Control::rotateLeft(byte intensity)
{
  motor1.backward(intensity);
  motor4.backward(intensity);
  motor3.forward(intensity);
  motor2.forward(intensity);
}

void Control::turnRight(byte intensity)
{
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  rotateRight(intensity);

  //Add sensor scanning in this area for more precision or obstacles
  while (true)
  {
    if (motor1.getEncoderTotalTurnPos() > turn90amount && motor2.getEncoderTotalTurnPos() > turn90amount &&
        motor3.getEncoderTotalTurnPos() > turn90amount && motor4.getEncoderTotalTurnPos() > turn90amount)
    {
      break;
    }
    
  }
  

  stopMotors();
}

void Control::turnLeft(byte intensity)
{
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  rotateLeft(intensity);

  //Add sensor scanning in this area for more precision or obstacles
  while (true)
  {
    if (motor1.getEncoderTotalTurnPos() > turn90amount && motor2.getEncoderTotalTurnPos() > turn90amount &&
        motor3.getEncoderTotalTurnPos() > turn90amount && motor4.getEncoderTotalTurnPos() > turn90amount)
    {
      break;
    }
  }

  stopMotors();
}

void Control::forwardTile(byte intensity)
{
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  forwardMotors(intensity);
  
  //Add sensor scanning in this area for more precision or obstacles
  while (true)
  {
    if (motor1.getEncoderTotalTurnPos() > forward30amount && motor2.getEncoderTotalTurnPos() > forward30amount &&
        motor3.getEncoderTotalTurnPos() > forward30amount && motor4.getEncoderTotalTurnPos() > forward30amount)
    {
      break;
    }

  }

  stopMotors();
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
  static Control& getControl() {return control;}

  static Control control;
private:

  static byte robotPosX;
  static byte robotPosY;
  static byte robotOrientation;

  static Tile tiles[10][5];

  static byte intensity;

};

byte Navigation::robotOrientation = 0;
byte Navigation::intensity = 150;

byte Navigation::robotPosX = 0;
byte Navigation::robotPosY = 0;
Tile Navigation::tiles[10][5];
Control Navigation::control;

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
      if (control.getUltraSensorRight().getDistance() < 20)
      {
        tiles[robotPosX][robotPosY].wallRight.setWallExists(true);
      }

      if (control.getUltraSensorLeft().getDistance() < 20)
      {
        tiles[robotPosX][robotPosY].wallLeft.setWallExists(true);
      }

      if(control.getUltraSensorFront().getDistance() < 20)
      {
        
      }


      Serial.println(control.getUltraSensorFront().getDistance());
      //Serial.println(control.getUltraSensorRight().getDistance());
      //Serial.println(control.getUltraSensorLeft().getDistance());
      
      break;
      
    case 1:
    
      break;
      
    case 2:
    
      break;
      
    case 3:
    
      break;
      
    default:
      break;
    
  }
}

void Navigation::moveToNextTile()
{
  control.forwardTile(intensity);
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
  if (digitalRead(Navigation::getControl().getMotor1().getEncoderA()))
  {
    Navigation::control.motor1.increaseEncoderPos();
    Navigation::control.motor1.increaseEncoderTotalTurnPos();
  }

}

void encodeInterruptM2()
{
  if (digitalRead(Navigation::getControl().getMotor2().getEncoderA()))
  {
    Navigation::control.motor2.increaseEncoderPos();
    Navigation::control.motor2.increaseEncoderTotalTurnPos();
  }
}

void encodeInterruptM3()
{
  if (digitalRead(Navigation::getControl().getMotor3().getEncoderA()))
  {
    Navigation::control.motor3.increaseEncoderPos();
    Navigation::control.motor3.increaseEncoderTotalTurnPos();
  }
}

void encodeInterruptM4()
{
  if (digitalRead(Navigation::getControl().getMotor4().getEncoderA()))
  {
    Navigation::control.motor4.increaseEncoderPos();
    Navigation::control.motor4.increaseEncoderTotalTurnPos();
  }
}


/*--------------------------------------
*
*
*         MAIN FUNCTIONS
*
*
----------------------------------------
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

  Navigation::start(5);

  Serial.begin(115200);
}

void loop() {

  Navigation::scanSides();

}
