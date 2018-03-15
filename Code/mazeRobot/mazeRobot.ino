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
motor1(4,5,19,25), motor2(7,6,18,24), motor3(9,8,3,23), motor4(10,11,2,22),
ultraSensorRight(A9), ultraSensorLeft(A8), ultraSensorFront(A3)
{
  turn90amount = 850;
  forward30amount = 1480;
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
  motor1.backward(intensity);
  motor4.backward(intensity);
  motor3.forward(intensity);
  motor2.forward(intensity);
}

void Control::rotateLeft(byte intensity)
{
  motor1.forward(intensity);
  motor4.forward(intensity);
  motor3.backward(intensity);
  motor2.backward(intensity);
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

    ultraSensorRight.getDistance();
    ultraSensorLeft.getDistance();
    ultraSensorFront.getDistance();

    delay(5);
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
    
    ultraSensorRight.getDistance();
    ultraSensorLeft.getDistance();
    ultraSensorFront.getDistance();
    
    delay(5);
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

    ultraSensorRight.getDistance();
    ultraSensorLeft.getDistance();
    ultraSensorFront.getDistance();
    
    delay(5);

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

  Wall wallNorth;
  Wall wallEast;
  Wall wallWest;
  Wall wallSouth;

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
  static void adjustToNextMove();
  static Control& getControl() {return control;}

  static byte getRobotPosX() {return robotPosX;}
  static byte getRobotPosY() {return robotPosY;}

  static Control control;

  
private:
  enum robotOrientation {North, South, East, West};
  enum possibleMoves {MoveForward, TurnRight, TurnLeft, DeadEnd};

  static byte robotPosX;
  static byte robotPosY;
  static robotOrientation orientation;

  static possibleMoves nextMove;

  static Tile tiles[3][3];

  static byte intensity;

public:
  static possibleMoves decideNextMove(bool, bool, bool);

};

Navigation::robotOrientation Navigation::orientation = North;
Navigation::possibleMoves Navigation::nextMove = MoveForward;

byte Navigation::intensity = 150;

byte Navigation::robotPosX = 0;
byte Navigation::robotPosY = 0;
Tile Navigation::tiles[3][3];
Control Navigation::control;

void Navigation::start(byte matSize)
{
  for (byte i = 0; i < 3; ++i)
  {
    for (byte j = 0; j < 3; ++j)
    {
      tiles[i][j].setX(i);
      tiles[i][j].setX(j);
    }
  }

  robotPosX = 0;
  robotPosY = 2;

  tiles[robotPosY][robotPosX].setIsRobotPresent(true);
  tiles[robotPosY][robotPosX].setIsVisited(true);
}

Navigation::possibleMoves Navigation::decideNextMove(bool frontAvailable, bool rightAvailable, bool leftAvailable)
{
  possibleMoves move;

  if (frontAvailable)
  {
    move = MoveForward;

    if (rightAvailable)
    {
      //There are several possible moves
      tiles[robotPosY][robotPosX].setIsNode(true);
    }
    else
    {
      if(leftAvailable)
      {
        //There are several possible moves
        tiles[robotPosY][robotPosX].setIsNode(true);
      }
    }
  }
  else
  {
    if (rightAvailable)
    {
      move = TurnRight;

      if (leftAvailable)
      {
        //There are several possible moves
        tiles[robotPosY][robotPosX].setIsNode(true);
      }
    }
    else
    {
      if (leftAvailable)
      {
        move = TurnLeft;
      }
      else
      {
        move = DeadEnd;
      }
    }
  }

  return move;
}

void Navigation::scanSides()
{
  bool frontAvailable;
  bool rightAvailable;
  bool leftAvailable;

  float ultraDistances[20];
  float sumUltraDistances = 0;

  //GET DISTANCES--------------------------

  //Average the readings of the right sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorRight().getDistance();
    sumUltraDistances += ultraDistances[i];
  }
  
  float averageDistanceRight = sumUltraDistances / 20;

  sumUltraDistances = 0;
  
  //Average the readings of the left sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorLeft().getDistance();
    sumUltraDistances += ultraDistances[i];
  }

  float averageDistanceLeft = sumUltraDistances / 20;

  sumUltraDistances = 0;

  //Average the readings of the front sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorFront().getDistance();
    sumUltraDistances += ultraDistances[i];
  }

  float averageDistanceFront = sumUltraDistances / 20;

  sumUltraDistances = 0;

  //-------------------------------------------------------------

  switch (orientation)
  {
    case North:

      //Scan all sides
      if (averageDistanceRight < 20)
      {
        Serial.println("Wall Right");
        tiles[robotPosY][robotPosX].wallEast.setWallExists(true);
      }

      if (averageDistanceLeft < 20)
      {
        Serial.println("Wall Left");
        tiles[robotPosY][robotPosX].wallWest.setWallExists(true);
      }

      if(averageDistanceFront < 20)
      {
        Serial.println("Wall Front");
        tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);
      }

      frontAvailable = !tiles[robotPosY][robotPosX].wallNorth.getWallExists() && !tiles[robotPosY-1][robotPosX].getIsVisited();
      rightAvailable = !tiles[robotPosY][robotPosX].wallEast.getWallExists() && !tiles[robotPosY][robotPosX+1].getIsVisited();
      leftAvailable = !tiles[robotPosY][robotPosX].wallWest.getWallExists() && !tiles[robotPosY][robotPosX-1].getIsVisited();
      
      //Decide robots next move
      nextMove = decideNextMove(frontAvailable, rightAvailable, leftAvailable);

      break;
      
    case East:
      
      if (averageDistanceRight < 20)
      {
        tiles[robotPosY][robotPosX].wallSouth.setWallExists(true);
      }

      if (averageDistanceLeft < 20)
      {
        tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);
      }

      if(averageDistanceFront < 20)
      {
        tiles[robotPosY][robotPosX].wallEast.setWallExists(true);
      }

      frontAvailable = !tiles[robotPosY][robotPosX].wallEast.getWallExists() && !tiles[robotPosY][robotPosX+1].getIsVisited();
      rightAvailable = !tiles[robotPosY][robotPosX].wallSouth.getWallExists() && !tiles[robotPosY+1][robotPosX].getIsVisited();
      leftAvailable = !tiles[robotPosY][robotPosX].wallNorth.getWallExists() && !tiles[robotPosY-1][robotPosX].getIsVisited();
      
      //Decide robots next move
      nextMove = decideNextMove(frontAvailable, rightAvailable, leftAvailable);

      break;
      
    case West:
      
      if (averageDistanceRight < 20)
      {
        tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);
      }

      if (averageDistanceLeft < 20)
      {
        tiles[robotPosY][robotPosX].wallSouth.setWallExists(true);
      }

      if(averageDistanceFront < 20)
      {
        tiles[robotPosY][robotPosX].wallWest.setWallExists(true);
      }

      frontAvailable = !tiles[robotPosY][robotPosX].wallWest.getWallExists() && !tiles[robotPosY][robotPosX-1].getIsVisited();
      rightAvailable = !tiles[robotPosY][robotPosX].wallNorth.getWallExists() && !tiles[robotPosY-1][robotPosX].getIsVisited();
      leftAvailable = !tiles[robotPosY][robotPosX].wallSouth.getWallExists() && !tiles[robotPosY+1][robotPosX].getIsVisited();
      
      //Decide robots next move
      nextMove = decideNextMove(frontAvailable, rightAvailable, leftAvailable);

      break;
      
    case South:

      if (averageDistanceRight < 20)
      {
        tiles[robotPosY][robotPosX].wallWest.setWallExists(true);
      }

      if (averageDistanceLeft < 20)
      {
        tiles[robotPosY][robotPosX].wallEast.setWallExists(true);
      }

      if(averageDistanceFront < 20)
      {
        tiles[robotPosY][robotPosX].wallSouth.setWallExists(true);
      }

      frontAvailable = !tiles[robotPosY][robotPosX].wallSouth.getWallExists() && !tiles[robotPosY+1][robotPosX].getIsVisited();
      rightAvailable = !tiles[robotPosY][robotPosX].wallWest.getWallExists() && !tiles[robotPosY][robotPosX-1].getIsVisited();
      leftAvailable = !tiles[robotPosY][robotPosX].wallEast.getWallExists() && !tiles[robotPosY][robotPosX+1].getIsVisited();

      //Decide robots next move
      nextMove = decideNextMove(frontAvailable, rightAvailable, leftAvailable); 

      break;
      
    default:
      break;  
  }
}

void Navigation::adjustToNextMove()
{
  switch(nextMove)
  {
    case MoveForward:

      break;

    case TurnRight:
      control.turnRight(intensity);

      switch(orientation)
      {
        case North:
          orientation = East;

          break;

        case West:
          orientation = North;
          break;

        case East:
          orientation = South;
          break;

        case South:
          orientation = West;
          break;
      }

      break;

    case TurnLeft:
      control.turnLeft(intensity);

      switch(orientation)
      {
        case North:
          orientation = West;
          break;

        case West:
          orientation = South;
          break;

        case East:
          orientation = North;
          break;

        case South:
          orientation = East;
          break;
      }
      break;

    case DeadEnd:
      control.turnRight(intensity);
      control.turnRight(intensity);

      switch(orientation)
      {
        case North:
          orientation = South;
          break;

        case West:
          orientation = East;
          break;

        case East:
          orientation = West;
          break;

        case South:
          orientation = North;
          break;
      }

      break;
  }
}

void Navigation::moveToNextTile()
{
  control.forwardTile(intensity);

  switch(orientation)
  {
    case North:
      robotPosY--;
      break;

    case West:
      robotPosX--;
      break;

    case East:
      robotPosX++;
      break;

    case South:
      robotPosY++;
      break;
  }

  tiles[robotPosY][robotPosX].setIsRobotPresent(true);
  tiles[robotPosY][robotPosX].setIsVisited(true);
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

  Serial.begin(9600);

  delay(2000);
}

void loop() {

  Serial.print(Navigation::getRobotPosX());
  Serial.print(" , ");
  Serial.println(Navigation::getRobotPosY());
  Navigation::scanSides();
  
  delay(1000);

  Navigation::adjustToNextMove();

  delay(2000);
  
  Navigation::moveToNextTile();

  delay(2000);


}
