#include <EnableInterrupt.h>
#include <NewPing.h>
//#include <Adafruit_MLX90614.h>


/*--------------------------------------
*
*
*         GLOBAL VARIABLES
*
*
----------------------------------------
*/

int mazeSizeX = 4;
int mazeSizeY = 4;
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
/*
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
}*/
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

  LimitSwitch getLimitSwitchA(){return LimitSwitchA;}
  LimitSwitch getLimitSwitchB(){return LimitSwitchB;}
  LimitSwitch getLimitSwitchC(){return LimitSwitchC;}
  LimitSwitch getLimitSwitchD(){return LimitSwitchD;}


  void forwardMotors(byte);
  void backwardMotors(byte);
  void stopMotors();

  //Simple motor rotation only
  void rotateRight(byte);
  void rotateLeft(byte);

  //90 degree rotation
  void turnRight(byte);
  void turnLeft(byte);

  void forwardTile(byte);
  void alignForward(byte);
  void backFiveCm(byte);

  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;
private:

  LimitSwitch LimitSwitchA;
  LimitSwitch LimitSwitchB;
  LimitSwitch LimitSwitchC;
  LimitSwitch LimitSwitchD;
  

  Ultrasonic ultraSensorRight;
  Ultrasonic ultraSensorLeft;
  Ultrasonic ultraSensorFront;

  int turn90amount;
  int back5amount;
  int forward30amount;

};

Control::Control() :
motor1(4,5,19,25), motor2(7,6,18,24), motor3(9,8,3,23), motor4(10,11,2,22),
ultraSensorRight(A9), ultraSensorLeft(A8), ultraSensorFront(A3), LimitSwitchA(26), LimitSwitchB(27), LimitSwitchC(28)
, LimitSwitchD(29)
{
  turn90amount = 850;
  forward30amount = 1480;
  back5amount = (forward30amount/6);
}

void Control::forwardMotors(byte intensity)
{
  motor1.forward(intensity);
  motor2.forward(intensity);
  motor3.forward(intensity);
  motor4.forward(intensity);
}

void Control::backwardMotors(byte intensity)
{
  motor1.backward(intensity);
  motor2.backward(intensity);
  motor3.backward(intensity);
  motor4.backward(intensity);
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


void Control::backFiveCm(byte intensity){
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  backwardMotors(intensity);
  
  //Add sensor scanning in this area for more precision or obstacles
  while (true)
  {
    if (motor1.getEncoderTotalTurnPos() > back5amount && motor2.getEncoderTotalTurnPos() > back5amount &&
        motor3.getEncoderTotalTurnPos() > back5amount && motor4.getEncoderTotalTurnPos() > back5amount)
    {
      break;
    }
    
    delay(5);

}
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

void Control::alignForward(byte intensity)
{
  forwardMotors(intensity);

  while(true)
  {
    if (LimitSwitchA.isPushed())
    {
      Serial2.print("n0.val=");
      Serial2.print("\""); 
      Serial2.print("1"); 
      Serial2.print("\"");  
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);
    }

    if (LimitSwitchB.isPushed())
    {
      Serial2.print("n1.val=");
      Serial2.print("\""); 
      Serial2.print("1"); 
      Serial2.print("\"");  
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);
    }

    if (LimitSwitchC.isPushed())
    {
      Serial2.print("n2.val=");
      Serial2.print("\""); 
      Serial2.print("1"); 
      Serial2.print("\"");  
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);
    }

    if (LimitSwitchD.isPushed())
    {
      Serial2.print("n3.val=");
      Serial2.print("\""); 
      Serial2.print("1"); 
      Serial2.print("\"");  
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);
    }
    if((LimitSwitchA.isPushed()||LimitSwitchB.isPushed())&&(LimitSwitchC.isPushed()||LimitSwitchD.isPushed()))
    {
      
      break;
    }
  }
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
  void setFloorType(byte type) {floorType = type;}
  void setSearchNumber(byte number) {searchNumber = number;}
  void setTraceNumber(byte number) {traceNumber = number;}

  bool getIsVisited() {return isVisited;}
  bool getIsNode() {return isNode;}
  bool getIsRobotPresent() {return isRobotPresent;}
  byte getFloorType() {return floorType;}
  byte getSearchNumber() {return searchNumber;}
  byte getTraceNumber() {return traceNumber;}


  Wall wallNorth;
  Wall wallEast;
  Wall wallWest;
  Wall wallSouth;

private:
  byte x;
  byte y;

  bool isVisited, isNode, isRobotPresent;
  byte floorType;
  byte searchNumber;
  byte traceNumber;

};

Tile::Tile()
{
  isVisited = false;
  isNode = false;
  isRobotPresent = false;

  floorType = 0;
  searchNumber = 100;
  traceNumber = 0;

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
  static void adjustToTraceNumber();

  static void moveToNextTile();
  static void adjustToNextMove();
  static Control& getControl() {return control;}

  static byte getRobotPosX() {return robotPosX;}
  static byte getRobotPosY() {return robotPosY;}
  static bool getNodeMode() {return nodeMode;}

  static bool findClosestNode();
  static void tracePath();

  static void checkNodeMode();

  static Control control;

  
private:
  enum robotOrientation {North, South, East, West};
  enum possibleMoves {MoveForward, TurnRight, TurnLeft, DeadEnd};

  static byte robotPosX;
  static byte robotPosY;
  static robotOrientation orientation;

  static possibleMoves nextMove;

  static Tile tiles[4][4];

  static byte intensity;

  static bool nodeMode;
  static bool mazeComplete;

  static byte currentSearchNumber;
  static byte nodePosX;
  static byte nodePosY;

public:
  static possibleMoves decideNextMove(bool, bool, bool);

};

Navigation::robotOrientation Navigation::orientation = North;
Navigation::possibleMoves Navigation::nextMove = MoveForward;

byte Navigation::intensity = 150; 

byte Navigation::currentSearchNumber = 1;
byte Navigation::nodePosX = 0;
byte Navigation::nodePosY = 0;

byte Navigation::robotPosX = 0;
byte Navigation::robotPosY = 0;
Tile Navigation::tiles[4][4];
Control Navigation::control;

bool Navigation::nodeMode = false;
bool Navigation::mazeComplete = false;

void Navigation::start(byte matSize)
{
  for (byte i = 0; i < mazeSizeY; ++i)
  {
    for (byte j = 0; j < mazeSizeX; ++j)
    {
      tiles[i][j].setY(i);
      tiles[i][j].setX(j);
    }
  }

  robotPosX = 0;
  robotPosY = 3;

  tiles[robotPosY][robotPosX].setIsRobotPresent(true);
  tiles[robotPosY][robotPosX].setIsVisited(true);
}

void Navigation::checkNodeMode()
{
  if (nodePosY == robotPosY)
  {
    if (nodePosX == robotPosX)
    {
      nodeMode = false;
    }
  }
}

bool Navigation::findClosestNode()
{
  bool nodeFound = false;
  bool finished = false;

  tiles[robotPosY][robotPosX].setSearchNumber(1);

  currentSearchNumber = 1;

  for (byte i = 0; i < mazeSizeY; ++i)
  {
    for (byte j = 0; j < mazeSizeX; ++j)
    {
      tiles[i][j].setSearchNumber(100);
    }
  }

  while(!finished)
  {
    finished = true;

    for (int i = 0; i < mazeSizeY; ++i)
    {
      for (int j = 0; j < mazeSizeX; ++j)
      {
        if (tiles[i][j].getSearchNumber() == currentSearchNumber)
        {
          //Check north available
          if (!tiles[i-1][j].wallNorth.getWallExists())
          {
            if (tiles[i-1][j].getSearchNumber() > currentSearchNumber+1)
            {
              tiles[i][j].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i-1][j].getIsNode();
              finished = false;
            }
            if (nodeFound)
            {
              nodePosX = j;
              nodePosY = i-1;
              currentSearchNumber++;
              return true;
            }
          }

          //Check East available
          if (!tiles[i][j].wallEast.getWallExists())
          {
            if (tiles[i][j+1].getSearchNumber() > currentSearchNumber+1)
            {
              tiles[i][j+1].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i][j+1].getIsNode();
              finished = false;
            }

            if (nodeFound)
            {
              nodePosX = j+1;
              nodePosY = i;
              currentSearchNumber++;
              return true;
            }
          }

          //Check West available
          if (!tiles[i][j].wallWest.getWallExists())
          {
            if (tiles[i][j-1].getSearchNumber() > currentSearchNumber+1)
            {
              tiles[i][j-1].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i][j-1].getIsNode();
              finished = false;
            }

            if (nodeFound)
            {
              nodePosX = j-1;
              nodePosY = i;
              currentSearchNumber++;
              return true;
            }
          }

          //Check South available
          if (!tiles[i][j].wallSouth.getWallExists())
          {
            if (tiles[i+1][j].getSearchNumber() > currentSearchNumber+1)
            {
              tiles[i+1][j].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i+1][j].getIsNode();
              finished = false;
            }
            
            if (nodeFound)
            {
              nodePosX = j;
              nodePosY = i+1;
              currentSearchNumber++;
              return true;
            }
          }
        }
      }
    }

    currentSearchNumber++;
  }

  return true;
}

void Navigation::tracePath()
{
  byte tracePosX = nodePosX;
  byte tracePosY = nodePosY;

  for (byte i = 0; i < mazeSizeY; ++i)
  {
    for (byte j = 0; j < mazeSizeX; ++j)
    {
      tiles[i][j].setTraceNumber(0);
    }
  }

  byte nextNumber = currentSearchNumber-1;

  tiles[tracePosY][tracePosX].setTraceNumber(currentSearchNumber-nextNumber);

  while (tracePosY != robotPosY && tracePosX != robotPosX)
  {
     //Check North
    if (!tiles[tracePosY][tracePosX].wallNorth.getWallExists())
    {
      if (tiles[tracePosY-1][tracePosX].getSearchNumber() == nextNumber)
      {
        tiles[tracePosY-1][tracePosX].setTraceNumber(currentSearchNumber-nextNumber+1);
        tracePosY = tracePosY-1;
        tracePosX = tracePosX;
        nextNumber--;
        continue;
      }
    }

     //Check East
    if (!tiles[tracePosY][tracePosX].wallEast.getWallExists())
    {
      if (tiles[tracePosY][tracePosX+1].getSearchNumber() == nextNumber)
      {
        tiles[tracePosY][tracePosX+1].setTraceNumber(currentSearchNumber-nextNumber+1);
        tracePosY = tracePosY;
        tracePosX = tracePosX+1;
        nextNumber--;
        continue;
      }
    }

     //Check West
    if (!tiles[tracePosY][tracePosX].wallWest.getWallExists())
    {
      if (tiles[tracePosY][tracePosX-1].getSearchNumber() == nextNumber)
      {
        tiles[tracePosY][tracePosX-1].setTraceNumber(currentSearchNumber-nextNumber+1);
        tracePosY = tracePosY;
        tracePosX = tracePosX-1;
        nextNumber--;
        continue;
      }
    }

     //Check South
    if (!tiles[tracePosY][tracePosX].wallSouth.getWallExists())
    {
      if (tiles[tracePosY+1][tracePosX].getSearchNumber() == nextNumber)
      {
        tiles[tracePosY+1][tracePosX].setTraceNumber(currentSearchNumber-nextNumber+1);
        tracePosY = tracePosY+1;
        tracePosX = tracePosX;
        nextNumber--;
        continue;
      }
    }
  }

  tiles[robotPosY][robotPosY].setTraceNumber(currentSearchNumber);
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
        //ACTIVATE NODE MODE
        nodeMode = true;
        move = DeadEnd;
        mazeComplete = findClosestNode();
        tracePath();
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

  //GET DISTANCES--------------------------

  float ultraDistances[20];
  float sumUltraDistances = 0;

  //Average the readings of the right sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorRight().getDistance();
    sumUltraDistances += ultraDistances[i];
    delay(2);
  }

  float averageDistanceRight = sumUltraDistances / 20;

  sumUltraDistances = 0;
  
  //Average the readings of the left sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorLeft().getDistance();
    sumUltraDistances += ultraDistances[i];
    delay(2);
  }

  float averageDistanceLeft = sumUltraDistances / 20;

  sumUltraDistances = 0;

  //Average the readings of the front sensor
  for (int i = 0; i < 20; ++i)
  {
    ultraDistances[i] = control.getUltraSensorFront().getDistance();
    sumUltraDistances += ultraDistances[i];
    delay(2);
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
        //Serial.print ("Wall Right  ");
        //Serial.println(averageDistanceRight);
        tiles[robotPosY][robotPosX].wallEast.setWallExists(true);
      }

      if (averageDistanceLeft < 20)
      {
        //Serial.print("Wall Left  ");
        //Serial.println(averageDistanceLeft);
        tiles[robotPosY][robotPosX].wallWest.setWallExists(true);
      }

      if(averageDistanceFront < 20)
      {
        //Serial.print("Wall Front  ");
        //Serial.println(averageDistanceFront);
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

void Navigation::adjustToTraceNumber()
{
  //CheckFront
  byte currentTraceNumber = tiles[robotPosY][robotPosX].getTraceNumber();

  switch (orientation)
  {
    case North:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber())
      {
        nextMove = MoveForward;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber())
      {
        nextMove = TurnRight;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber())
      {
        nextMove = DeadEnd;
        return;
      }

      break;

    case East:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber())
      {
        nextMove = MoveForward;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber())
      {
        nextMove = TurnRight;
        return;
      }

      break;
    case West:
      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber())
      {
        nextMove = TurnRight;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber())
      {
        nextMove = MoveForward;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber())
      {
        nextMove = TurnLeft;
        return;
      }
      break;

    case South:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber())
      {
        nextMove = TurnRight;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber())
      {
        nextMove = MoveForward;
        return;
      }

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
      control.alignForward(intensity);
      control.backFiveCm(intensity);
      delay(300);
      control.turnRight(intensity);

      switch(orientation)
      {
        case North:
          orientation = East;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case West:
          orientation = North;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("NORTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          
          break;

        case East:
          orientation = South;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          
          break;

        case South:
          orientation = West;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;
      }

      break;

    case TurnLeft:
      control.alignForward(intensity);
      control.backFiveCm(intensity);
      delay(300);
      control.turnLeft(intensity);

      switch(orientation)
      {
        case North:
          orientation = West;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case West:
          orientation = South;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case East:
          orientation = North;
          break;

        case South:
          orientation = East;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;
      }
      break;

    case DeadEnd:
      control.alignForward(intensity);
      control.backFiveCm(intensity);
      delay(300);
      control.turnRight(intensity);
      control.turnRight(intensity);

      switch(orientation)
      {
        case North:
          orientation = South;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case West:
          orientation = East;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case East:
          orientation = West;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          break;

        case South:
          orientation = North;
          Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("NORTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
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
  Serial2.begin(9600);

  delay(500);
}

void loop()
{
  if (!Navigation::getNodeMode())
  {
    Serial2.print("t1.txt=");
    Serial2.print("\""); 
    Serial2.print("NO NODE"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);
    Navigation::scanSides();
    
    if (Navigation::getNodeMode())
    {
      return;
    }


    delay(100);
    Navigation::adjustToNextMove();
    delay(100);  
    Navigation::moveToNextTile();
    delay(100);
  }
  else
  {
    Serial2.print("t1.txt=");
    Serial2.print("\""); 
    Serial2.print("NODE"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);

    Navigation::adjustToTraceNumber();
    delay(100);
    Navigation::moveToNextTile();
    Navigation::checkNodeMode();
  }

    Serial2.print("n0.val=");
    Serial2.print("\""); 
    Serial2.print("0"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);

    Serial2.print("n1.val=");
    Serial2.print("\""); 
    Serial2.print("0"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);

    Serial2.print("n2.val=");
    Serial2.print("\""); 
    Serial2.print("0"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);

    Serial2.print("n3.val=");
    Serial2.print("\""); 
    Serial2.print("0"); 
    Serial2.print("\"");  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);
}
