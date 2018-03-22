
//#include <i2cmaster.h>
//#include <Adafruit_TCS34725.h>
#include <EnableInterrupt.h>
#include <NewPing.h>
#include <Wire.h>


/*--------------------------------------
*
*
*         GLOBAL VARIABLES
*
*
----------------------------------------
*/

int mazeSizeX = 7;
int mazeSizeY = 7;

int mazeSizeX2 = 7;
int mazeSizeY2 = 7;

int robotStartPosX = 0;
int robotStartPosY = 6;

int luxBlack = 100;
int luxWhite = 700;
int luxCheckpoint = 700;

Nextion myNextion;

void checkCheckpointButton();

/*--------------------------------------
*
*
*         SENSORS
*
*
----------------------------------------
*/

/*--------------------------------------
  Class ColorSensor
*/

class ColorSensor
{
public:
  ColorSensor();

  byte calculateColor();

private:
  //Adafruit_TCS34725 tcs;
  byte redPin, greenPin, bluePin;
};

ColorSensor::ColorSensor()
{
  //tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  //tcs.begin();

  
}

byte ColorSensor::calculateColor()
{
  return 1;
}
//-------------------------------------------------

/*--------------------------------------
  Class LimitSwitch
*/
class LimitSwitch
{
  public:
    LimitSwitch(byte);
    int isPushed();
  private:
    int pin;
};

LimitSwitch::LimitSwitch(byte addPin)
{
  pin = addPin;
  pinMode(pin, INPUT);
}

int LimitSwitch::isPushed()
{
  int value = analogRead(pin);

  return value;
}
//-------------------------------------------------

/*--------------------------------------
  Class Thermometer
*/

class Thermometer
{
public: 
  Thermometer(int ad);
  float getTemperature();

private: 
  float temperature;
  int address;
};

Thermometer::Thermometer(int ad)
{
  address = ad;
}
/*
float Thermometer::getTemperature()
{
  int melexisAddress = address<<1;
  int dev = melexisAddress; 
  int data_low = 0; 
  int data_high = 0; 
  int pec = 0;

  //write
  i2c_start_wait(dev + I2C_WRITE);
  i2c_write(0x07);

  //read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();
  double tempFactor = 0.02;
  double tempData = 0x0000;
  int frac;
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  float temperature = tempData - 273.15;
  return temperature;
}

float temperatureCelcius(int address) 
{
  int dev = address;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  // Write
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);

  // Read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();
  double tempFactor = 0.02;
  double tempData = 0x0000;
  int frac;
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  float celcius = tempData - 273.15;
  return celcius;
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

  Thermometer getTempSensorRight(){return tempSensorRight;}
  Thermometer getTempSensorLeft(){return tempSensorLeft;}

  //ColorSensor getColorSensor() {return colorSensor;}

  float getFrontDistace() {return frontDistance;}
  void setFrontDistance(float distance) {frontDistance = distance;}

  void forwardMotors(byte);
  void backwardMotors(byte);
  void stopMotors();

  //Simple motor rotation only
  void rotateRight(byte);
  void rotateLeft(byte);

  //90 degree rotation
  void turnRight(byte);
  void turnLeft(byte);
  
  int forwardTile(byte);
  void alignForward(byte);
  void alignBackward(byte);
  void backFiveCm(byte);
  void forwardFiveCm(byte);

  void backFromBlack(byte);

  void goUpRamp(byte, bool);

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

  Thermometer tempSensorRight;
  Thermometer tempSensorLeft;

  ColorSensor colorSensor;

  int turn90amount;
  int back5amount;
  int forward30amount;
  float frontDistance;

};

Control::Control() :
motor1(4,5,19,25), motor2(7,6,18,24), motor3(9,8,3,23), motor4(10,11,2,22),
ultraSensorRight(A9), ultraSensorLeft(A8), ultraSensorFront(A3), LimitSwitchA(A13), LimitSwitchB(27), LimitSwitchC(29), 
LimitSwitchD(A12), tempSensorRight(0x1B), tempSensorLeft(0x2B), colorSensor()
{
  turn90amount = 900;
  forward30amount = 1520;
  back5amount = (forward30amount/6);
  frontDistance = 0;
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

void Control::goUpRamp(byte intensity, bool isUp)
{
  if (isUp)
  {
    forwardMotors(intensity);

    while (true)
    {
      if (true)//gyroSensor.getInclination() > 5)
      {
        break;
      }
    }

    while (true)
    {
      if (true)//gyroSensor.getInclination() < 5)
      {
        break;
      }
    }
  }
  else
  {
    while (true)
    {
      if (true)//gyroSensor.getInclination() < -5)
      {
        break;
      }
    }

    while (true)
    {
      if (true)//gyroSensor.getInclination() > -5)
      {
        break;
      }
    }
  }

  stopMotors();
}

void Control::backFiveCm(byte intensity)
{
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

  stopMotors();
}

void Control::forwardFiveCm(byte intensity)
{
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  forwardMotors(intensity);
  
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

void Control::alignForward(byte intensity)
{
  forwardMotors(intensity);

  while(true)
  {
    if(LimitSwitchA.isPushed()==LimitSwitchD.isPushed() && LimitSwitchA.isPushed() > 1000 && LimitSwitchB.isPushed() > 1000 && ultraSensorFront.getDistance()<2)
    {
      break;
    }
  }
}

void Control::alignBackward(byte intensity)
{
  backwardMotors(intensity);

  while(true)
  {
    delay(1000);
    break;
  }
}

void Control::backFromBlack(byte intensity)
{
  unsigned long targetPosM1 = motor1.getEncoderTotalTurnPos()*2;
  unsigned long targetPosM2 = motor2.getEncoderTotalTurnPos()*2;
  unsigned long targetPosM3 = motor3.getEncoderTotalTurnPos()*2;
  unsigned long targetPosM4 = motor4.getEncoderTotalTurnPos()*2;

  backwardMotors(intensity);

  while(true)
  {
    if (motor1.getEncoderTotalTurnPos() > targetPosM1 && motor2.getEncoderTotalTurnPos() > targetPosM2 &&
        motor3.getEncoderTotalTurnPos() > targetPosM3 && motor4.getEncoderTotalTurnPos() > targetPosM4)
    {
      break;
    }
  }
}

int Control::forwardTile(byte intensity)
{
  motor1.resetEncoderTotalTurnPos();
  motor2.resetEncoderTotalTurnPos();
  motor3.resetEncoderTotalTurnPos();
  motor4.resetEncoderTotalTurnPos();

  forwardMotors(intensity);

  int returnValue = 0;
  
  //Add sensor scanning in this area for more precision or obstacles
  while (true)
  {
    if (motor1.getEncoderTotalTurnPos() > forward30amount && motor2.getEncoderTotalTurnPos() > forward30amount &&
        motor3.getEncoderTotalTurnPos() > forward30amount && motor4.getEncoderTotalTurnPos() > forward30amount &&
        (int)ultraSensorFront.getDistance() < frontDistance - 30)
    {
      frontDistance -= 30;
      stopMotors();
      return colorSensor.calculateColor();
    }

    if (colorSensor.calculateColor() == 2)
    {
      backFromBlack(intensity);
      return 2;
    }

    //If there is a ramp
    if (false)//control.getGyroScope().getInclination() > 5)
    {
      backFromBlack(intensity);
      return 4;
    }

    ultraSensorRight.getDistance();
    ultraSensorLeft.getDistance();
    ultraSensorFront.getDistance();
    
    delay(5);

  }

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
  void setTemperature(float temp) {temperature = temp;}
  void setIsRamp(bool value) {isRamp = value;}

  bool getIsVisited() {return isVisited;}
  bool getIsNode() {return isNode;}
  bool getIsRobotPresent() {return isRobotPresent;}
  byte getFloorType() {return floorType;}
  byte getSearchNumber() {return searchNumber;}
  byte getTraceNumber() {return traceNumber;}
  float getTemperature() {return temperature;}
  bool getIsRamp() {return isRamp;}

  Wall wallNorth;
  Wall wallEast;
  Wall wallWest;
  Wall wallSouth;

private:
  byte x;
  byte y;

  bool isVisited, isNode, isRobotPresent;
  bool isRamp;
  byte floorType;
  byte searchNumber;
  byte traceNumber;

 float temperature;

};

Tile::Tile()
{
  isVisited = false;
  isNode = false;
  isRobotPresent = false;
  isRamp = false;

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
  static void scanForVictims();
  
  static void decideToTraceNumber();

  static void moveToNextTile();
  static void adjustToNextMove();
  static Control& getControl() {return control;}

  static byte getRobotPosX() {return robotPosX;}
  static byte getRobotPosY() {return robotPosY;}
  static byte getNodePosX() {return nodePosX;}
  static byte getNodePosY() {return nodePosY;}
  static byte getNodeAmount() {return nodeAmount;}
  static bool getNodeMode() {return nodeMode;}
  static bool getMazeComplete() {return mazeComplete;}
  static bool getActivateNode() {return activateNode;}
  static bool getDone() {return done;}
  static bool getInMainMaze() {return inMainMaze;}
  static bool getToSecondMaze() {return toSecondMaze;}
  static bool getBackToMain() {return backToMain;}

  static void findClosestNode();
  static void tracePath();

  static void backToHome();
  static void changeMaze();

  static void restartToCheckPoint();
  static void saveCheckpoint();

  static void checkNodeMode();
  static void eraseNodes();
  static Control control;

private:
  enum robotOrientation {North, South, East, West};
  enum possibleMoves {MoveForward, TurnRight, TurnLeft, DeadEnd};

  static byte robotPosX;
  static byte robotPosY;

  static robotOrientation orientation;
  static robotOrientation rampOrientation;

  static possibleMoves nextMove;
  static possibleMoves rampMove;

  static Tile tiles[7][7];
  static Tile tiles2[7][7];
  static Tile tilesCheck[7][7];

  static byte intensity;

  static bool nodeMode;
  static bool mazeComplete;
  static bool activateNode;
  static bool inMainMaze;
  static bool maze2Complete;
  static bool backToMain;
  static bool toSecondMaze;
  static bool done;
  static bool thereIsRamp;

  static byte currentSearchNumber;
  static byte nodePosX;
  static byte nodePosY;

  static byte lastCheckPointPosX;
  static byte lastCheckPointPosY;

  static byte rampPosX;
  static byte rampPosY;

  static byte maze2StartPosX;
  static byte maze2StartPosY;

  static byte nodeAmount;

public:
  static possibleMoves decideNextMove(bool, bool, bool);

};

Navigation::robotOrientation Navigation::orientation = North;
Navigation::possibleMoves Navigation::nextMove = MoveForward;

Navigation::robotOrientation Navigation::rampOrientation = North;

byte Navigation::intensity = 255; 

byte Navigation::currentSearchNumber = 1;
byte Navigation::nodePosX = 0;
byte Navigation::nodePosY = 0;
byte Navigation::nodeAmount = 0;

byte Navigation::lastCheckPointPosX = 0;
byte Navigation::lastCheckPointPosY = 0;

byte Navigation::rampPosX = 0;
byte Navigation::rampPosY = 0;

byte Navigation::maze2StartPosX = 0;
byte Navigation::maze2StartPosY = 0;

byte Navigation::robotPosX = 0;
byte Navigation::robotPosY = 0;
Tile Navigation::tiles[7][7];
Tile Navigation::tiles2[7][7];
Tile Navigation::tilesCheck[7][7];
Control Navigation::control;

bool Navigation::nodeMode = false;
bool Navigation::mazeComplete = false;
bool Navigation::activateNode = true;
bool Navigation::inMainMaze = true;
bool Navigation::maze2Complete = false;
bool Navigation::backToMain = false;
bool Navigation::toSecondMaze = false;
bool Navigation::done = false;
bool Navigation::thereIsRamp = false;

void Navigation::start(byte matSize)
{
  for (byte i = 0; i < mazeSizeY; ++i)
  {
    for (byte j = 0; j < mazeSizeX; ++j)
    {
      if (j==0)
      {
        tiles[i][j].wallWest.setWallExists(true);
      }

      if (j == mazeSizeX-1)
      {
        tiles[i][j].wallEast.setWallExists(true);
      }

      if (i==0)
      {
        tiles[i][j].wallNorth.setWallExists(true);
      }

      if (i== mazeSizeY-1)
      {
        tiles[i][j].wallSouth.setWallExists(true);
      }
    }
  }

  robotPosX = robotStartPosX;
  robotPosY = robotStartPosY;

  tiles[robotPosY][robotPosX].setIsRobotPresent(true);
  tiles[robotPosY][robotPosX].setIsVisited(true);
}

void Navigation::restartToCheckPoint()
{
  robotPosX = lastCheckPointPosX;
  robotPosY = lastCheckPointPosY;

  for (int i = 0; i < mazeSizeY; ++i)
  {
    for (int j = 0; j < mazeSizeX; ++j)
    {
      tiles[i][j] = tilesCheck[i][j];
    }
  }
  orientation = North;
  
  String message;
  
  while (true)
  {
    message = myNextion.listen();

    if (message == "65 0 15 1 ffff ffff ffff")
    {
      break;
    }
  }
}

void Navigation::saveCheckpoint()
{
  lastCheckPointPosX = robotPosX;
  lastCheckPointPosY = robotPosY;

  for (int i = 0; i < mazeSizeY; ++i)
  {
    for (int j = 0; j < mazeSizeX; ++j)
    {
      tilesCheck[i][j] = tiles[i][j];
    }
  }
}

void Navigation::backToHome()
{
  nodeMode = true;
  tiles[robotStartPosY][robotStartPosX].setIsNode(true);
}

void Navigation::checkNodeMode()
{
  if (nodePosY == robotPosY)
  {
    if (nodePosX == robotPosX)
    {
      nodeMode = false;
      if (toSecondMaze)
      {
        changeMaze();
        inMainMaze = false;
        toSecondMaze = false;
      }
      else
      {
        if (backToMain)
        {
          changeMaze();
          inMainMaze = true;
          backToMain = false;
        }
      }
    }
  }
}

void Navigation::eraseNodes()
{
  bool northBlocked = true;
  bool westBlocked = true;
  bool eastBlocked = true;
  bool southBlocked = true;

  for (byte i = 0; i < mazeSizeY; ++i)
  {
    for (byte j = 0; j < mazeSizeX; ++j)
    {
      if (tiles[i][j].getIsNode())
      {

        northBlocked = tiles[i][j].wallNorth.getWallExists() || tiles[i-1][j].getIsVisited();
        westBlocked = tiles[i][j].wallWest.getWallExists() || tiles[i][j-1].getIsVisited();
        eastBlocked = tiles[i][j].wallEast.getWallExists() || tiles[i][j+1].getIsVisited();
        southBlocked = tiles[i][j].wallSouth.getWallExists() || tiles[i+1][j].getIsVisited();

        if (northBlocked && westBlocked && eastBlocked && southBlocked)
        {
          tiles[i][j].setIsNode(false);
          --nodeAmount;  
        }
      }
    }
  }  
}

void Navigation::findClosestNode()
{
  bool nodeFound = false;
  bool finished = false;
  bool activateNode = false;

  mazeComplete = false;
  maze2Complete = false;
  
  if (inMainMaze)
  { 
    mazeComplete = true;

    for (byte i = 0; i < mazeSizeY; ++i)
    {
      for (byte j = 0; j < mazeSizeX; ++j)
      {
        tiles[i][j].setSearchNumber(103);

        if (tiles[i][j].getIsNode())
        {
          mazeComplete = false;
        }
      }
    }

    if (mazeComplete)
    {

      if (thereIsRamp)
      {
         tiles[rampPosY][rampPosX].setIsNode(true);
         toSecondMaze = true;
         backToMain = false;
      }
      else
      {
        tiles[robotStartPosY][robotStartPosX].setIsNode(true);
      }
     
    }
  }
  else
  {
    maze2Complete = true;

    for (byte i = 0; i < mazeSizeY; ++i)
    {
      for (byte j = 0; j < mazeSizeX; ++j)
      {
        tiles[i][j].setSearchNumber(103);

        if (tiles[i][j].getIsNode())
        {
          maze2Complete = false;
        }
      }
    }

    if (maze2Complete)
    {
      backToMain = true;
      toSecondMaze = false;
      tiles[maze2StartPosY][maze2StartPosX].setIsNode(true);
    }
  }
  

  tiles[robotPosY][robotPosX].setSearchNumber(1);

  currentSearchNumber = 1;


  while(true)
  {
    for (int i = 0; i < mazeSizeY; ++i)
    {
      for (int j = 0; j < mazeSizeX; ++j)
      {
        if (tiles[i][j].getSearchNumber() == currentSearchNumber)
        {
          //Check north available
          if (!tiles[i][j].wallNorth.getWallExists())
          {
            if (tiles[i-1][j].getSearchNumber() > currentSearchNumber+1)
            {
              Serial.print("Set number ");
              Serial.print(currentSearchNumber+1);
              Serial.print(" at (");
              Serial.print(i-1);
              Serial.print(",");
              Serial.print(j);
              Serial.println(")");

              //KEBEEERRRRR
              tiles[i-1][j].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i-1][j].getIsNode();
            }
            if (nodeFound)
            {
              Serial.println("Node found!");

              nodePosX = j;
              nodePosY = i-1;
              Serial.print(" at (");
              Serial.print(nodePosX);
              Serial.print(",");
              Serial.print(nodePosY);
              Serial.println(")");
              currentSearchNumber++;
              return;
            }
          }

          //Check East available
          if (!tiles[i][j].wallEast.getWallExists())
          {
            if (tiles[i][j+1].getSearchNumber() > currentSearchNumber+1)
            {
              Serial.print("Set number ");
              Serial.print(currentSearchNumber+1);
              Serial.print(" at (");
              Serial.print(i);
              Serial.print(",");
              Serial.print(j+1);
              Serial.println(")");
              tiles[i][j+1].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i][j+1].getIsNode();
            }

            if (nodeFound)
            {
              Serial.println("Node found!");
              nodePosX = j+1;
              nodePosY = i;
              Serial.print(" at (");
              Serial.print(nodePosX);
              Serial.print(",");
              Serial.print(nodePosY);
              Serial.println(")");
              currentSearchNumber++;
              return;
            }
          }

          //Check West available
          if (!tiles[i][j].wallWest.getWallExists())
          {
            if (tiles[i][j-1].getSearchNumber() > currentSearchNumber+1)
            {
              Serial.print("Set number ");
              Serial.print(currentSearchNumber+1);
              Serial.print(" at (");
              Serial.print(j-1);
              Serial.print(",");
              Serial.print(i);
              Serial.println(")");
              tiles[i][j-1].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i][j-1].getIsNode();
            }

            if (nodeFound)
            {
              Serial.println("Node found!");
              nodePosX = j-1;
              nodePosY = i;
              Serial.print(" at (");
              Serial.print(nodePosX);
              Serial.print(",");
              Serial.print(nodePosY);
              Serial.println(")");
              currentSearchNumber++;

              //Y EL RETURN???
              return;
            }
          }

          //Check South available
          if (!tiles[i][j].wallSouth.getWallExists())
          {
            if (tiles[i+1][j].getSearchNumber() > currentSearchNumber+1)
            {
              Serial.print("Set number ");
              Serial.print(currentSearchNumber+1);
              Serial.print(" at (");
              Serial.print(i+1);
              Serial.print(",");
              Serial.print(j);
              Serial.println(")");
              tiles[i+1][j].setSearchNumber(currentSearchNumber+1);
              nodeFound = tiles[i+1][j].getIsNode();
            }
            
            if (nodeFound)
            {
              Serial.println("Node found!");
              nodePosX = j;
              nodePosY = i+1;
              Serial.print(" at (");
              Serial.print(nodePosX);
              Serial.print(",");
              Serial.print(nodePosY);
              Serial.println(")");
              currentSearchNumber++;
              return;
            }
          }
        }
      }
    }

    currentSearchNumber++;
  }

  Serial.println("Out of found loop");
  
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

  byte nextNumber = currentSearchNumber;
  nextNumber--;

  tiles[tracePosY][tracePosX].setTraceNumber(1);

  Serial.print("Trace set ");
        Serial.print(currentSearchNumber-nextNumber);
        Serial.print(" at (");
              Serial.print(tracePosX);
              Serial.print(",");
              Serial.print(tracePosY);
              Serial.println(")");

  do 
  {

     //Check North
    if (!tiles[tracePosY][tracePosX].wallNorth.getWallExists())
    {
      if (tiles[tracePosY-1][tracePosX].getSearchNumber() == nextNumber)
      {
        tiles[tracePosY-1][tracePosX].setTraceNumber(currentSearchNumber-nextNumber+1);

        Serial.print("Trace set ");
        Serial.print(currentSearchNumber-nextNumber+1);
        tracePosY = tracePosY-1;
        tracePosX = tracePosX;
        Serial.print(" at (");
              Serial.print(tracePosX);
              Serial.print(",");
              Serial.print(tracePosY);
              Serial.println(")");
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

        Serial.print("Trace set ");
        tracePosY = tracePosY;
        tracePosX = tracePosX+1;
        Serial.print(" at (");
              Serial.print(tracePosX);
              Serial.print(",");
              Serial.print(tracePosY);
              Serial.println(")");
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

        Serial.print("Trace set ");
        tracePosY = tracePosY;
        tracePosX = tracePosX-1;
        Serial.print(" at (");
              Serial.print(tracePosX);
              Serial.print(",");
              Serial.print(tracePosY);
              Serial.println(")");
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

        Serial.print("Trace set ");
        tracePosY = tracePosY+1;
        tracePosX = tracePosX;
        Serial.print(" at (");
              Serial.print(tracePosX);
              Serial.print(",");
              Serial.print(tracePosY);
              Serial.println(")");
        nextNumber--;
        continue;
      }
    }

  }
  while (nextNumber > 0);

  //(tracePosY != robotPosY && tracePosX != robotPosX)||(tracePosY==robotPosY||tracePosX==robotPosX)

  tiles[robotPosY][robotPosY].setTraceNumber(currentSearchNumber);
  Serial.println("What the helllll");
  
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
      ++nodeAmount;
      Serial2.print("t0.txt=");
      Serial2.print("\""); 
      Serial2.print("NODE CRE"); 
      Serial2.print("\"");  
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);

    }
    else
    {
      if(leftAvailable)
      {
        //There are several possible moves
        tiles[robotPosY][robotPosX].setIsNode(true);
        ++nodeAmount;
        Serial2.print("t0.txt=");
        Serial2.print("\""); 
        Serial2.print("NODE CRE"); 
        Serial2.print("\"");  
        Serial2.write(0xff); 
        Serial2.write(0xff);
        Serial2.write(0xff);

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
        ++nodeAmount;
        Serial2.print("t0.txt=");
        Serial2.print("\""); 
        Serial2.print("NODE CRE"); 
        Serial2.print("\"");  
        Serial2.write(0xff); 
        Serial2.write(0xff);
        Serial2.write(0xff);

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
        Serial.println("DEAD END");
        nodeMode = true;
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

  activateNode = true;

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

void Navigation::scanForVictims()
{
  //float tempRight = control.getTempSensorRight().getTemperature();
  //float tempLeft = control.getTempSensorLeft().getTemperature();

  
}

void Navigation::decideToTraceNumber()
{
  //CheckFront
  byte currentTraceNumber = tiles[robotPosY][robotPosX].getTraceNumber();

  switch (orientation)
  {
    case North:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallNorth.getWallExists())
      {
        Serial.println("Move forward??");
        nextMove = MoveForward;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallEast.getWallExists())
      {
        nextMove = TurnRight;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallWest.getWallExists())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallSouth.getWallExists())
      {
        nextMove = DeadEnd;
        return;
      }

      break;

    case East:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallNorth.getWallExists())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallEast.getWallExists())
      {
        nextMove = MoveForward;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallWest.getWallExists())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallSouth.getWallExists())
      {
        nextMove = TurnRight;
        return;
      }

      break;
    case West:
      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallNorth.getWallExists())
      {
        nextMove = TurnRight;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallEast.getWallExists())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallWest.getWallExists())
      {
        nextMove = MoveForward;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallSouth.getWallExists())
      {
        nextMove = TurnLeft;
        return;
      }
      break;

    case South:

      //Move North
      if (currentTraceNumber-1 == tiles[robotPosY-1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallNorth.getWallExists())
      {
        nextMove = DeadEnd;
        return;
      }

      //Move East
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX+1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallEast.getWallExists())
      {
        nextMove = TurnLeft;
        return;
      }

      //Move West
      if (currentTraceNumber-1 == tiles[robotPosY][robotPosX-1].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallWest.getWallExists())
      {
        nextMove = TurnRight;
        return;
      }

      //Move South
      if (currentTraceNumber-1 == tiles[robotPosY+1][robotPosX].getTraceNumber() &&
          !tiles[robotPosY][robotPosX].wallSouth.getWallExists())
      {
        nextMove = MoveForward;
        return;
      }

      break;
  }
}

void Navigation::adjustToNextMove()
{
  bool adjustToLeft = control.getUltraSensorLeft().getDistance() < 20;
  bool adjustToRight = control.getUltraSensorRight().getDistance() < 20;
  bool adjustToFront = control.getUltraSensorFront().getDistance() < 20;
  
  switch(nextMove)
  {
    case MoveForward:

      break;

    case TurnRight:
      
      if(adjustToFront)
      {
        control.alignForward(intensity);
        control.backFiveCm(intensity);
      }

      delay(300);
      control.turnRight(intensity);

      if(adjustToLeft)
      {
        control.alignBackward(intensity);
        control.forwardFiveCm(intensity);
      }

      delay(50);
      control.setFrontDistance(control.getUltraSensorFront().getDistance());

      switch(orientation)
      {
        case North:
          orientation = East;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case West:
          orientation = North;
        /*  Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("NORTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);
          */
          break;

        case East:
          orientation = South;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          
          break;

        case South:
          orientation = West;
          /*Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;
      }

      break;

    case TurnLeft:
      
      if(adjustToFront)
      {
        control.alignForward(intensity);
        control.backFiveCm(intensity);
      }
      
      delay(300);
      control.turnLeft(intensity);

      if(adjustToRight)
      {
        control.alignBackward(intensity);
        control.forwardFiveCm(intensity);
      }
      delay(50);      
      control.setFrontDistance(control.getUltraSensorFront().getDistance());

      switch(orientation)
      {
        case North:
          orientation = West;
          /*Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case West:
         orientation = South;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case East:
          orientation = North;
          break;

        case South:
          orientation = East;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;
      }
      break;

    case DeadEnd:
    
      if(adjustToFront)
      {
        control.alignForward(intensity);
        control.backFiveCm(intensity);
      }
      
      delay(300);
      control.turnRight(intensity);

      if(adjustToRight)
      {
        control.alignForward(intensity);
        control.backFiveCm(intensity);
      }
      else
      {
        if(adjustToLeft)
        {
          control.alignBackward(intensity);
          control.forwardFiveCm(intensity);
        }
      }

      delay(300);
      control.turnRight(intensity);
      
      delay(300);

      control.alignBackward(intensity);
      control.forwardFiveCm(intensity);

      delay(50);      
      control.setFrontDistance(control.getUltraSensorFront().getDistance());

      switch(orientation)
      {
        case North:
          orientation = South;
        /*  Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("SOUTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case West:
          orientation = East;
          /*Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("EAST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case East:
          orientation = West;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("WEST"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;

        case South:
          orientation = North;
         /* Serial2.print("t0.txt=");
          Serial2.print("\""); 
          Serial2.print("NORTH"); 
          Serial2.print("\"");  
          Serial2.write(0xff); 
          Serial2.write(0xff);
          Serial2.write(0xff);*/
          break;
      }

      break;
  }
}

void Navigation::changeMaze()
{

  //Change to second maze
  if (inMainMaze)
  {
    //Copy main maze to tiles2
    for (int i = 0; i < mazeSizeY; ++i)
    {
      for (int j = 0; j < mazeSizeX; ++j)
      {
        tiles2[i][j] = tiles[i][j];
      }
    }

    Tile tile;
    //Erase tiles for new usage
    for (int i = 0; i < mazeSizeY; ++i)
    {
      for (int j = 0; j < mazeSizeX; ++j)
      {
        tiles[i][j] = tile;
      }
    }

    //Adjust to orientation of the ramp
    switch (orientation)
    {
      case North:
        switch (rampOrientation)
        {
          case North:
            //No change
            break;

          case East:
            nextMove = TurnRight;
            break;

          case West:
            nextMove = TurnLeft;
            break;

          case South:
            //Impossible
            nextMove = DeadEnd;
            break;
        }

        break;

      case East:
        switch (rampOrientation)
        {
          case North:
            nextMove = TurnLeft;
            break;

          case East:
            //No change
            break;

          case West:
            //Impossible
            nextMove = DeadEnd;
            break;

          case South:
            nextMove = TurnRight;
            break;
        }
        
        break;

      case West:
        switch (rampOrientation)
        {
          case North:
            nextMove = TurnRight;
            break;

          case East:
            //Impossible
            nextMove = DeadEnd;
            break;

          case West:
            //No change
            break;

          case South:
            nextMove = TurnLeft;
            break;
        }
        
        break;

      case South:
        switch (rampOrientation)
        {
          case North:
            //Impossible
          nextMove = DeadEnd;
            break;

          case East:
            nextMove = TurnLeft;
            break;

          case West:
            nextMove = TurnRight;
            break;

          case South:
            //No change
            break;
        }
        
        break;
    }

    adjustToNextMove();

    control.goUpRamp(intensity, true);

    robotPosY = maze2StartPosY;
    robotPosX = maze2StartPosX;

    orientation = North;

    tiles[robotPosY][robotPosX].setIsVisited(true);

    inMainMaze = false;
  }
  //Change to first maze
  else
  {
    //Retrieve main maze from tiles2
    for (int i = 0; i < mazeSizeY; ++i)
    {
      for (int j = 0; j < mazeSizeX; ++j)
      {
        tiles[i][j] = tiles2[i][j];
      }
    }

    switch (orientation)
    {
      case North:
        //Impossible
        nextMove = DeadEnd;
        break;

      case East:
        nextMove = TurnRight;
        break;

      case West:
        nextMove = TurnLeft;
        break;

      case South:
        //No change
        break;
    }

    adjustToNextMove();

    control.goUpRamp(intensity, false);

    robotPosY = rampPosY;
    robotPosX = rampPosX;

    switch (rampOrientation)
    {
      case North:
        orientation = South;
        break;

      case East:
        orientation = West;
        break;

      case West:
        orientation = East;
        break;

      case South:
        orientation = North;
        break;
    }
    inMainMaze = true;
    backToHome();
  }
}

void Navigation::moveToNextTile()
{ 
  //If value is 2, there is black tile
  //If value is 3, there is a checkpoint
  //If value is 4, there is a ramp
  //If value is 1, it is a normal tile
  int tileValue = control.forwardTile(intensity);

  switch(orientation)
  {
    case North:

      if (tileValue == 2)
      {
        tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);

        if (robotPosY > 0 && robotPosX > 0)
          tiles[robotPosY-1][robotPosX-1].wallEast.setWallExists(true);

        if (robotPosY > 0 && robotPosX < mazeSizeX-1)
           tiles[robotPosY-1][robotPosX+1].wallWest.setWallExists(true);

        if (robotPosY > 1)
          tiles[robotPosY-2][robotPosX].wallSouth.setWallExists(true);
      }
      else
      {
        if (tileValue == 4)
        {
          rampPosX = robotPosX;
          rampPosY = robotPosY;
          tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);
          rampOrientation = North;
          thereIsRamp = true;
        }
        else
        {
          robotPosY--;
        }
        
      }
      
      break;

    case West:

      if (tileValue == 2)
      {
        tiles[robotPosY][robotPosX].wallWest.setWallExists(true);

        if (robotPosY > 0 && robotPosX > 0)
          tiles[robotPosY-1][robotPosX-1].wallSouth.setWallExists(true);

        if (robotPosY < mazeSizeY-1 && robotPosX > 0)
          tiles[robotPosY+1][robotPosX-1].wallNorth.setWallExists(true);

        if (robotPosX > 1)
          tiles[robotPosY][robotPosX-2].wallEast.setWallExists(true);
      }
      else
      {
        if (tileValue == 4)
        {
          rampPosX = robotPosX;
          rampPosY = robotPosY;
          tiles[robotPosY][robotPosX].wallWest.setWallExists(true);
          rampOrientation = West;
          thereIsRamp = true;
        }
        else
        {
          robotPosX--;
        }
      }

      break;

    case East:

      if (tileValue == 2)
      {
        tiles[robotPosY][robotPosX].wallEast.setWallExists(true);

        if (robotPosY > 0 && robotPosX < mazeSizeX-1)
          tiles[robotPosY-1][robotPosX+1].wallSouth.setWallExists(true);

        if (robotPosY < mazeSizeY-1 && robotPosX < mazeSizeX-1)
          tiles[robotPosY+1][robotPosX+1].wallNorth.setWallExists(true);

        if (robotPosX < mazeSizeX-2)
          tiles[robotPosY][robotPosX+2].wallEast.setWallExists(true);
      }
      else
      {
        if (tileValue == 4)
        {
          rampPosX = robotPosX;
          rampPosY = robotPosY;
          tiles[robotPosY][robotPosX].wallEast.setWallExists(true);
          rampOrientation = East;
          thereIsRamp = true;
        }
        else
        {
          robotPosX++;
        }
      }

      break;

    case South:

      if (tileValue == 2)
      {
        tiles[robotPosY][robotPosX].wallSouth.setWallExists(true);

        if (robotPosY < mazeSizeY-1 && robotPosX > 0)
          tiles[robotPosY+1][robotPosX-1].wallEast.setWallExists(true);

        if (robotPosY < mazeSizeY-1 && robotPosX < mazeSizeX-1)
           tiles[robotPosY+1][robotPosX+1].wallWest.setWallExists(true);

        if (robotPosY < mazeSizeY-2)
          tiles[robotPosY+2][robotPosX].wallSouth.setWallExists(true);
      }
      else
      {
        if (tileValue == 4)
        {
          rampPosX = robotPosX;
          rampPosY = robotPosY;
          tiles[robotPosY][robotPosX].wallNorth.setWallExists(true);
          rampOrientation = South;
          thereIsRamp = true;
        }
        else
        {
          robotPosY++;
        }
      }

      break;
  }

  //If it is a checkpoint
  if (tileValue == 3)
  {
    saveCheckpoint();
  }

  //If it is a ramp
  
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

void checkCheckpointButton()
{
  String message = myNextion.listen();
  if(message != "")
  {
    if(message == "65 0 15 1 ffff ffff ffff")
    {
      Navigation::restartToCheckPoint();
    }
  }
}

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

void setup() 
{
  enableInterrupt(Navigation::getControl().getMotor1().getEncoderA(), encodeInterruptM1, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor1().getEncoderB(), encodeInterruptM1, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor2().getEncoderA(), encodeInterruptM2, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor2().getEncoderB(), encodeInterruptM2, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor3().getEncoderA(), encodeInterruptM3, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor3().getEncoderB(), encodeInterruptM3, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor4().getEncoderA(), encodeInterruptM4, CHANGE);
  enableInterrupt(Navigation::getControl().getMotor4().getEncoderB(), encodeInterruptM4, CHANGE);

  Navigation::start(5);

 // i2c_init();

  Serial.begin(9600);
  Serial2.begin(9600);

  myNextion.init();
  
  Navigation::control.setFrontDistance(Navigation::control.getUltraSensorFront().getDistance());

  Serial.print("Done");

   delay(1000);
}

void loop()
{
  checkCheckpointButton();
  if (!Navigation::getNodeMode())
  {
    //MAPPING NAVIGATION

    //CHECK IF DONE
    if (Navigation::getDone())
    {
      delay(10000);
    }

    //USE ULTRASONIC TO GET WALLS
    Navigation::scanSides();
    checkCheckpointButton();

    Serial2.print("n0.val=");
    Serial2.print(7);  
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);

    //Navigation::scanForVictims();
    
    //IF NO POSSIBLE MOVE, GO NODE MODE
    if (Navigation::getNodeMode())
    {
      return;
    }
    
    delay(100);
    checkCheckpointButton();

    //ROTATE ROBOT TO MOVE
    Navigation::adjustToNextMove();
    checkCheckpointButton();

    delay(100);  

    //Navigation::scanForVictims();
    checkCheckpointButton();
    delay(50);

    //MOVE FORWARD TO NEXT TILE
    Navigation::moveToNextTile();
    checkCheckpointButton();
    delay(100);
    
  }
  else
  {
    //NODE MODE

    //CHECK IF FIRST TIME IN NODE
    if (Navigation::getActivateNode())
    {
      checkCheckpointButton();
      Navigation::findClosestNode();
      checkCheckpointButton();
      Navigation::tracePath();
      checkCheckpointButton();
    }

    //DECIDE NEXT MOVE BASED ON NUMBERS
    Navigation::decideToTraceNumber();
    checkCheckpointButton();

    //ROTATE TO NEXT MOVE
    Navigation::adjustToNextMove();
    checkCheckpointButton();

    delay(100);

    //MOVE FORWARD TO NEXT TILE
    Navigation::moveToNextTile();
    checkCheckpointButton();

    //CHECK IF DONE WITH NODES
    Navigation::checkNodeMode();
    checkCheckpointButton();

  }

  //ERASE ALREADY EXHAUSTED NODES
  Navigation::eraseNodes();

}
