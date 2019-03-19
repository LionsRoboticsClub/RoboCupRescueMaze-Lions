//LIONS MAZE

int crashSensor[4] = {1,2,3,4};
int crashSensorValues[4];
int sharpDistanceSensor[8] = {5,6,7,8,9,10,11,12};
int sharpDistanceSensorValues[8];
int distanceRangeSensor[4] = {13,14,15,16};
int distanceRangeSensorValues[4];
int encoder[4] = {17,18,19,20};
int color[];
int gyro[];

int motor[4][2] = {
	{21,22},
	{23,24},
	{25,26},
	{27,28}
};

void forward(){};

int map[][][]={
	{{0,0,0},{0,0,0},{0,0,0},{0,0,0}},
	{{0,0,0},{0,0,0},{0,0,0},{0,0,0}},
	{{0,0,0},{0,0,0},{0,0,0},{0,0,0}},
};

void updateSensors(int crashSensor[], int sharpDistanceSensor[], 
	int distanceRangeSensor[], 
	int encoder[], int color[], int gyro[],
	int crashSensorValues[], int sharpDistanceSensorValues[], 
	int distanceRangeSensorValues[]){

}

void setup(){

}

void loop(){



}