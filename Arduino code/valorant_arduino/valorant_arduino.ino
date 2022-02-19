//#include <SoftwareSerial.h>
#include <Wire.h>   
#include <math.h>

//IMU
#define MPU6050_I2C_ADDRESS 0x68
#define FREQ  30.0

double gyrX = 0;
double gyrY = 0;
double gyrZ = 0;

int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;

double angleFromGyro_x = 0;
double angleFromGyro_y = 0;
double angleFromGyro_z = 0;

double gyrXoffs = 0;
double gyrYoffs = 0;
double gyrZoffs = 0;

double gSensitivity = 65.5; 

int error;
double dT;
double ax, ay, az;
unsigned long start_time, end_time;

//LED
int red[3] = {255,0,0};
int green[3] = {0,255,0};
int blue[3] = {0,0,255};
int purple[3] = {255,0,255};
int sky[3] = {0,255,255};
int white[3] = {255,255,255};
int black[3] = {0,0,0};

//--------------------------------Variable definition---------------------------------//
const int sizeIn = 14;
int countLED = 0;
int anval = 0;
int dival = 0;
int button10 = 0;
int skill_1 = 1;
int skill_2 = 1;
int ult = 1;
int skill_3 = 1;

//-------------------------------------Setup------------------------------------------//
void setup() {
  int error;
  uint8_t c;
  uint8_t sample_div;
  Serial.begin(38400);
  pinMode(13, OUTPUT); 
  Wire.begin();
  
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x6b, 0x00);
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x1a, 0x01);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);
  sample_div = 1000 / FREQ - 1;
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x19, sample_div);
  
  digitalWrite(13, HIGH);
  calibrate();
  digitalWrite(13, LOW);
  
  for(int j = 3; j <= 12; j++){
    pinMode(j, INPUT_PULLUP);
  }
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A6, INPUT);
}

void loop() {  
  start_time = millis();

  read_sensor_data();

  //* angles based on accelerometer
  ax = atan2(accY, accZ) * 180 / M_PI;
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;

  angleFromGyro_x = angleFromGyro_x  + gyrX / FREQ;  
  angleFromGyro_y = angleFromGyro_y  + gyrY / FREQ;
  angleFromGyro_z = angleFromGyro_z  + gyrZ / FREQ;
  
  //exponential filter
  angleFromGyro_x = angleFromGyro_x * 0.80 + ax * 0.2;
  angleFromGyro_y = angleFromGyro_y * 0.80 + ay * 0.2;
  
  anval = analogRead(A6);
  dival = digitalRead(A0);

  //depending on the potentiometer value, when the button is pressed, only one skill will be activitated
  if(dival == 0){
    if(anval>=113 && anval<= 200 && skill_2 != 0 && ult != 0 && skill_3 != 0){
      skill_1 = 0;
    }else if(anval>200 && anval<=500 && skill_1 != 0 && ult != 0 && skill_3 != 0){
      skill_2 = 0;
    }else if(anval>500 && anval<=800 && skill_1 != 0 && ult != 0 && skill_2 != 0){
      skill_3 = 0;
    }else if(anval>800 && anval<=900 && skill_1 != 0 && skill_2 != 0 && skill_3 != 0){
      ult = 0;
    }
  }else{
    skill_1 = 1; skill_2 = 1; skill_3 = 1; ult = 1;
  }
  //LED coloring as function of the skill
  if(anval>=113 && anval<= 200){LedColor(sky);}
  else if(anval>200 && anval<=500){LedColor(red);}
  else if(anval>500 && anval<=800){LedColor(blue);}
  else if(anval>800 && anval<=900){LedColor(purple);}
 
  //printing the values in the the buffer so that the data will be collected by python
  for(int i = 3; i <=12; i++){
    Serial.print(digitalRead(i)); Serial.print(" ");
  }  
  Serial.print(skill_1); Serial.print(" ");
  Serial.print(skill_2); Serial.print(" ");
  Serial.print(skill_3); Serial.print(" ");
  Serial.print(ult);     Serial.print(" ");
  digitalWrite(13, HIGH);
  Serial.print(int(angleFromGyro_z*100)); Serial.print(" ");
  Serial.print(int(angleFromGyro_y*100)); Serial.print(" ");
  Serial.println(int(angleFromGyro_x*100));
  digitalWrite(13, LOW);   
    
  end_time = millis();

  //wait for the loop time to finish
  delay(((1/FREQ) * 1000) - (end_time - start_time));
}

//function that changes the color of the LED according to the RGB input
void LedColor(int colors[]){
  analogWrite(A1, colors[1]);
  analogWrite(A2, colors[2]);
  analogWrite(A3, colors[0]);
}

//* ----- -----
void calibrate(){
  int x;
  int num = 500;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6]; 
  uint8_t error;

  for (x = 0; x < num; x++){
    error = i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, 6);
    if(error!=0)
    return;

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

  Serial.println("Calibration result:");
  Serial.print(gyrXoffs);
  Serial.print(", ");
  Serial.print(gyrYoffs);
  Serial.print(", ");
  Serial.println(gyrZoffs);  
} 

//* ----- -----
void read_sensor_data(){
 uint8_t i2cData[14];
 uint8_t error;
 //* read imu data
 error = i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, 14);
 if(error!=0)
 return;

 //* assemble 16 bit sensor data
 accX = ((i2cData[0] << 8) | i2cData[1]);
 accY = ((i2cData[2] << 8) | i2cData[3]);
 accZ = ((i2cData[4] << 8) | i2cData[5]);

 gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
 gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
 gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
}

//* ----- I2C routines -----
int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);
  if (n != 1)
  return (-10);

  n = Wire.endTransmission(false);    //* hold the I2C-bus
  if (n != 0)
  return (n);

  //* Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while(Wire.available() && i<size){
    buffer[i++] = Wire.read();
  }
  if ( i != size)
  return (-11);

  return (0);  // return : no error
}

//* ----- -----
int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);        //* write the start address
  if (n != 1)
  return (-20);

  n = Wire.write(pData, size);  //* write data bytes
  if (n != size)
  return (-21);

  error = Wire.endTransmission(true); //* release the I2C-bus
  if (error != 0)
  return (error);

  return (0);   //* return : no error
}

//* ----- -----
int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;
  
  error = i2c_write(addr, reg, &data, 1);
  return (error);
}
