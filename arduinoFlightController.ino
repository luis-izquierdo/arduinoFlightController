#include<Wire.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050/MPU-6500
const int loopsPerSerialWrite = 80;
const int loopsPerOrthonormalization = 5 * 250; // orthonormalize orientation matrix every 5 seconds
const int microsBetweenReads = 4000;
const double accelerometerCorrectionFactor = 0.15;

int16_t accX, accY, accZ, temperature, gyroX, gyroY, gyroZ;
int gyroAvgDevX, gyroAvgDevY, gyroAvgDevZ;
// components of the orientation matrix
double noseX, noseY, noseZ, rWingX, rWingY, rWingZ, ceilingX, ceilingY, ceilingZ;

unsigned long lastGyroReadTime, previousReadingTime;
int loopsSinceLastSerialWrite;
int loopsSinceLastOrthonormalization;

// gyro units to deg/s, rad/s according to default full-scale config
#define DEG_PER_SECOND_PER_GYRO_UNIT 131.072
#define RAD_PER_SECOND_PER_GYRO_UNIT 7509.87241234

#define SCALAR_PROD(a1, a2, a3, b1, b2, b3) ((a1) * (b1) + (a2) * (b2) + (a3) * (b3))
#define COMPUTE_SIN_COS(ANG, SINVAR, COSVAR) { SINVAR = sin(ANG); COSVAR = cos(ANG); }
// alternative way of computing sin/cos in case we want to replace some calls to trigonometric 
// functions by something else for performance improvements
//#define COMPUTE_SIN_COS(ANG, SINVAR, COSVAR) { SINVAR = sin(ANG); COSVAR = sqrt(1 - SINVAR * SINVAR);}
//   if (ANG > M_PI / 2 || ANG < - M_PI / 2) COSVAR *= -1; 

void setup() {
  initializeMPU();
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  // turn off the LED during gyro calibration
  determineGyroDeviationPerAxis();
  digitalWrite(13, HIGH); // turn on the LED when calibration is over

  Serial.begin(9600);
  lastGyroReadTime = micros();
  loopsSinceLastSerialWrite = 0;
  loopsSinceLastOrthonormalization = 0;

  // initial state of the orientation matrix (nose, rWing, ceiling vectors)
  noseX = 0;
  noseY = 1;
  noseZ = 0;
  rWingX = 1;
  rWingY = 0;
  rWingZ = 0;
  ceilingX = 0;
  ceilingY = 0;
  ceilingZ = 1;
}

void loop() {
  // wait until it's time to read the gyro again
  previousReadingTime = lastGyroReadTime;
  while (micros() < previousReadingTime + microsBetweenReads);
  lastGyroReadTime = micros();
  readGyroAccel();
  
  updateOrientationAccordingToGyroInfo();

  correctOrientationAccordingToAccelInfo(accelerometerCorrectionFactor);

  loopsSinceLastOrthonormalization++;
  if (loopsSinceLastOrthonormalization == loopsPerOrthonormalization)
  {
    loopsSinceLastOrthonormalization = 0;
    orthonormalizeOrientationMatrix();    
  }
  
  loopsSinceLastSerialWrite++;
  if (loopsSinceLastSerialWrite == loopsPerSerialWrite)
  {
    loopsSinceLastSerialWrite = 0;
    Serial.print(noseX); Serial.print(" , "); Serial.print(noseY); Serial.print(" , "); Serial.println(noseZ);
    //Serial.print(rWingX); Serial.print(" , "); Serial.print(rWingY); Serial.print(" , "); Serial.println(rWingZ);
  }
}

void initializeMPU()
{
  Wire.begin();
  // not calling Wirer.setClock() was causing the Wire.requestFrom(MPU_addr, 14, true) operation 
  // to take ~1400us "some times", whereas with this setting it takes always ~450
  Wire.setClock(400000L); 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // activate low-pass filter
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);   // CONFIG register (1A hex)
  Wire.write(0x03);   // Set the register bits to 00000011 (Digital Low Pass Filter -> ~43Hz)
  Wire.endTransmission(true);
}

// compute gyro deviation assuming the gyro is at rest, and store it in global 
// variables gyroAvgDevX, gyroAvgDevY, gyroAvgDevZ
void determineGyroDeviationPerAxis()
{
  int samples = 2000;
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  for (int i = 0; i < samples; i++)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);  // request a total of 6 registers
    sumGyroX += Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    sumGyroY += Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    sumGyroZ += Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }
  gyroAvgDevX = (int)(sumGyroX / samples);
  gyroAvgDevY = (int)(sumGyroY / samples);
  gyroAvgDevZ = (int)(sumGyroZ / samples);
}

// read current accel/gyro data from MPU sensor (takes around 290us)
void readGyroAccel()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // correct gyro reading with average deviation sensed while the gyro was at rest
  gyroX -= gyroAvgDevX;
  gyroY -= gyroAvgDevY;
  gyroZ -= gyroAvgDevZ;  

  // make the Y gyro reading sign match my drawings (tip of right wing going up from 
  // horizontal => positive angular velocity in Y axis)
  gyroY = - gyroY;  
}

// Given the orientation matrix computed some microseconds ago and the current angular
// velocities read from the gyroscope, estimate how much the aircraft must have rotated
// with respect to each axis and update the orientation matrix accordingly.
void updateOrientationAccordingToGyroInfo()
{
  int microsSinceLastRead = (int) (lastGyroReadTime - previousReadingTime);
  // TODO: for greater precision, consider averaging the current gyro values with those from the previous reading
  double gyroDeltaPitch = gyroX / RAD_PER_SECOND_PER_GYRO_UNIT * microsSinceLastRead / 1000000;
  double gyroDeltaRoll =  gyroY / RAD_PER_SECOND_PER_GYRO_UNIT * microsSinceLastRead / 1000000;
  double gyroDeltaYaw = gyroZ / RAD_PER_SECOND_PER_GYRO_UNIT * microsSinceLastRead / 1000000;

  double sinGDP, cosGDP, sinGDR, cosGDR, sinGDY, cosGDY;
  COMPUTE_SIN_COS(gyroDeltaPitch, sinGDP, cosGDP);
  COMPUTE_SIN_COS(gyroDeltaRoll, sinGDR, cosGDR);
  COMPUTE_SIN_COS(gyroDeltaYaw, sinGDY, cosGDY);

  // shorter-name variables so that formulas are "less huge"
  double e = sinGDP, f = cosGDP,
    g = sinGDR, h = cosGDR,
    i = sinGDY, j = cosGDY;

  // matrix that represents the 3 rotations (roll, then pitch, then yaw) associated with 
  // gyroDeltaPitch, gyroDeltaRoll and gyroDeltaYaw. rotation[row][column]. Each column 
  // corresponds to nose/rWing/vectors if the aircraft system started at default orientation 
  // and was just transformed according to "rotation".
  double rotation[3][3];
  rotation[0][0] = j*h + e*g*i;
  rotation[1][0] = h*i - e*g*j;
  rotation[2][0] = f*g;
  rotation[0][1] = -f*i;
  rotation[1][1] = f*j;
  rotation[2][1] = e;
  rotation[0][2] = -g*j + e*h*i;
  rotation[1][2] = -g*i - e*h*j;
  rotation[2][2] = f*h;

  // pre-multiply rotation by the current (nose | rWing | ceiling) matrix to obtain the updated orientation matrix
  double newNoseX, newNoseY, newNoseZ, newRWingX, newRWingY, newRWingZ, newCeilingX, newCeilingY, newCeilingZ;
  newRWingX = SCALAR_PROD(rWingX, noseX, ceilingX, rotation[0][0], rotation[1][0], rotation[2][0]);
  newRWingY = SCALAR_PROD(rWingY, noseY, ceilingY, rotation[0][0], rotation[1][0], rotation[2][0]);
  newRWingZ = SCALAR_PROD(rWingZ, noseZ, ceilingZ, rotation[0][0], rotation[1][0], rotation[2][0]);
  newNoseX = SCALAR_PROD(rWingX, noseX, ceilingX, rotation[0][1], rotation[1][1], rotation[2][1]);
  newNoseY = SCALAR_PROD(rWingY, noseY, ceilingY, rotation[0][1], rotation[1][1], rotation[2][1]);
  newNoseZ = SCALAR_PROD(rWingZ, noseZ, ceilingZ, rotation[0][1], rotation[1][1], rotation[2][1]);
  newCeilingX = SCALAR_PROD(rWingX, noseX, ceilingX, rotation[0][2], rotation[1][2], rotation[2][2]);
  newCeilingY = SCALAR_PROD(rWingY, noseY, ceilingY, rotation[0][2], rotation[1][2], rotation[2][2]);
  newCeilingZ = SCALAR_PROD(rWingZ, noseZ, ceilingZ, rotation[0][2], rotation[1][2], rotation[2][2]);

  // update orientation matrix
  noseX = newNoseX;
  noseY = newNoseY;
  noseZ = newNoseZ;
  rWingX = newRWingX;
  rWingY = newRWingY;
  rWingZ = newRWingZ;
  ceilingX = newCeilingX;  
  ceilingY = newCeilingY;  
  ceilingZ = newCeilingZ;

  /*
   Matrix operations involved in updating the orientation matrix according to the angular velocities
   obtained from the gyroscope and the time ellapsed since the last gyroscope reading.
  Mdy_dp_dr =
  {{cosdy, -sindy, 0},{sindy, cosdy, 0},{0, 0, 1}} . ({{1, 0, 0},{0, cosdp, -sindp}, {0, sindp, cosdp}}.{{cosdr,0,-sindr},{0,1,0},{sindr,0,cosdr}})
  
  e = sindp
  f = cosdp
  g = sindr
  h = cosdr
  i = sindy
  j = cosdy
  
  Mdy_dp_dr =
  |j  -i 0 |     ( |1  0  0  |      | h  0  -g| )
  |i  j  0 |  .  ( |0  f  -e |   .  | 0  1  0 | )
  |0  0  1 |     ( |0  e  f  |      | g  0  h | )
  */    
}

void correctOrientationAccordingToAccelInfo(double accelerometerCorrectionFactor)
{
  // TODO
}

// We perform hundreds of corrections per second to the orientation matrix, each
// correction involving multiple matrix multiplications. The theoretical result
// of all these operations is always an orthogonal matrix (its columns are
// orthogonal unit vectors), but in practice, numerical errors make the matrix
// progressively lose this condition, so we need to restablish it from time
// to time.
void orthonormalizeOrientationMatrix()
{
    // normalize col1
    double col1Length = sqrt(rWingX * rWingX + rWingY * rWingY + rWingZ * rWingZ);
    rWingX /= col1Length;
    rWingY /= col1Length;
    rWingZ /= col1Length;
    // adjust second column to make it orthogonal to first column
    double compCol2InDirCol1 = SCALAR_PROD(rWingX, rWingY, rWingZ, noseX, noseY, noseZ); // since length(col1) is 1
    // col2 -= component of col2 in the direction of col1
    noseX -= compCol2InDirCol1 * rWingX;
    noseY -= compCol2InDirCol1 * rWingY;
    noseZ -= compCol2InDirCol1 * rWingZ;
    // normalize col2
    double col2Length = sqrt(noseX * noseX + noseY * noseY + noseZ * noseZ);
    noseX /= col2Length;
    noseY /= col2Length;
    noseZ /= col2Length;
    double compCol3InDirCol1 = SCALAR_PROD(rWingX, rWingY, rWingZ, ceilingX, ceilingY, ceilingZ); // since length(col1) is 1
    double compCol3InDirCol2 = SCALAR_PROD(noseX, noseY, noseZ, ceilingX, ceilingY, ceilingZ); // since length(col2) is 1
    // col3 -= component of col3 in the direction of col1
    ceilingX -= compCol3InDirCol1 * rWingX;
    ceilingY -= compCol3InDirCol1 * rWingY;
    ceilingZ -= compCol3InDirCol1 * rWingZ;
    // col3 -= component of col3 in the direction of col2
    ceilingX -= compCol3InDirCol2 * noseX;
    ceilingY -= compCol3InDirCol2 * noseY;
    ceilingZ -= compCol3InDirCol2 * noseZ;
    // normalize col3
    double col3Length = sqrt(ceilingX * ceilingX + ceilingY * ceilingY + ceilingZ * ceilingZ);
    ceilingX /= col3Length;
    ceilingY /= col3Length;
    ceilingZ /= col3Length;    
}
