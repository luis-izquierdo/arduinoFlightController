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
double gyroUnitToRadPerSecond; // gyro units to rad/s according to full-scale config

#define SCALAR_PROD(a1, a2, a3, b1, b2, b3) ((a1) * (b1) + (a2) * (b2) + (a3) * (b3))
#define COMPUTE_SIN_COS(ANG, SINVAR, COSVAR) { SINVAR = sin(ANG); COSVAR = cos(ANG); }
// alternative way of computing sin/cos in case we want to replace some calls to trigonometric 
// functions by something else for performance improvements
//#define COMPUTE_SIN_COS(ANG, SINVAR, COSVAR) { SINVAR = sin(ANG); COSVAR = sqrt(1 - SINVAR * SINVAR);}
//   if (ANG > M_PI / 2 || ANG < - M_PI / 2) COSVAR *= -1; 

// Variables related with Remote Control input. Some of these variables must be declared
// as volatile so that they can be safely accessed by both the Interrupt Service Routine
// and the main loop.
// - current state of pulse
bool rcChannel1PulseOn, rcChannel2PulseOn, 
              rcChannel3PulseOn, rcChannel4PulseOn, 
              rcChannel5PulseOn, rcChannel6PulseOn;
// - value of micros() when the last pulse started
unsigned long rcChannel1PulseStart, rcChannel2PulseStart, 
                        rcChannel3PulseStart, rcChannel4PulseStart, 
                        rcChannel5PulseStart, rcChannel6PulseStart;
// - duration of the last finished pulse (regardless of a possible ongoing pulse)
volatile unsigned long rcChannel1PulseWidth, rcChannel2PulseWidth, 
                        rcChannel3PulseWidth, rcChannel4PulseWidth, 
                        rcChannel5PulseWidth, rcChannel6PulseWidth;

void setup() {
  initializeMPU();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  // turn off the LED during gyro calibration
  determineGyroDeviationPerAxis();
  digitalWrite(13, HIGH); // turn on the LED when calibration is over

  pinMode(13, INPUT);   // pin 13 will be used to read RC receiver pulses via ISR
  setupPinChangeInterrupts();

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
    Serial.print(noseX); Serial.print(", "); Serial.print(noseY); Serial.print(", "); Serial.println(noseZ);
    //Serial.print(rWingX); Serial.print(", "); Serial.print(rWingY); Serial.print(", "); Serial.println(rWingZ);
    Serial.print(rcChannel1PulseWidth); Serial.print(", "); Serial.print(rcChannel2PulseWidth); Serial.print(", ");
    Serial.print(rcChannel3PulseWidth); Serial.print(", "); Serial.print(rcChannel4PulseWidth); Serial.print(", ");
    Serial.print(rcChannel5PulseWidth); Serial.print(", "); Serial.println(rcChannel6PulseWidth);
  }
}

void initializeMPU()
{
  Wire.begin();
  // not calling Wirer.setClock() was causing the Wire.requestFrom(MPU_addr, 14, true) operation 
  // to take ~1400us "sometimes", whereas with this setting it takes always ~450us
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
  // set the full scale range of the gyroscope to +/- 500 deg/s
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);   // GYRO_CONFIG register (1B hex)
  Wire.write(B00001000);   // ---00--- (00 = 250 d/s, 01 = 500 d/s, 10 = 1000 d/s, 11 = 2000 d/s)
  gyroUnitToRadPerSecond = 500 * M_PI / 180 / 32767;  // full-scale config (deg/s) * conversion from deg to rad / maximum int16 value
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
  double gyroDeltaPitch = gyroX * gyroUnitToRadPerSecond * microsSinceLastRead / 1000000;
  double gyroDeltaRoll =  gyroY * gyroUnitToRadPerSecond * microsSinceLastRead / 1000000;
  double gyroDeltaYaw = gyroZ * gyroUnitToRadPerSecond * microsSinceLastRead / 1000000;

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

// accelerometerCorrectionFactor goes from 0 (disregard accelerometer info) to 1 
// (discard pitch/roll info coming from gyroscope, use accelerometer data only)
void correctOrientationAccordingToAccelInfo(double accelerometerCorrectionFactor)
{
  // Most of the time the aircraft is not going to be accelerating a lot in any
  // direction. Under that assumption, [accX, accY, accZ] is a vector pointing
  // approximatly "up" (away from the center of the Earth). This vector is in
  // aircraft coordinates and its scale is not relevant for this method, since
  // only its direction will be used.

  // transform [accX, accY, accZ] from aircraft coordinates into word coordinates
  // accelCeiling = where should be the ceiling vector pointing in world coordinates, 
  // according to the accelerometer, and assuming that the aircraft is not accelerating
  double  accelCeilingX = SCALAR_PROD(rWingX, noseX, ceilingX, accX, accY, accZ),
          accelCeilingY = SCALAR_PROD(rWingY, noseY, ceilingY, accX, accY, accZ),
          accelCeilingZ = SCALAR_PROD(rWingZ, noseZ, ceilingZ, accX, accY, accZ);
  // normalize accelCeiling vector
  double accelCeilingNorm = sqrt(SCALAR_PROD(accelCeilingX, accelCeilingY, accelCeilingZ, accelCeilingX, accelCeilingY, accelCeilingZ));
  accelCeilingX /= accelCeilingNorm;
  accelCeilingY /= accelCeilingNorm;
  accelCeilingZ /= accelCeilingNorm;

  // Now let's determine what's the smallest rotation (axis + angle) that needs 
  // to be performed to transform [accelCeilingX, accelCeilingY, accelCeilingZ] into 
  // [0, 0, 1].

  // If accelCeiling is already too close to [0, 0, 1], finding the smallest rotation that
  // turns one into the other will be pointless and very sensitive to numerical errors
  if (accelCeilingZ > 0.999)
    return;

  // angle = acos(SCALAR_PROD(accelCeiling, [0, 0, 1])) / (length(accelCeiling) * length([0, 0, 1]))
  //       = acos((0 + 0 + accelCeiling) / (1 * 1))
  double angle = acos(accelCeilingZ);   
  // In general, the rotation axis is defined as SOURCE x TARGET where "x" is
  // the "cross product". Since TARGET is [0, 0, 1], the cross product is trivial:
  double  rotationAxisX = accelCeilingY, 
          rotationAxisY = -accelCeilingX,
          rotationAxisZ = 0;
  // normalize rotationAxis
  double rotationAxisNorm = sqrt(SCALAR_PROD(rotationAxisX, rotationAxisY, rotationAxisZ, rotationAxisX, rotationAxisY, rotationAxisZ));
  rotationAxisX /= rotationAxisNorm;
  rotationAxisY /= rotationAxisNorm;
  rotationAxisZ /= rotationAxisNorm;

  angle *= accelerometerCorrectionFactor;
  double sinAngle = sin(angle), cosAngle = cos(angle);

  rotateVector(rWingX, rWingY, rWingZ,
                rotationAxisX, rotationAxisY, rotationAxisZ,
                sinAngle, cosAngle,
                rWingX, rWingY, rWingZ);
  rotateVector(noseX, noseY, noseZ,
                rotationAxisX, rotationAxisY, rotationAxisZ,
                sinAngle, cosAngle,
                noseX, noseY, noseZ);
  rotateVector(ceilingX, ceilingY, ceilingZ,
                rotationAxisX, rotationAxisY, rotationAxisZ,
                sinAngle, cosAngle,
                ceilingX, ceilingY, ceilingZ);                
}

void rotateVector(double vectX, double vectY, double vectZ, 
                  double axisX, double axisY, double axisZ,
                  double sinAngle, double cosAngle,
                  double &resultX, double &resultY, double &resultZ)
{
  double  crossProductX = axisY * vectZ - axisZ * vectY,
          crossProductY = axisZ * vectX - axisX * vectZ,
          crossProductZ = axisX * vectY - axisY * vectX;
  double scalarProduct = SCALAR_PROD(axisX, axisY, axisZ, vectX, vectY, vectZ);

  resultX = vectX * cosAngle + crossProductX * sinAngle + axisX * scalarProduct * (1 - cosAngle);
  resultY = vectY * cosAngle + crossProductY * sinAngle + axisY * scalarProduct * (1 - cosAngle);
  resultZ = vectZ * cosAngle + crossProductZ * sinAngle + axisZ * scalarProduct * (1 - cosAngle);
}

// We perform hundreds of orientation corrections per second to the orientation matrix,
// each correction involving multiple matrix multiplications. The theoretical result
// of all these operations is always an orthogonal matrix (its columns are
// orthogonal unit vectors), but in practice, numerical errors make the matrix
// progressively lose this condition, so we need to restablish it from time to time.
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

// Use pin change interrupts to keep track of RC receiver pulse widths "in parallel"
// with the program main loop
void setupPinChangeInterrupts()
{
  // initialize pulse states and widths
  rcChannel1PulseOn = rcChannel2PulseOn = rcChannel3PulseOn = 
    rcChannel4PulseOn = rcChannel5PulseOn = rcChannel6PulseOn = false;
  rcChannel1PulseWidth = rcChannel2PulseWidth = rcChannel3PulseWidth = 
    rcChannel4PulseWidth = rcChannel5PulseWidth = rcChannel6PulseWidth = 0;

  // Use digital pins 8 - 13 to monitor RC receiver channels 1 - 6
  // see http://playground.arduino.cc/Main/PinChangeInterrupt and https://www.arduino.cc/en/Reference/attachInterrupt
  PCICR |= (1 << PCIE0);      // Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= B00111111;        // All 6 pins from D8 to D13 will trigger an interrupt on state change.
}

// Interrupt Service Routine monitoring when the pulses coming from the each RC receiver channel start 
// and end to determine their widths.
ISR(PCINT0_vect){
  // Avoiding loops and arrays in order to make this function as fast as possible (as recommended for ISRs).
  // Will consider loops and arrays (shorter / more readable) at some point and see if it actually makes a difference.
  unsigned long currentTime = micros();
  //== Channel1 =========================================
  if(PINB & B00000001){                   //Is input 8 high?
    if (!rcChannel1PulseOn){              //Pin changed from LOW to HIGH
      rcChannel1PulseOn = true;                                                   
      rcChannel1PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel1PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel1PulseOn = false;
    rcChannel1PulseWidth = (currentTime - rcChannel1PulseStart);
  }
  //== Channel2 =========================================
  if(PINB & B00000010){                   //Is input 9 high?
    if (!rcChannel2PulseOn){              //Pin changed from LOW to HIGH
      rcChannel2PulseOn = true;                                                   
      rcChannel2PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel2PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel2PulseOn = false;
    rcChannel2PulseWidth = (currentTime - rcChannel2PulseStart);
  }
  //== Channel3 =========================================
  if(PINB & B00000100){                   //Is input 10 high?
    if (!rcChannel3PulseOn){              //Pin changed from LOW to HIGH
      rcChannel3PulseOn = true;                                                   
      rcChannel3PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel3PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel3PulseOn = false;
    rcChannel3PulseWidth = (currentTime - rcChannel3PulseStart);
  }
  //== Channel4 =========================================
  if(PINB & B00001000){                   //Is input 11 high?
    if (!rcChannel4PulseOn){              //Pin changed from LOW to HIGH
      rcChannel4PulseOn = true;                                                   
      rcChannel4PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel4PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel4PulseOn = false;
    rcChannel4PulseWidth = (currentTime - rcChannel4PulseStart);
  }
  //== Channel5 =========================================
  if(PINB & B00010000){                   //Is input 12 high?
    if (!rcChannel5PulseOn){              //Pin changed from LOW to HIGH
      rcChannel5PulseOn = true;                                                   
      rcChannel5PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel5PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel5PulseOn = false;
    rcChannel5PulseWidth = (currentTime - rcChannel5PulseStart);
  }
  //== Channel6 =========================================
  if(PINB & B00100000){                   //Is input 13 high?
    if (!rcChannel6PulseOn){              //Pin changed from LOW to HIGH
      rcChannel6PulseOn = true;                                                   
      rcChannel6PulseStart = currentTime; //Take note of when the pulse started
    }
  }
  else if (rcChannel6PulseOn)             // Pin changed from HIGH to LOW 
  {
    rcChannel6PulseOn = false;
    rcChannel6PulseWidth = (currentTime - rcChannel6PulseStart);
  }
}
