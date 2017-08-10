#include <Arduino.h>
#include <Wire.h>
#include "sensors.h"
#include "DCM.h"

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;

/*
internal variables
*/
float magnetom[3];
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float gyro[3];

float magnetom_tmp[3];

float MAG_Heading;

// Sensor variables
float accel_min[3];
float accel_max[3];
float magnetom_min[3];
float magnetom_max[3];

float gyro_average[3];
int gyro_num_samples = 0;

int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float magn_ellipsoid_center[3] = {26.1835, 13.2903, 17.3125};
float magn_ellipsoid_transform[3][3] = {{0.840036, -0.0141205, -0.00335269}, {-0.0141205, 0.860271, 0.00513197}, {-0.00335269, 0.00513197, 0.999717}};

const float default_ellipsoid_center[3] = {0.0, 0.0, 0.0};
const float default_ellipsoid_transform[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors
// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b)
  #define WIRE_RECEIVE() Wire.read()
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive()
#endif
/*
#ifdef GY85
#include <HMC58853L.h>
HMC5883L mag;
#endif
*/

void I2C_Init() {
  Wire.begin(PIN_SDA,PIN_SCL);
//  mag.initialize();
}

void Accel_Init() {
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

int getAccelErrors() {
  return num_accel_errors;
}

void setAccelErrors(int accerr) {
  num_accel_errors = accerr;
}

// Reads x, y and z accelerometer registers
void Read_Accel() {
  int i = 0;
  byte buff[6];
  int16_t tmpaxis;
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available()) { // ((Wire.available())&&(i<6))
      buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6)  {// All bytes received?
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    tmpaxis = (((int16_t) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
//    Serial.print("x=");Serial.print(tmpaxis);
    accel[0] = (float)tmpaxis;
    tmpaxis = (((int16_t) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
//    Serial.print(" y=");Serial.print(tmpaxis);
    accel[1] = (float)tmpaxis;
    tmpaxis = (((int16_t) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
//    Serial.print(" z=");Serial.println(tmpaxis);
    accel[2] = (float)tmpaxis;
  } else {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
  }
}

void Magn_Init() {
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02);
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

// TODO: insert assert in all helpers
float* getMagnBuffer() {
  return magnetom;
}
float getMagnAxis(unsigned int axis) {
  return magnetom[axis];
}
float getMagnMin(unsigned int axis) {
  return magnetom_min[axis];
}
float getMagnMax(unsigned int axis) {
  return magnetom_max[axis];
}
void setMagnMin(unsigned int axis, float val) {
  magnetom_min[axis] = val;
}
void setMagnMax(unsigned int axis, float val) {
  magnetom_max[axis] = val;
}

float getAccelAxis(unsigned int axis) {
  return accel[axis];
}
float getAccelMin(unsigned int axis) {
  return accel_min[axis];
}
void setAccelMin(unsigned int axis, float val) {
  accel_min[axis] = val;
}
float getAccelMax(unsigned int axis) {
  return accel_max[axis];
}
void setAccelMax(unsigned int axis, float val) {
  accel_max[axis] = val;
}

float* getAccelBuffer() {
  return accel;
}


float getGyroAxis(unsigned int axis) {
  return gyro[axis];
}
float getGyroAvg(unsigned int axis) {
  return gyro_average[axis];
}
void setGyroAvg(unsigned int axis, float val) {
  gyro_average[axis] = val;
}
int getGyroNumSamples() {
  return gyro_num_samples;
}
void setGyroNumSamples(int val) {
  gyro_num_samples = val;
}
float* getGyroBuffer() {
  return gyro;
}

void Read_Magn() {
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available()) { // ((Wire.available())&&(i<6))

    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6) { // All bytes received?

    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    int16_t x,y,z;
    x = (int16_t) buff[0] << 8; x |= buff[1];
    y = (int16_t) buff[4] << 8; y |= buff[5];
    z = (int16_t) buff[2] << 8; z |= buff[3];

    magnetom[0] = x;
    magnetom[1] = -1.0 * y;
    magnetom[2] = -1.0 * z;
/*
    magnetom[0] = 1.0 * x;
    magnetom[1] = -1.0 * y;
    magnetom[2] = -1.0 * z;
*/
//    magnetom[0] = 1.0 * ((((int16_t) buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)
//    magnetom[1] = -1.0 * ((((int16_t) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
//    magnetom[2] = -1.0 * ((((int16_t) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
/*
    Serial.print(magnetom[0]);
    Serial.print("\t");
    Serial.print(magnetom[1]);
    Serial.print("\t");
    Serial.println(magnetom[2]);
*/

  } else {
    num_magn_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

int getMagnErrors() {
  return num_magn_errors;
}

void setMagnErrors(int newerror) {
  num_magn_errors = newerror;
}

void Gyro_Init() {
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);

  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);

  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro() {
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available()) { // ((Wire.available())&&(i<6))
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6) { // All bytes received?
    int16_t x,y,z;
    x = (int16_t)((((int16_t) buff[2]) << 8) | (int16_t)buff[3]);    // X axis (internal sensor -y axis)
    y = (int16_t)((((int16_t) buff[0]) << 8) | (int16_t)buff[1]);    // Y axis (internal sensor -x axis)
    z = (int16_t)((((int16_t) buff[4]) << 8) | (int16_t)buff[5]);    // Z axis (internal sensor -z axis)
    gyro[0] = y * -1.0;
    gyro[1] = x * -1.0;
    gyro[2] = z * -1.0;
  } else {
    num_gyro_errors++;
    if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}

int getGyroErrors() {
  return num_gyro_errors;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session() {
  // Raw sensor values have to be read already, but no error compensation applied

    // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

void updateCompass_Heading() {
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(getRoll());
  sin_roll = sin(getRoll());
  cos_pitch = cos(getPitch());
  sin_pitch = sin(getPitch());

  // Tilt compensated magnetic field X
  mag_x = getMagnAxis(0) * cos_pitch + getMagnAxis(1) * sin_roll * sin_pitch + getMagnAxis(2) * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = getMagnAxis(1) * cos_roll - getMagnAxis(2) * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}

float getCompass_Heading() {
  // Magnetic Heading
  return MAG_Heading;
}

void setOutputErrors(bool er) {
  output_errors = er;
}

bool getOutputErrors() {
  return output_errors;
}

void setDefaultCalibrationValues() {
  for(int i=0; i < 3;i++) {
    magn_ellipsoid_center[i] = default_ellipsoid_center[i];
  }
  for(int i=0; i < 3;i++) {
    for(int j=0; j < 3;j++) {
      magn_ellipsoid_transform[i][j] = default_ellipsoid_transform[i][j];
    }
  }
}
