#ifndef SENSORS_H
#define SENSORS_H
#include <stdint.h>
#include "settings.h"

#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)


// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

void I2C_Init();
void Accel_Init();
void Read_Accel();
void Magn_Init();
void Read_Magn();
void Gyro_Init();
void Read_Gyro();

float getMagnAxis(unsigned int axis);
void setMagnMin(unsigned int axis, float val);
void setMagnMax(unsigned int axis, float val);
float getMagnMin(unsigned int axis);
float getMagnMax(unsigned int axis);
int getMagnErrors();
void setMagnErrors(int newerror);
float* getMagnBuffer();

float getAccelAxis(unsigned int axis);
float getAccelMin(unsigned int axis);
void setAccelMin(unsigned int axis, float val);
float getAccelMax(unsigned int axis);
void setAccelMax(unsigned int axis, float val);
int getAccelErrors();
void setAccelErrors();
float* getAccelBuffer();

float getGyroAxis(unsigned int axis);
float getGyroAvg(unsigned int axis);
void setGyroAvg(unsigned int axis, float val);
void check_reset_calibration_session();
int getGyroErrors();
float* getGyroBuffer();

void setGyroNumSamples(int val);
int getGyroNumSamples();

void compensate_sensor_errors();
void read_sensors();

void setOutputErrors(bool er);
bool getOutputErrors();

void updateCompass_Heading();
float getCompass_Heading();

void setDefaultCalibrationValues();
#endif /* end of include guard: SENSORS_H */
