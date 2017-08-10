/**
* DCM algorithm definitions
*/
#ifndef DCM_H
#define DCM_H
#include <stdint.h>
//#include <Adruino.h>

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);

// helpers

float getYaw();
float getPitch();
float getRoll();

float Vector_Dot_Product(const float v1[3], const float v2[3]);
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
void Vector_Scale(float out[3], const float v[3], float scale);
void Vector_Add(float out[3], const float v1[3], const float v2[3]);
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);

void reset_sensor_fusion();
unsigned long getInterval();
void updateInterval();

#endif
/* end of include guard: DCM_H */
