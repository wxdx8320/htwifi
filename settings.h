#ifndef SETTINGS_H
#define SETTINGS_H
#define CALIBRATION__MAGN_USE_EXTENDED true

#define SENSOR_DEVICE GY85
/* SENSOR_DEVICE
MPU6050
GY85
GY86
MPU9150
MPU9250
GY521_271 - GY521 + GY271
GY521_273 - GY521 + GY273

*/

/**
  Project-wide settings
*/
#define USE_WIFI

#define USE_WEMOS

#ifdef USE_WEMOS
  #define PIN_SDA 14
  #define PIN_SCL 12
  #define STATUS_LED_PIN 2  // Pin number of status LED
#else
  #define PIN_SDA 0
  #define PIN_SCL 2
#endif

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

#endif /* end of include guard: SETTINGS_H */
