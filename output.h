#ifndef OUTPUT_H
#define OUTPUT_H
#include "Settings.h"

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
#define OUTPUT__MODE_HATIRE 5 // Outputs data as hatire format
#define OUTPUT__MODE_UDP    6 // Outputs data as UDP tracker format

// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT   0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

void output_calibration(int calibration_sensor);
// Output angles: yaw, pitch, roll
void output_angles();

void output_sensors();

void setOutputMode(int mode);
int getOutputMode();

int getOutputFormat();
void setOutputFormat(int format);

#endif /* end of include guard: OUTPUT_H */
