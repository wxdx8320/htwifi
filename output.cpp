/* This file is part of the Razor AHRS Firmware */
#include <Arduino.h>
#include "output.h"
#include "sensors.h"
#include "DCM.h"
#include "networking.h"

float magnetom1[3] = {10.0,20.0,30.0};
float accel1[3] = {0.0, 5.0, 10.0};
float gyro1[3] = {-1.0, -2.0, -3.0};

extern float magnetom[3];
extern float accel[3];
extern float gyro[3];

// Select your startup output mode and format here!
//int output_mode = OUTPUT__MODE_UDP;
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

char strBuffer[128];
double udpBuffer[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int getOutputMode() {
  return output_mode;
}

void setOutputMode(int mode) {
  output_mode = mode;
}

int getOutputFormat() {
  return output_format;
}

void setOutputFormat(int format) {
  output_format = format;
}

// Output angles: yaw, pitch, roll
void output_angles() {
  if (output_format == OUTPUT__FORMAT_BINARY) {
    float ypr[3];
    ypr[0] = TO_DEG(getYaw());
    ypr[1] = TO_DEG(getPitch());
    ypr[2] = TO_DEG(getRoll());
    Serial.write((byte*) ypr, 12);  // No new-line
  } else if (output_format == OUTPUT__FORMAT_TEXT) {
    Serial.print("#YPR=");
    Serial.print(TO_DEG(getYaw())); Serial.print(",");
    Serial.print(TO_DEG(getPitch())); Serial.print(",");
    Serial.print(TO_DEG(getRoll())); Serial.println();
  }
}

void output_calibration(int calibration_sensor) {
  if (calibration_sensor == 0) { // Accelerometer
    // Output MIN/MAX values
    Serial.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (getAccelAxis(i) < getAccelMin(i)) setAccelMin(i, getAccelAxis(i));
      if (getAccelAxis(i) > getAccelMax(i)) setAccelMax(i, getAccelAxis(i));
      Serial.print(getAccelMin(i));
      Serial.print("/");
      Serial.print(getAccelMax(i));
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  } else if (calibration_sensor == 1)  { // Magnetometer
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (getMagnAxis(i) < getMagnMin(i)) setMagnMin(i, getMagnAxis(i));
      if (getMagnAxis(i) > getMagnMin(i)) setMagnMax(i, getMagnAxis(i));
      Serial.print(getMagnMin(i));
      Serial.print("/");
      Serial.print(getMagnMax(i));
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  } else if (calibration_sensor == 2)  { // Gyroscope
    // Average gyro values
    for (int i = 0; i < 3; i++)
      setGyroAvg(i, getGyroAvg(i) + getGyroAxis(i));
    setGyroNumSamples(getGyroNumSamples()+1);

    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(getGyroAxis(i));
      Serial.print("/");
      Serial.print(getGyroAvg(i) / (float) getGyroNumSamples());
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated) {
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(getAccelAxis(0)); Serial.print(",");
  Serial.print(getAccelAxis(1)); Serial.print(",");
  Serial.print(getAccelAxis(2)); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(getMagnAxis(0)); Serial.print(",");
  Serial.print(getMagnAxis(1)); Serial.print(",");
  Serial.print(getMagnAxis(2)); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(getGyroAxis(0)); Serial.print(",");
  Serial.print(getGyroAxis(1)); Serial.print(",");
  Serial.print(getGyroAxis(2)); Serial.println();
}

void output_sensors_binary() {
  float m[3];
  m[0] = getMagnAxis(0);
  m[1] = getMagnAxis(1);
  m[2] = getMagnAxis(2);

  Serial.write((uint8_t*) accel, 12);
  Serial.write((uint8_t *) magnetom, 12);
  Serial.write((uint8_t*) gyro, 12);

/*
float t;
  Serial.write(getAccelAxis(0));
  Serial.write(getAccelAxis(1));
  Serial.write(getAccelAxis(2));

  t = getMagnAxis(0);
  Serial.write((byte *)&t, 4);
  Serial.write(getMagnAxis(1));
  Serial.write(getMagnAxis(2));

  Serial.write(getGyroAxis(0));
  Serial.write(getGyroAxis(1));
  Serial.write(getGyroAxis(2));
*/
/*
  Serial.write((uint8_t*) getAccelBuffer(), 12);
  Serial.write((uint8_t*) getMagnBuffer(), 12);
  Serial.write((uint8_t*) getGyroBuffer(), 12);
*/
}
static int ledPause=0;

void output_sensors() {

  if (output_mode == OUTPUT__MODE_SENSORS_RAW) {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  } else if (output_mode == OUTPUT__MODE_SENSORS_CALIB) {
    // Apply sensor calibration
    compensate_sensor_errors();

    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  } else if (output_mode == OUTPUT__MODE_SENSORS_BOTH) {
    if (output_format == OUTPUT__FORMAT_BINARY)  {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    } else if (output_format == OUTPUT__FORMAT_TEXT) {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  } else if(output_mode == OUTPUT__MODE_HATIRE) {
    // FIXME: not implemented
  } else if(output_mode == OUTPUT__MODE_UDP) {
    // TODO: implement movements
//    udpBuffer[0] = TO_DEG((double)getRoll());
//    udpBuffer[1] = TO_DEG((double)getPitch());
//    udpBuffer[2] = TO_DEG((double)getYaw());
    compensate_sensor_errors();
    udpBuffer[3] = TO_DEG((double)getYaw());
    udpBuffer[4] = TO_DEG((double)getPitch());
    udpBuffer[5] = TO_DEG((double)getRoll());
//    snprintf(strBuffer, 120, "%f %f %f 0.0, 0.0, 0.0", udpBuffer[0], udpBuffer[1], udpBuffer[2]);
/*
    Serial.print("UDP:");
    for(int i=0; i <6; i++) {
      Serial.print("\t");
      Serial.print(udpBuffer[i]);
    }
    Serial.println();
    */
#ifdef USE_WIFI
#ifdef STATUS_LED_PIN
    if(ledPause++ >= 30) {
      // blink to indicate wireless transmission
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      ledPause = 0;
    }
#endif
    if(sendBytes((unsigned char *)udpBuffer, 48 /*sizeof(udpBuffer)*/) != true) {
      Serial.println("Error sending to UDP");
    } else {
      output_sensors_text('C');
    }

#endif
  }
}
