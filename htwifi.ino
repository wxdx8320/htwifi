/***************************************************************************************************************
* ESP8266 AHRS firmware 
* features IDP opentrack protocol
* can save calibration data to EEPROM
* Released under GNU GPL (General Public License) v3.0
*
* Based on Razor AHRS by Peter Bartz
*
* TODO:
* - configure network settings via serial comm and save to EEPROM
* - make use of MPU6050, MPU9x50, and other common sensors
* - HATIRE protocol for compatibility with Arduino boards
* - backwards compatibility with Arduino
*
* History:
*   Razor AHRS Firmware v1.4.2
*   9 Degree of Measurement Attitude and Heading Reference System
*   for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
*   and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
*   Released under GNU GPL (General Public License) v3.0
*   Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
*   Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
*   Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
* etc
***************************************************************************************************************/

/*
  GY-85:
  ADXL345  : Accelerometer
  HMC5883L : Magnetometer
  ITG-3200 : Gyro
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.

  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up

  Transformation order: first yaw then pitch then roll.
*/
#include <Arduino.h>

#include "settings.h"
#include "sensors.h"
#include "DCM.h"
#include "output.h"

#include <EEPROM.h>

/**
 * WIFI section starts
 */
#ifdef USE_WIFI
  #include "networking.h"
#endif // USE_WIFI

/**
 * WIFI section ends
 */
 // 12 bytes
 #define EEPROM_CALIBRATION_CENTER 0
 // 4*9 => 36 bytes
 #define EEPROM_CALIBRATION_MATRIX 12
 // 3 * 4 = 12 bytes
 #define EEPROM_ACCEL_CALIBRATION 48
 // 3 * 4 = 12 bytes
#define EEPROM_GYRO_CALIBRATION 60
 // 128 bytes
 #define EEPROM_WIFI_SSID 72
 // 128 bytes
 #define EEPROM_WIFI_PWD 200
 // 4 bytes
 #define EEPROM_NET_SRC_IP 204
 // 4 bytes
 #define EEPROM_NET_DST_IP 208
 // 4 bytes
 #define EEPROM_NET_DNS_IP 212
 // 4 bytes
 #define EEPROM_NET_GW_IP 216

 #define EEPROM_SIZE 220

/*
  Serial commands that the firmware understands:

  "#o<params>" - Set OUTPUT mode and parameters. The available options are:

      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.

      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).

      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.

      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).

      // Error message output
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.

      // Tracker output
      '#ou' - UDP opentracker format to UDP
      '#ok' - send hatire data to serial

  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.


  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.

  "#c<params>" work with calibration data
    #cc
    #cd
    #cr
    #cs


  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)

  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/

//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds


// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
//boolean output_errors = false;  // true or false

extern float magn_ellipsoid_center[3];
extern float magn_ellipsoid_transform[3][3];

//const float magn_ellipsoid_center[3] = {32,2479, -13,3706, -13,5504};
//const float magn_ellipsoid_transform[3][3] = {{0,846939, 0,0160681, -0,00226731}, {0,0160681, 0,864563, 0,00742070}, {-0,00226731, 0,00742070, 0,999581}};
/*
// Calibration example:

// "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
#define ACCEL_X_MIN ((float) -277)
#define ACCEL_X_MAX ((float) 264)
#define ACCEL_Y_MIN ((float) -256)
#define ACCEL_Y_MAX ((float) 278)
#define ACCEL_Z_MIN ((float) -299)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
//#define MAGN_X_MIN ((float) -511)
//#define MAGN_X_MAX ((float) 581)
//#define MAGN_Y_MIN ((float) -516)
//#define MAGN_Y_MAX ((float) 568)
//#define MAGN_Z_MIN ((float) -489)
//#define MAGN_Z_MAX ((float) 486)

// Extended magn
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

// Extended magn (with Sennheiser HD 485 headphones)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

//"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
#define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
#define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
#define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
*/
#define CALIBRATION__MAGN_USE_EXTENDED true


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/




// Stuff

void saveCalibrationToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  for(int i=0; i < 3; i++) {
    EEPROM.put(EEPROM_CALIBRATION_CENTER+i*4, magn_ellipsoid_center[i]);
  }
  for(int i=0; i < 3; i++) {
    for(int j=0; j < 3; j++) {
      EEPROM.put(EEPROM_CALIBRATION_MATRIX+(i+j)*4, magn_ellipsoid_transform[i][j]);
    }
  }
  EEPROM.end();
}

void readCalibrationFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  for(int i=0; i < 3; i++) {
    EEPROM.get(EEPROM_CALIBRATION_CENTER+i*4, magn_ellipsoid_center[i]);
    if(isnan(magn_ellipsoid_center[i])) {
      // handle empty data
      magn_ellipsoid_center[i] = 0;
    }
  }
  for(int i=0; i < 3; i++) {
    for(int j=0; j < 3; j++) {
      EEPROM.get(EEPROM_CALIBRATION_MATRIX+(i+j)*4, magn_ellipsoid_transform[i][j]);
      if(isnan(magn_ellipsoid_transform[i][j])) {
        // handle empty data
        magn_ellipsoid_transform[i][j] = 0;
      }
    }
  }
  EEPROM.end();
}

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;

void turn_output_stream_on() {
  output_stream_on = true;
  #ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, HIGH);
  #endif
}

void turn_output_stream_off() {
  output_stream_on = false;
  #ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, LOW);
  #endif
}

// Blocks until another byte is available on serial port
char readChar() {
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

float readFloat() {
  while (Serial.available() < 1) { } // Block
  String s = Serial.readStringUntil('\r');
  double res = s.toFloat();
  return res;
}

void setup() {
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);

  // Init status LED
  #ifdef STATUS_LED_PIN
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  #endif

  // Init sensors
  delay(50);  // Give sensors enough time to start
  Serial.println("Starting I2C");
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
#ifdef USE_WIFI
  Serial.println("Starting WiFi");
  InitWifi();
#endif
  //Serial.println("Setting default calibration values");
  //setDefaultCalibrationValues();
  Serial.println("Reading EEPROM");
  readCalibrationFromEEPROM();
  // Init output
  Serial.println("Starting output stream");
  turn_output_stream_on();
}

void processCommand() {
  // Read incoming control messages
  if (Serial.available() >= 2) {
    if (Serial.read() == '#') {// Start of new control message
      int command = Serial.read(); // Commands
      if (command == 'f') {// request one output _f_rame
        output_single_on = true;
      } else if (command == 's') { // _s_ynch request
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();

        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      } else if (command == 'o') { // Set _o_utput mode
        char output_param = readChar();
        if (output_param == 'n') { // Calibrate _n_ext sensor
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        } else if (output_param == 't') { // Output angles as _t_ext
          setOutputMode(OUTPUT__MODE_ANGLES);
          setOutputFormat(OUTPUT__FORMAT_TEXT);
        } else if (output_param == 'b') { // Output angles in _b_inary format
          setOutputMode(OUTPUT__MODE_ANGLES);
          setOutputFormat(OUTPUT__FORMAT_BINARY);
        } else if (output_param == 'c') { // Go to _c_alibration mode
          setOutputMode(OUTPUT__MODE_CALIBRATE_SENSORS);
          reset_calibration_session_flag = true;
        } else if (output_param == 's') { // Output _s_ensor values
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  {// Output _r_aw sensor values
            setOutputMode(OUTPUT__MODE_SENSORS_RAW);
          } else if (values_param == 'c') { // Output _c_alibrated sensor values
            setOutputMode(OUTPUT__MODE_SENSORS_CALIB);
          } else if (values_param == 'b') { // Output _b_oth sensor values (raw and calibrated)
            setOutputMode(OUTPUT__MODE_SENSORS_BOTH);
          }
          if (format_param == 't') {// Output values as _t_text
            setOutputFormat(OUTPUT__FORMAT_TEXT);
          } else if (format_param == 'b') {// Output values in _b_inary format
            setOutputFormat(OUTPUT__FORMAT_BINARY);
          }
        } else if (output_param == '0') { // Disable continuous streaming output
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        } else if (output_param == '1') { // Enable continuous streaming output
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        } else if (output_param == 'e') { // _e_rror output settings
          char error_param = readChar();
          if (error_param == '0') setOutputErrors(false);
          else if (error_param == '1') setOutputErrors(true);
          else if (error_param == 'c') { // get error count
            Serial.print("#AMG-ERR:");
            Serial.print(getAccelErrors()); Serial.print(",");
            Serial.print(getMagnErrors()); Serial.print(",");
            Serial.println(getGyroErrors());
          }
        } else if (output_param == 'k') { // Output in hatire format
          setOutputMode(OUTPUT__MODE_HATIRE);
        } else if (output_param == 'u') { // Output to UDP
          #ifdef USE_WIFI
          #ifdef STATUS_LED_PIN
          digitalWrite(STATUS_LED_PIN, LOW);
          #endif
          #endif
          setOutputMode(OUTPUT__MODE_UDP);
        }
      } else if (command == 'c') { // calibration data
        char output_param = readChar();
        if (output_param == 'd') { // show (display) current calibration values
          Serial.print("\nmagn_ellipsoid_center = ");
          for(int i=0; i < 3; i++) {
            Serial.print(magn_ellipsoid_center[i], 4);
            Serial.print("\t");
          }
          Serial.println();
          Serial.print("magn_ellipsoid_transform = ");
          for(int i=0; i < 3; i++) {
            for(int j=0; j < 3; j++) {
              Serial.print(magn_ellipsoid_transform[i][j], 4);
              Serial.print("\t");
            }
            Serial.println();
          }
          Serial.println();
        } else if (output_param == 'p') { // put new calibration values from Serial
          for(int i=0; i < 3;i++) {
            magn_ellipsoid_center[i] = readFloat();
            Serial.println(magn_ellipsoid_center[i]);
          }
          for(int i=0; i < 3;i++) {
            for(int j=0; j < 3;j++) {
              magn_ellipsoid_transform[i][j] = readFloat();
              Serial.println(magn_ellipsoid_transform[i][j]);
            }
          }
        } else if (output_param == 'c') { // clear calibration values
/*
          for(int i=0; i < 3;i++) {
            magn_ellipsoid_center[i] = default_ellipsoid_center[i];
          }
          for(int i=0; i < 3;i++) {
            for(int j=0; j < 3;j++) {
              magn_ellipsoid_transform[i][j] = default_ellipsoid_transform[i][j];
            }
          }
*/
          setDefaultCalibrationValues();
        } else if (output_param == 'r') { // read calibration from EEPROM
           readCalibrationFromEEPROM();
           Serial.println("Calibration data extracted");
        } else if (output_param == 's') { // save calibration to EEPROM
          saveCalibrationToEEPROM();
          Serial.println("Calibration data saved");
        } else  { } // Skip character
      }
    }
  }
  #ifdef USE_WIFI
  #ifdef STATUS_LED_PIN
  if(getOutputMode() != OUTPUT__MODE_UDP) {
    // turn off
      digitalWrite(STATUS_LED_PIN, HIGH);
  }
  #endif
  #endif
}

void processUDPcommand() {
  char buf[32];
  unsigned int avail = receiveBytes((uint8_t *)buf, 30);
  unsigned int pos=0;
// debug
  if(avail > 0)
    Serial.println(buf);
  // Read incoming control messages
  if (avail >= 2) {
    if (buf[pos++] == '#') {// Start of new control message
      int command = buf[pos++]; // Commands
      if (command == 'f') {// request one output _f_rame
        output_single_on = true;
      } else if (command == 's') { // _s_ynch request
        // Read ID
        byte id[2];
        id[0] = buf[pos++];
        id[1] = buf[pos++];

        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      } else if (command == 'o') { // Set _o_utput mode
        char output_param = buf[pos++];
        if (output_param == 'n') { // Calibrate _n_ext sensor
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        } else if (output_param == 't') { // Output angles as _t_ext
          setOutputMode(OUTPUT__MODE_ANGLES);
          setOutputFormat(OUTPUT__FORMAT_TEXT);
        } else if (output_param == 'b') { // Output angles in _b_inary format
          setOutputMode(OUTPUT__MODE_ANGLES);
          setOutputFormat(OUTPUT__FORMAT_BINARY);
        } else if (output_param == 'c') { // Go to _c_alibration mode
          setOutputMode(OUTPUT__MODE_CALIBRATE_SENSORS);
          reset_calibration_session_flag = true;
        } else if (output_param == 's') { // Output _s_ensor values
          char values_param = buf[pos++];
          char format_param = buf[pos++];
          if (values_param == 'r')  {// Output _r_aw sensor values
            setOutputMode(OUTPUT__MODE_SENSORS_RAW);
          } else if (values_param == 'c') { // Output _c_alibrated sensor values
            setOutputMode(OUTPUT__MODE_SENSORS_CALIB);
          } else if (values_param == 'b') { // Output _b_oth sensor values (raw and calibrated)
            setOutputMode(OUTPUT__MODE_SENSORS_BOTH);
          }
          if (format_param == 't') {// Output values as _t_text
            setOutputFormat(OUTPUT__FORMAT_TEXT);
          } else if (format_param == 'b') {// Output values in _b_inary format
            setOutputFormat(OUTPUT__FORMAT_BINARY);
          }
        } else if (output_param == '0') { // Disable continuous streaming output
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        } else if (output_param == '1') { // Enable continuous streaming output
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        } else if (output_param == 'e') { // _e_rror output settings
          char error_param = buf[pos++];
          if (error_param == '0') setOutputErrors(false);
          else if (error_param == '1') setOutputErrors(true);
          else if (error_param == 'c') { // get error count
            Serial.print("#AMG-ERR:");
            Serial.print(getAccelErrors()); Serial.print(",");
            Serial.print(getMagnErrors()); Serial.print(",");
            Serial.println(getGyroErrors());
          }
        } else if (output_param == 'k') { // Output in hatire format
          setOutputMode(OUTPUT__MODE_UDP);
        } else if (output_param == 'u') { // Output to UDP
          setOutputMode(OUTPUT__MODE_UDP);
        }
      } else if (command == 'c') { // calibration data
        char output_param = buf[pos++];
        if (output_param == 'd') { // show (display) current calibration values
          Serial.print("\nmagn_ellipsoid_center = ");
          for(int i=0; i < 3; i++) {
            Serial.print(magn_ellipsoid_center[i]);
            Serial.print("\t");
          }
          Serial.println();
          Serial.print("magn_ellipsoid_transform = ");
          for(int i=0; i < 3; i++) {
            for(int j=0; j < 3; j++) {
              Serial.print(magn_ellipsoid_transform[i][j]);
              Serial.print("\t");
            }
            Serial.println();
          }
          Serial.println();
        } else if (output_param == 'p') { // put new calibration values from Serial
          for(int i=0; i < 3;i++) {
            magn_ellipsoid_center[i] = readFloat();
            Serial.println(magn_ellipsoid_center[i]);
          }
          for(int i=0; i < 3;i++) {
            for(int j=0; j < 3;j++) {
              magn_ellipsoid_transform[i][j] = readFloat();
              Serial.println(magn_ellipsoid_transform[i][j]);
            }
          }
        } else if (output_param == 'c') { // clear calibration values
          setDefaultCalibrationValues();
        } else if (output_param == 'r') { // read calibration from EEPROM
           readCalibrationFromEEPROM();
           Serial.println("Calibration data extracted");
        } else if (output_param == 's') { // save calibration to EEPROM
          saveCalibrationToEEPROM();
          Serial.println("Calibration data saved");
        } else  { } // Skip character
      }
    }
  }
}

// Main loop
void loop() {

  processCommand();
  processUDPcommand();

  // Time to read the sensors again?
  if(getInterval() >= OUTPUT__DATA_INTERVAL) {
    updateInterval();

    // Update sensor readings
    read_sensors();
//    Serial.print("Mode: ");
//    Serial.println(getOutputMode());

    if (getOutputMode() == OUTPUT__MODE_CALIBRATE_SENSORS)  { // We're in calibration mode
      // Reset this calibration session?
      if (reset_calibration_session_flag) {
        check_reset_calibration_session();  // Check if this session needs a reset
        reset_calibration_session_flag = false;
      }
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    } else if (getOutputMode() == OUTPUT__MODE_ANGLES)  { // Output angles
      // Apply sensor calibration
      compensate_sensor_errors();

      // Run DCM algorithm
      updateCompass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();

      if (output_stream_on || output_single_on) {
        output_angles();
      }
    } else if (getOutputMode() == OUTPUT__MODE_UDP)  { // Output angles
      // Apply sensor calibration
      compensate_sensor_errors();

      // Run DCM algorithm
      updateCompass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();

      if (output_stream_on || output_single_on) {
        output_sensors();
      }
    } else { // Output sensor values
      if (output_stream_on || output_single_on) output_sensors();
    }

    output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println((micros() - timestamp)*1000.0);
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else {
    Serial.println("waiting...");
  }
#endif
}
