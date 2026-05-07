
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h" // Accelerometer Part 1
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <string>
#include <BasicLinearAlgebra.h>
using namespace BLA;


// IMU Lab Setup:
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

//ToF Lab Setup:
#define SHUTDOWN_PIN 4
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;

//Motor Driver Setup:
#define LEFT_MOTOR_BACKWARD_PIN 2
#define LEFT_MOTOR_FORWARD_PIN 3
#define RIGHT_MOTOR_BACKWARD_PIN 14
#define RIGHT_MOTOR_FORWARD_PIN 15

//////////// Temperature Stuff ////////////
#define RESOLUTION_BITS (8)      // choose resolution (explained in depth below)

#ifdef ADCPIN
#define EXTERNAL_ADC_PIN ADCPIN   // ADCPIN is the lowest analog capable pin exposed on the variant
#endif                            // - if no exposed pins are analog capable this will be undefined
                                  // - to use another pin provide an analog capable pin number such as:
                                  //   - A0 -> A9 (when analog pins are named sequentially from 0)
                                  //   - A11 -> A13, A16, A29, A31 -> A35 (when pins are named after Apollo3 pads)
                                  //   - A variant-specific pin number (when none of the above apply)

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "09b7e77f-37b8-4d5f-8e52-241aa421c19a"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_CURR_TIME_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    GET_ROLL_PITCH,
    SEND_ROLL_DATA,
    SEND_PITCH_DATA,
    COLLECT_GYR_PITCH_ROLL_YAW,
    SEND_GYR_PITCH,
    SEND_GYR_ROLL,
    SEND_GYR_YAW,
    SEND_TOF_1_DATA,
    SEND_TOF_2_DATA,
    GET_TOF_DATA,
    DRIVE_FORWARD,
    DRIVE_TO_WALL_PID,
    SEND_MOTOR_TIME_DATA,
    SEND_MOTOR_DATA,
    STABILIZE_ORIENTATION,
    SEND_DMP_TIME_DATA,
    SEND_DMP_YAW_DATA,
    SEND_SETPOINT_DATA,
    SEND_SETPOINT_TIME_DATA,
    SEND_TOF_TIME_DATA,
    DRIVE_TO_WALL_PID_KALMAN,
    SEND_KALMAN_TOF_2_DATA,
    STABILIZE_ORIENTATION_DRIFT,
    STABILIZE_ORIENTATION_TOF_READINGS,
    STABILIZE_ORIENTATION_INVERTED_PENDULUM
};

unsigned long TIME_STAMPS[10];
int TEMP_READINGS[10];
float PITCH_READINGS[10];
float ROLL_READINGS[10];
float GYR_PITCH_READINGS[10];
float GYR_ROLL_READINGS[10];
float GYR_YAW_READINGS[10];
int TOF_1_READINGS[10];
int MOTOR_INPUTS[500];
unsigned long MOTOR_TIME_STAMPS[500];
unsigned long DMP_TIME_STAMPS[500];
double DMP_YAW_READINGS[500];
unsigned long SETPOINT_TIME_STAMPS[500];
double SETPOINTS[500];
unsigned long TOF_TIME_STAMPS[500];
int TOF_2_READINGS[500];
float KALMAN_TOF_2_READINGS[500];


void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */
            float float_a, float_b, float_c;
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;
            
            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            /*
             * Your code goes here.
             */
            Serial.print(char_arr);
            Serial.println(", over!");
            tx_estring_value.clear();
            tx_estring_value.append("Robot Repeats -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(", over!");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
            break;
        
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        /*
         * GET_TIME_MILLIS
         */
        case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(float(currentMillis));
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        /*
         * GET_CURR_TIME_LOOP
         */
        case GET_CURR_TIME_LOOP:
            for (int i = 0; i < sizeof(TIME_STAMPS)/sizeof(TIME_STAMPS[0]); i++) {
                // tx_estring_value.clear();
                // tx_estring_value.append("T:");
                // tx_estring_value.append(float(millis()));
                // tx_characteristic_string.writeValue(tx_estring_value.c_str());
                TIME_STAMPS[i] = millis();
                TEMP_READINGS[i] = analogReadTemp();
            }
            break;

        case SEND_TIME_DATA:
            for (int i = 0; i < sizeof(TIME_STAMPS)/sizeof(unsigned long); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(TIME_STAMPS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        
        case SEND_TOF_TIME_DATA:
            for (int i = 0; i < sizeof(TOF_TIME_STAMPS)/sizeof(unsigned long); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(TOF_TIME_STAMPS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_MOTOR_TIME_DATA:
            for (int i = 0; i < sizeof(MOTOR_TIME_STAMPS)/sizeof(unsigned long); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(MOTOR_TIME_STAMPS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        
        case SEND_DMP_TIME_DATA:
            for (int i = 0; i < sizeof(DMP_TIME_STAMPS)/sizeof(unsigned long); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(DMP_TIME_STAMPS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_DMP_YAW_DATA:
            for (int i = 0; i < sizeof(DMP_YAW_READINGS)/sizeof(double); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(DMP_YAW_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_SETPOINT_DATA:
            for (int i = 0; i < sizeof(SETPOINTS)/sizeof(double); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(SETPOINTS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_SETPOINT_TIME_DATA:
            for (int i = 0; i < sizeof(SETPOINT_TIME_STAMPS)/sizeof(unsigned long); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(float(SETPOINT_TIME_STAMPS[i]));
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case GET_TEMP_READINGS:
            for (int i = 0; i < sizeof(TEMP_READINGS)/sizeof(int); i++) {
                tx_estring_value.clear();
                tx_estring_value.append("Time:");
                tx_estring_value.append(float(TIME_STAMPS[i]));
                tx_estring_value.append(", Temp:");
                tx_estring_value.append(float(TEMP_READINGS[i]));
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case GET_ROLL_PITCH:
            for (int i = 0; i < sizeof(ROLL_READINGS)/sizeof(float); i++) {
                if (myICM.dataReady()) {
                    myICM.getAGMT();
                    TIME_STAMPS[i] = millis();
                    ROLL_READINGS[i] = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                    PITCH_READINGS[i] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                } else {
                    Serial.println("Waiting for data");
                }
            }
            break;
        
        case SEND_ROLL_DATA:
            for (int i = 0; i < sizeof(ROLL_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(ROLL_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_PITCH_DATA:
            for (int i = 0; i < sizeof(PITCH_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(PITCH_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case COLLECT_GYR_PITCH_ROLL_YAW:
            if (myICM.dataReady()) {
                myICM.getAGMT();
                TIME_STAMPS[0] = millis();
                GYR_PITCH_READINGS[0] = 0.0f;
                GYR_ROLL_READINGS[0] = 0.0f;
                GYR_YAW_READINGS[0] = 0.0f;
            } else {
                delay(10);
            }
            float dt;
            for (int i = 1; i < sizeof(GYR_PITCH_READINGS)/sizeof(float); i++) {
                if (myICM.dataReady()) {
                    myICM.getAGMT();
                    TIME_STAMPS[i] = millis();
                    dt = (TIME_STAMPS[i] - TIME_STAMPS[i - 1]) / 1000.0f;
                    GYR_PITCH_READINGS[i] = (GYR_PITCH_READINGS[i - 1] + myICM.gyrY() * dt);
                    GYR_ROLL_READINGS[i] = (GYR_ROLL_READINGS[i - 1] + myICM.gyrX() * dt);
                    GYR_YAW_READINGS[i] = (GYR_YAW_READINGS[i - 1] + myICM.gyrZ() * dt);

                    ROLL_READINGS[i] = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                    PITCH_READINGS[i] = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                    delay(200);
                } else {
                    Serial.print("jeadpje");
                }
            }
            break;
        
        case SEND_GYR_PITCH:
            for (int i = 0; i < sizeof(GYR_PITCH_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(GYR_PITCH_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_GYR_ROLL:
            for (int i = 0; i < sizeof(GYR_ROLL_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(GYR_ROLL_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        
        case SEND_GYR_YAW:
            for (int i = 0; i < sizeof(GYR_YAW_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(GYR_YAW_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case GET_TOF_DATA:
            for (int i = 0; i < sizeof(TOF_1_READINGS)/sizeof(int); i++) {
                distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
                distanceSensor2.startRanging();
                while (!distanceSensor1.checkForDataReady() || !distanceSensor2.checkForDataReady())
                {
                    delay(1);
                }
                int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
                int distance2 = distanceSensor2.getDistance();
                distanceSensor1.clearInterrupt();
                distanceSensor2.clearInterrupt();
                distanceSensor1.stopRanging();
                distanceSensor2.stopRanging();
                TOF_1_READINGS[i] = distance1;
                TOF_2_READINGS[i] = distance2;
                TIME_STAMPS[i] = millis();
            }
            break;

        case SEND_TOF_1_DATA:
            for (int i = 0; i < sizeof(TOF_1_READINGS)/sizeof(int); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(TOF_1_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        
        case SEND_TOF_2_DATA:
            for (int i = 0; i < sizeof(TOF_2_READINGS)/sizeof(int); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(TOF_2_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_KALMAN_TOF_2_DATA:
            for (int i = 0; i < sizeof(KALMAN_TOF_2_READINGS)/sizeof(float); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(KALMAN_TOF_2_READINGS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case DRIVE_FORWARD: {
            int speed;
            float time;
            float motorStopTime;
            success = robot_cmd.get_next_value(speed);
            if (!success) return;
            success = robot_cmd.get_next_value(time);
            if (!success) return;
            success = robot_cmd.get_next_value(motorStopTime);
            if (!success) return;
            unsigned long ulTime = (unsigned long)time;
            unsigned long ulMotorStopTime = (unsigned long)motorStopTime;

            for (int i = 0; i < sizeof(TOF_2_READINGS)/sizeof(int); i++) {
                TOF_2_READINGS[i] = 0;
                TOF_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            distanceSensor2.startRanging();
            while (!distanceSensor2.checkForDataReady()) {
                delay(1);
            }
            int rawDistance = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            distanceSensor2.stopRanging();
            if (i < sizeof(TOF_2_READINGS)/sizeof(int)) {
                TOF_2_READINGS[i] = rawDistance;
                TOF_TIME_STAMPS[i] = millis();
                i++;
            }
            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, speed);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, speed);
            unsigned long startTime = millis();
            while (millis() - startTime < ulTime) {
                distanceSensor2.startRanging();
                if (distanceSensor2.checkForDataReady())
                {
                    rawDistance = distanceSensor2.getDistance();
                    distanceSensor2.clearInterrupt();
                    distanceSensor2.stopRanging();
                    if (i < sizeof(TOF_2_READINGS)/sizeof(int)) {
                        TOF_2_READINGS[i] = rawDistance;
                        TOF_TIME_STAMPS[i] = millis();
                        i++;
                    }
                }
                if (millis() - startTime > ulMotorStopTime) {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                }
            }
            distanceSensor2.stopRanging();
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }

        case SEND_MOTOR_DATA:
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                tx_estring_value.clear();
                tx_estring_value.append(MOTOR_INPUTS[i]);
                tx_estring_value.append(",");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case DRIVE_TO_WALL_PID:
        {
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(maxTime);
            if (!success) return;
            success = robot_cmd.get_next_value(stopWhenClose);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) {
                minCommand = 40;
            }

            const float targetDistance = 304.0f;
            const float stopTolerance = 10.0f;
            const int maxCommand = 255;

            int nugget = 0;

            float integral = 0.0f;
            float prevError = 0.0f;

            // Extrapolation state
            float prevPrevDistance = 0.0f;
            float prevDistance = 0.0f;
            unsigned long prevPrevTofTime = 0;
            unsigned long prevTofTime = 0;
            float estimatedDistance = 0.0f;

            // bool hasFirstSample = false;
            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(TOF_2_READINGS)/sizeof(int); i++) {
                TOF_2_READINGS[i] = 0;
                TOF_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_TIME_STAMPS)/sizeof(unsigned long); i++) {
                MOTOR_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            int k = 0;
            float alpha = 0.05f;
            float lastDerivative = 0.0f;

            distanceSensor2.startRanging();
            while (!distanceSensor2.checkForDataReady()) {
                delay(1);
            }
            int rawDistance = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            if (i < sizeof(TOF_2_READINGS)/sizeof(int)) {
                TOF_2_READINGS[i] = rawDistance;
                TOF_TIME_STAMPS[i] = millis();
                i++;
            }
            unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            prevDistance = (float)rawDistance;
            prevTofTime = millis();
            while (millis() - startTime < (unsigned long)maxTime)
            {
                unsigned long now = millis();
                int rawDistance = -1;

                if (distanceSensor2.checkForDataReady())
                {
                    rawDistance = distanceSensor2.getDistance();
                    distanceSensor2.clearInterrupt();
                    if (i < sizeof(TOF_2_READINGS)/sizeof(int)) {
                        TOF_2_READINGS[i] = rawDistance;
                        TOF_TIME_STAMPS[i] = millis();
                        i++;
                    }
                    prevPrevDistance = prevDistance;
                    prevPrevTofTime = prevTofTime;

                    prevDistance = (float)rawDistance;
                    prevTofTime = now;

                    hasSecondSample = true;

                    estimatedDistance = prevDistance;
                }
                else if (hasSecondSample)
                {
                    float dtTof = (prevTofTime - prevPrevTofTime); // / 1000.0f;
                    if (dtTof > 0.0f) {
                        float slope = (prevDistance - prevPrevDistance) / dtTof;
                        float dtPredict = (now - prevTofTime); // / 1000.0f;

                        estimatedDistance = prevDistance + slope * dtPredict;
                    }
                    else {
                        estimatedDistance = prevDistance;
                    }
                }
                else {
                    estimatedDistance = prevDistance;
                }

                // PID
                float dt = (now - prevPidTime); // / 1000.0f;
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevPidTime = now;
                float error = estimatedDistance - targetDistance;
                if (nugget < 10) {
                    Serial.println(dt);
                    Serial.println(prevError);
                    Serial.println(error);
                    Serial.println("new");
                    nugget += 1;
                }
                if (stopWhenClose == 1) {
                    if (fabs(error) <= stopTolerance) {
                        break;
                    }
                }
                integral += error * dt;
                float derivative = (error - prevError) / dt;
                prevError = error;
                derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                lastDerivative = derivative;
                float control = Kp * error + Ki * integral + Kd * derivative;
                int motorCommand = (int)control;

                if (motorCommand > maxCommand) motorCommand = maxCommand;
                if (motorCommand < -maxCommand) motorCommand = -maxCommand;
                

                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    MOTOR_TIME_STAMPS[k] = millis();
                    k++;
                }

                if (motorCommand > 0 && motorCommand < minCommand) {
                    motorCommand = minCommand;
                }
                if (motorCommand < 0 && motorCommand > -minCommand) {
                    motorCommand = -minCommand;
                }

                if (motorCommand > 0) {
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, motorCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, motorCommand);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, reverseCommand);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, reverseCommand);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                }
            }

            distanceSensor2.stopRanging();
            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }

        case DRIVE_TO_WALL_PID_KALMAN:
        {
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;
            float sigmaOne;
            float sigmaTwo;
            float sigmaThree;
            float distanceUncertainty;
            float velocityUncertainty;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(maxTime);
            if (!success) return;
            success = robot_cmd.get_next_value(stopWhenClose);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) {
                minCommand = 40;
            }
            success = robot_cmd.get_next_value(sigmaOne);
            if (!success) return;
            success = robot_cmd.get_next_value(sigmaTwo);
            if (!success) return;
            success = robot_cmd.get_next_value(sigmaThree);
            if (!success) return;
            success = robot_cmd.get_next_value(distanceUncertainty);
            if (!success) return;
            success = robot_cmd.get_next_value(velocityUncertainty);
            if (!success) return;

            float d = 0.272481406355;
            float m = 140.719661065;
            Matrix<2,2> A = {0.0f, 1.0f,
                             0.0f, -d/m};
            Matrix<2,1> B = {0.0f,
                             1.0f/m};
            Matrix<1,2> C = {1.0f, 0.0f};
            Matrix<2,1> x = {0.0f, 0.0f};
            Matrix<2,2> sigma = {distanceUncertainty * distanceUncertainty, 0.0f,
                            0.0f, velocityUncertainty * velocityUncertainty};

            Matrix<2,2> sig_u = {sigmaOne * sigmaOne, 0.0f,
                            0.0f, sigmaTwo * sigmaTwo};
            Matrix<1,1> sig_z = {sigmaThree * sigmaThree};

            Matrix<2,2> I = {1.0f, 0.0f,
                            0.0f, 1.0f};
            // float dt = 100.0;
            // Matrix<2,2> Ad = I + dt * A;
            // Matrix<2,1> Bd = dt * B;

            bool kfInitialized = false;
            unsigned long prevKalmanTime = 0;
            float estimatedDistance = 0.0f;
            float estimatedVelocity = 0.0f;

            const float targetDistance = 304.0f;
            const float stopTolerance = 10.0f;
            const int maxCommand = 255;

            int nugget = 0;

            float integral = 0.0f;
            float prevError = 0.0f;

            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(TOF_2_READINGS)/sizeof(int); i++) {
                TOF_2_READINGS[i] = 0;
                TOF_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_TIME_STAMPS)/sizeof(unsigned long); i++) {
                MOTOR_TIME_STAMPS[i] = 0;
                KALMAN_TOF_2_READINGS[i] = 0;
            }
            int i = 0;
            int k = 0;
            float alpha = 0.05f;
            float lastDerivative = 0.0f;

            distanceSensor2.startRanging();
            while (!distanceSensor2.checkForDataReady()) {
                delay(1);
            }
            int rawDistance = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            if (i < sizeof(TOF_2_READINGS)/sizeof(int)) {
                TOF_2_READINGS[i] = rawDistance;
                TOF_TIME_STAMPS[i] = millis();
                i++;
            }
            x(0,0) = (float)rawDistance;
            x(1,0) = 0.0f;
            estimatedDistance = x(0,0);
            estimatedVelocity = x(1,0);
            prevKalmanTime = millis();
            kfInitialized = true;
            // unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            while (millis() - startTime < (unsigned long)maxTime)
            {
                unsigned long now = millis();
                float dt = (now - prevKalmanTime);
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevKalmanTime = now;
                Matrix<2,2> Ad = I + dt * A;
                Matrix<2,1> Bd = dt * B;
                float u_ss = 130.0f / 255.0f;
                Matrix<1> u = {u_ss};
                Matrix<2,1> mu_p = Ad * x + Bd * u;
                Matrix<2,2> sigma_p = Ad * sigma * ~Ad + sig_u;
                bool update = false;
                float yValue = 0.0f;
                if (distanceSensor2.checkForDataReady())
                {
                    int rawDistance = distanceSensor2.getDistance();
                    distanceSensor2.clearInterrupt();
                    if (i < sizeof(TOF_2_READINGS) / sizeof(int)) {
                        TOF_2_READINGS[i] = rawDistance;
                        TOF_TIME_STAMPS[i] = millis();
                        i++;
                    }

                    update = true;
                    yValue = (float)rawDistance;
                }
                if (update)
                {
                    Matrix<1,1> y = {yValue};

                    Matrix<1,1> sigma_m = C * sigma_p * ~C + sig_z;
                    Matrix<2,1> kkf_gain = sigma_p * ~C * Inverse(sigma_m);

                    Matrix<1,1> y_m = y - C * mu_p;
                    x = mu_p + kkf_gain * y_m;
                    sigma = (I - kkf_gain * C) * sigma_p;
                } else {
                    x = mu_p;
                    sigma = sigma_p;
                }

                estimatedDistance = x(0,0);
                estimatedVelocity = x(1,0);

                // PID
                // float dt = (now - prevPidTime); // / 1000.0f;
                // if (dt < 0.1f) {
                //     dt = 0.1f;
                // }
                // prevPidTime = now;
                float error = estimatedDistance - targetDistance;
                if (stopWhenClose == 1) {
                    if (fabs(error) <= stopTolerance) {
                        break;
                    }
                }
                integral += error * dt;
                float derivative = (error - prevError) / dt;
                prevError = error;
                derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                lastDerivative = derivative;
                float control = Kp * error + Ki * integral + Kd * derivative;
                int motorCommand = (int)control;

                if (motorCommand > maxCommand) motorCommand = maxCommand;
                if (motorCommand < -maxCommand) motorCommand = -maxCommand;
                

                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    KALMAN_TOF_2_READINGS[k] = estimatedDistance;
                    MOTOR_TIME_STAMPS[k] = millis();
                    k++;
                }

                if (motorCommand > 0 && motorCommand < minCommand) {
                    motorCommand = minCommand;
                }
                if (motorCommand < 0 && motorCommand > -minCommand) {
                    motorCommand = -minCommand;
                }

                if (motorCommand > 0) {
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, motorCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, motorCommand);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, reverseCommand);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, reverseCommand);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                }
            }

            distanceSensor2.stopRanging();
            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }

        case STABILIZE_ORIENTATION: {
            bool success = true;
            // Initialize the DMP
            success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
            // Enable the DMP Game Rotation Vector sensor
            success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
            // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
            // Enable the FIFO queue
            success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
            // Enable the DMP
            success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
            // Reset DMP
            success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
            // Reset FIFO
            success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
            // Check success
            if (!success) {
                Serial.println("Enabling DMP failed!");
                while (1) {
                    // Freeze
                }
            }
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;
            float setPoint;
            int setPointTime;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(maxTime);
            if (!success) return;
            success = robot_cmd.get_next_value(stopWhenClose);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) {
                minCommand = 40;
            }
            success = robot_cmd.get_next_value(setPoint);
            if (!success) return;
            success = robot_cmd.get_next_value(setPointTime);
            if (!success) return;

            const double stopTolerance = 10.0;
            const int maxCommand = 255;

            float integral = 0.0f;
            float prevError = 0.0f;

            // Extrapolation state
            double prevPrevYaw = 0.0;
            double prevYaw = 0.0;
            unsigned long prevPrevDMPTime = 0;
            unsigned long prevDMPTime = 0;
            double estimatedYaw = 0.0;
            double targetYaw = 0.0;

            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(DMP_YAW_READINGS)/sizeof(double); i++) {
                DMP_YAW_READINGS[i] = 0;
                DMP_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
                MOTOR_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(SETPOINTS)/sizeof(double); i++) {
                SETPOINTS[i] = 0;
                SETPOINT_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            int j = 0;
            int k = 0;

            icm_20948_DMP_data_t data;
            double rawYaw = 0.0;
            do {
                myICM.readDMPdataFromFIFO(&data);
                delay(1);
            } while (!((myICM.status == ICM_20948_Stat_Ok) || 
                    (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)));
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    double qw = q0;
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;
                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    rawYaw = atan2(t3, t4) * 180.0 / PI;
                    if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                        DMP_YAW_READINGS[i] = rawYaw;
                        DMP_TIME_STAMPS[i] = millis();
                        i++;
                    }
                }
            }
            unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            setPoint += rawYaw;
            prevYaw = rawYaw;
            prevDMPTime = millis();
            while (millis() - startTime < (unsigned long)maxTime)
            {
                unsigned long now = millis();
                if (now - startTime > (unsigned long)setPointTime) {
                    targetYaw = (double)setPoint;
                }
                if (j < sizeof(SETPOINTS)/sizeof(double)) {
                    SETPOINTS[j] = targetYaw;
                    SETPOINT_TIME_STAMPS[j] = now;
                    j++;
                }
                rawYaw = -1.0;
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                    // We have asked for GRV data so we should receive Quat6
                    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                        double qw = q0;
                        double qx = q2;
                        double qy = q1;
                        double qz = -q3;
                        // yaw (z-axis rotation)
                        double t3 = +2.0 * (qw * qz + qx * qy);
                        double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                        rawYaw = atan2(t3, t4) * 180.0 / PI;
                        if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                            DMP_YAW_READINGS[i] = rawYaw;
                            DMP_TIME_STAMPS[i] = now;
                            i++;
                        }
                        prevPrevYaw = prevYaw;
                        prevPrevDMPTime = prevDMPTime;
                        prevYaw = rawYaw;
                        prevDMPTime = now;
                        hasSecondSample = true;
                        estimatedYaw = prevYaw;
                    }
                } else if (hasSecondSample) {
                    float dtDMP = (prevDMPTime - prevPrevDMPTime);
                    if (dtDMP > 0.0f) {
                        float slope = (prevYaw - prevPrevYaw) / dtDMP;
                        float dtPredict = (now - prevDMPTime);
                        estimatedYaw = prevYaw + slope * dtPredict;
                    }
                } else {
                    estimatedYaw = prevYaw;
                }

                // PID
                float dt = (now - prevPidTime);
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevPidTime = now;
                double error = estimatedYaw - targetYaw;
                if (error > 180.0) {
                    error = estimatedYaw - 180;
                    error = error + (-180 - targetYaw);
                } else if (error < -180.0) {
                    error = estimatedYaw + 180;
                    error = error + (180 - targetYaw);
                }
                if (stopWhenClose == 1) {
                    if (fabs(error) <= stopTolerance) {
                        break;
                    }
                }
                integral += error * dt;
                float derivative = (error - prevError) / dt;
                // derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                // lastDerivative = derivative;
                prevError = error;
                float control = Kp * error + Ki * integral + Kd * derivative;
                int motorCommand = (int)control;

                if (motorCommand > maxCommand) motorCommand = maxCommand;
                if (motorCommand < -maxCommand) motorCommand = -maxCommand;
                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    MOTOR_TIME_STAMPS[k] = now;
                    k++;
                }
                Serial.println(motorCommand);
                if (motorCommand > 0 && motorCommand < minCommand) {
                    motorCommand = minCommand;
                }
                if (motorCommand < 0 && motorCommand > -minCommand) {
                    motorCommand = -minCommand;
                }
                if (motorCommand > 0) {
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, motorCommand);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, motorCommand);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, reverseCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, reverseCommand);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                }
                if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
                    delay(10);
                }
            }

            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }

        case STABILIZE_ORIENTATION_DRIFT: {
            bool success = true;
            // Initialize the DMP
            success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
            // Enable the DMP Game Rotation Vector sensor
            success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
            // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2) == ICM_20948_Stat_Ok); // Set to the maximum
            // Enable the FIFO queue
            success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
            // Enable the DMP
            success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
            // Reset DMP
            success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
            // Reset FIFO
            success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
            // Check success
            if (!success) {
                Serial.println("Enabling DMP failed!");
                while (1) {
                    // Freeze
                }
            }
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;
            int maxCommand = 255;
            float setPoint;
            int setPointTime;
            int driveUpSpeed;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(maxTime);
            if (!success) return;
            success = robot_cmd.get_next_value(stopWhenClose);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) {
                minCommand = 40;
            }
            success = robot_cmd.get_next_value(maxCommand);
            if (!success) {
                maxCommand = 255;
            }
            success = robot_cmd.get_next_value(setPoint);
            if (!success) return;
            success = robot_cmd.get_next_value(setPointTime);
            if (!success) return;
            success = robot_cmd.get_next_value(driveUpSpeed);
            if (!success) return;

            const double stopTolerance = 10.0;
            float integral = 0.0f;
            float prevError = 0.0f;

            // Extrapolation state
            double prevPrevYaw = 0.0;
            double prevYaw = 0.0;
            unsigned long prevPrevDMPTime = 0;
            unsigned long prevDMPTime = 0;
            double estimatedYaw = 0.0;
            double targetYaw = 0.0;

            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(DMP_YAW_READINGS)/sizeof(double); i++) {
                DMP_YAW_READINGS[i] = 0;
                DMP_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
                MOTOR_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(SETPOINTS)/sizeof(double); i++) {
                SETPOINTS[i] = 0;
                SETPOINT_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            int j = 0;
            int k = 0;
            int driftCounter = 0;

            icm_20948_DMP_data_t data;
            double rawYaw = 0.0;
            do {
                myICM.readDMPdataFromFIFO(&data);
                delay(1);
            } while (!((myICM.status == ICM_20948_Stat_Ok) || 
                    (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)));
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    double qw = q0;
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;
                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    rawYaw = atan2(t3, t4) * 180.0 / PI;
                    if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                        DMP_YAW_READINGS[i] = rawYaw;
                        DMP_TIME_STAMPS[i] = millis();
                        i++;
                    }
                }
            }
            unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            targetYaw += rawYaw;
            setPoint += rawYaw;
            prevYaw = rawYaw;
            prevDMPTime = millis();
            while (millis() - startTime < (unsigned long)maxTime)
            {
                unsigned long now = millis();
                if (now - startTime > (unsigned long)setPointTime) {
                    targetYaw = (double)setPoint;
                }
                if (j < sizeof(SETPOINTS)/sizeof(double)) {
                    SETPOINTS[j] = targetYaw;
                    SETPOINT_TIME_STAMPS[j] = now;
                    j++;
                }
                rawYaw = -1.0;
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                    // We have asked for GRV data so we should receive Quat6
                    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                        double qw = q0;
                        double qx = q2;
                        double qy = q1;
                        double qz = -q3;
                        // yaw (z-axis rotation)
                        double t3 = +2.0 * (qw * qz + qx * qy);
                        double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                        rawYaw = atan2(t3, t4) * 180.0 / PI;
                        if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                            DMP_YAW_READINGS[i] = rawYaw;
                            DMP_TIME_STAMPS[i] = now;
                            i++;
                        }
                        prevPrevYaw = prevYaw;
                        prevPrevDMPTime = prevDMPTime;
                        prevYaw = rawYaw;
                        prevDMPTime = now;
                        hasSecondSample = true;
                        estimatedYaw = prevYaw;
                    }
                } else if (hasSecondSample) {
                    float dtDMP = (prevDMPTime - prevPrevDMPTime);
                    if (dtDMP > 0.0f) {
                        float slope = (prevYaw - prevPrevYaw) / dtDMP;
                        float dtPredict = (now - prevDMPTime);
                        estimatedYaw = prevYaw + slope * dtPredict;
                    }
                } else {
                    estimatedYaw = prevYaw;
                }

                // PID
                float dt = (now - prevPidTime);
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevPidTime = now;
                double error = estimatedYaw - targetYaw;
                if (error > 180.0) {
                    error = estimatedYaw - 180;
                    error = error + (-180 - targetYaw);
                } else if (error < -180.0) {
                    error = estimatedYaw + 180;
                    error = error + (180 - targetYaw);
                }
                // if (error < 5.0 or error > -5.0) {
                //     driftCounter += 1;
                // }
                // if (driftCounter > 8) {
                //     break;
                // }
                if (stopWhenClose == 1) {
                    if (fabs(error) <= stopTolerance) {
                        break;
                    }
                }
                integral += error * dt;
                float derivative = (error - prevError) / dt;
                // derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                // lastDerivative = derivative;
                prevError = error;
                float control = Kp * error + Ki * integral + Kd * derivative;
                int motorCommand = (int)control;

                if (motorCommand > maxCommand) motorCommand = maxCommand;
                if (motorCommand < -maxCommand) motorCommand = -maxCommand;
                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    MOTOR_TIME_STAMPS[k] = now;
                    k++;
                }
                if (motorCommand > 0 && motorCommand < minCommand) {
                    motorCommand = minCommand;
                }
                if (motorCommand < 0 && motorCommand > -minCommand) {
                    motorCommand = -minCommand;
                }
                int diffForward = 0;
                int diffBackward = 0;
                if (motorCommand > 0) {
                    if (motorCommand >= 128) {
                        int nugget = (motorCommand - 128) * 2;
                        diffBackward = nugget;
                    } else {
                        int nugget = (128 - motorCommand) * 2;
                        diffForward = nugget;
                    }
                    // diffForward = maxCommand - motorCommand;
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, diffBackward);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, diffForward);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, maxCommand);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    if (reverseCommand >= 128) {
                        int nugget = (reverseCommand - 128) * 2;
                        diffBackward = nugget;
                    } else {
                        int nugget = (128 - reverseCommand) * 2;
                        diffForward = nugget;
                    }
                    // diffForward = maxCommand - reverseCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, maxCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, diffForward);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, diffBackward);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, driveUpSpeed);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, driveUpSpeed);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                }
                if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
                    delay(10);
                }
            }

            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }



        case STABILIZE_ORIENTATION_TOF_READINGS: {
            bool success = true;
            // Initialize the DMP
            success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
            // Enable the DMP Game Rotation Vector sensor
            success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
            // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2) == ICM_20948_Stat_Ok); // Set to the maximum
            // Enable the FIFO queue
            success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
            // Enable the DMP
            success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
            // Reset DMP
            success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
            // Reset FIFO
            success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
            // Check success
            if (!success) {
                Serial.println("Enabling DMP failed!");
                while (1) {
                    // Freeze
                }
            }
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;
            float setPoint;
            int setPointTime;
            float turnAngle;
            float turnTimes;
            float rightMultiplier;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) return;
            success = robot_cmd.get_next_value(rightMultiplier);
            if (!success) return;
            success = robot_cmd.get_next_value(turnAngle);
            if (!success) return;
            success = robot_cmd.get_next_value(turnTimes);
            if (!success) return;

            const double stopTolerance = 10.0;
            const int maxCommand = 255;

            float integral = 0.0f;
            float prevError = 0.0f;

            // Extrapolation state
            double prevPrevYaw = 0.0;
            double prevYaw = 0.0;
            unsigned long prevPrevDMPTime = 0;
            unsigned long prevDMPTime = 0;
            double estimatedYaw = 0.0;
            double currTargetYaw = 0.0;
            double adjCurrTargetYaw = 0.0;

            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(DMP_YAW_READINGS)/sizeof(double); i++) {
                DMP_YAW_READINGS[i] = 0;
                DMP_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
                MOTOR_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(SETPOINTS)/sizeof(double); i++) {
                SETPOINTS[i] = 0;
                SETPOINT_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(TOF_2_READINGS)/sizeof(int); i++) {
                TOF_2_READINGS[i] = 0;
                TOF_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            int j = 0;
            int k = 0;
            int l = 0;

            icm_20948_DMP_data_t data;
            double rawYaw = 0.0;
            do {
                myICM.readDMPdataFromFIFO(&data);
                delay(1);
            } while (!((myICM.status == ICM_20948_Stat_Ok) || 
                    (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)));
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    double qw = q0;
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;
                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    rawYaw = atan2(t3, t4) * 180.0 / PI;
                    if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                        DMP_YAW_READINGS[i] = rawYaw;
                        DMP_TIME_STAMPS[i] = millis();
                        i++;
                    }
                }
            }
            unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            prevYaw = rawYaw;
            prevDMPTime = millis();
            bool hold = true;
            while (currTargetYaw <= turnTimes * turnAngle)
            {
                Serial.println(currTargetYaw);
                unsigned long now = millis();
                if (j < sizeof(SETPOINTS)/sizeof(double)) {
                    SETPOINTS[j] = currTargetYaw;
                    SETPOINT_TIME_STAMPS[j] = now;
                    j++;
                }
                rawYaw = -1.0;
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                    // We have asked for GRV data so we should receive Quat6
                    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                        double qw = q0;
                        double qx = q2;
                        double qy = q1;
                        double qz = -q3;
                        // yaw (z-axis rotation)
                        double t3 = +2.0 * (qw * qz + qx * qy);
                        double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                        rawYaw = atan2(t3, t4) * 180.0 / PI;
                        prevPrevYaw = prevYaw;
                        prevPrevDMPTime = prevDMPTime;
                        prevYaw = rawYaw;
                        prevDMPTime = now;
                        hasSecondSample = true;
                        estimatedYaw = prevYaw;
                        // Serial.println(rawYaw);
                        // Serial.println();
                        if (fabs(rawYaw - adjCurrTargetYaw) <= 0.5) {
                            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                            Serial.println("stallington");
                            distanceSensor2.startRanging();
                            while (!distanceSensor2.checkForDataReady())
                            {
                                delay(1);
                                Serial.println("stalling");
                            }
                            int distance = distanceSensor2.getDistance();
                            distanceSensor2.clearInterrupt();
                            distanceSensor2.stopRanging();

                            Serial.println();
                            if (l < sizeof(TOF_2_READINGS) / sizeof(int)) {
                                TOF_2_READINGS[l] = distance;
                                TOF_TIME_STAMPS[l] = millis();
                                l++;
                            }
                            if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                                DMP_YAW_READINGS[i] = rawYaw;
                                DMP_TIME_STAMPS[i] = millis();
                                i++;
                            }
                            hold = false;
                        }
                    }
                } else if (hasSecondSample) {
                    float dtDMP = (prevDMPTime - prevPrevDMPTime);
                    if (dtDMP > 0.0f) {
                        float slope = (prevYaw - prevPrevYaw) / dtDMP;
                        float dtPredict = (now - prevDMPTime);
                        estimatedYaw = prevYaw + slope * dtPredict;
                    }
                } else {
                    estimatedYaw = prevYaw;
                }

                // PID
                float dt = (now - prevPidTime);
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevPidTime = now;
                double error = estimatedYaw - adjCurrTargetYaw;
                if (error > 180.0) {
                    error = estimatedYaw - 180;
                    error = error + (-180 - adjCurrTargetYaw);
                } else if (error < -180.0) {
                    error = estimatedYaw + 180;
                    error = error + (180 - adjCurrTargetYaw);
                }
                if (fabs(error) <= 0.5 && hold == false) {
                    currTargetYaw += turnAngle;
                    adjCurrTargetYaw += turnAngle;
                    hold = true;
                }
                if (currTargetYaw > 180.0) {
                    adjCurrTargetYaw = currTargetYaw - 360;
                }
                integral += error * dt;
                float derivative = (error - prevError) / dt;
                // derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                // lastDerivative = derivative;
                // Serial.println(error - prevError);
                // Serial.println(dt);
                prevError = error;
                // Serial.println(error);
                // Serial.println(integral);
                // Serial.println(derivative);
                float control = Kp * error + Ki * integral + Kd * derivative;
                if (control > 0.1 && control < 1.0) {
                    control = 1.0;
                } else if (control < -0.1 && control > -1.0) {
                    control = -1.0;
                }
                int motorCommand = (int)control;
                // Serial.println(motorCommand);
                // Serial.println();
                int motorCommandRight = motorCommand * rightMultiplier;

                if (motorCommand > maxCommand) {
                    motorCommand = maxCommand;
                    motorCommandRight = maxCommand;
                }
                if (motorCommand < -maxCommand) {
                    motorCommand = -maxCommand;
                    motorCommandRight = -maxCommand;
                }
                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    MOTOR_TIME_STAMPS[k] = now;
                    k++;
                }
                if (motorCommand > 0 && motorCommand < minCommand) {
                    motorCommand = minCommand;
                    motorCommandRight = minCommand * rightMultiplier;
                }
                if (motorCommand < 0 && motorCommand > -minCommand) {
                    motorCommand = -minCommand;
                    motorCommandRight = -minCommand * rightMultiplier;
                }
                if (motorCommand > 0) {
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, motorCommand);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, motorCommandRight);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, reverseCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, -motorCommandRight);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                }
                if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
                    delay(10);
                }
            }

            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            break;
        }



        case STABILIZE_ORIENTATION_INVERTED_PENDULUM: {
            bool success = true;
            // Initialize the DMP
            success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
            // Enable the DMP Game Rotation Vector sensor
            success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
            // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
            // Enable the FIFO queue
            success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
            // Enable the DMP
            success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
            // Reset DMP
            success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
            // Reset FIFO
            success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
            // Check success
            if (!success) {
                Serial.println("Enabling DMP failed!");
                while (1) {
                    // Freeze
                }
            }
            int maxTime;
            float Kp, Ki, Kd;
            int stopWhenClose;
            int minCommand = 40;
            float setPoint;
            int setPointTime;
            float rightMultiplier;
            float alpha;
            float stopTolerance;
            int activeBrakePWM;

            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(maxTime);
            if (!success) return;
            success = robot_cmd.get_next_value(stopWhenClose);
            if (!success) return;
            success = robot_cmd.get_next_value(minCommand);
            if (!success) {
                minCommand = 40;
            }
            success = robot_cmd.get_next_value(setPoint);
            if (!success) return;
            success = robot_cmd.get_next_value(setPointTime);
            if (!success) return;
            success = robot_cmd.get_next_value(rightMultiplier);
            if (!success) return;
            success = robot_cmd.get_next_value(alpha);
            if (!success) return;
            success = robot_cmd.get_next_value(stopTolerance);
            if (!success) return;
            success = robot_cmd.get_next_value(activeBrakePWM);
            if (!success) return;

            const int maxCommand = 255;

            float integral = 0.0f;
            float prevError = 0.0f;

            // Extrapolation state
            double prevPrevYaw = 0.0;
            double prevYaw = 0.0;
            unsigned long prevPrevDMPTime = 0;
            unsigned long prevDMPTime = 0;
            double estimatedYaw = 0.0;
            double targetYaw = 0.0;

            float lastDerivative = 0.0;

            bool hasSecondSample = false;

            for (int i = 0; i < sizeof(DMP_YAW_READINGS)/sizeof(double); i++) {
                DMP_YAW_READINGS[i] = 0;
                DMP_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(MOTOR_INPUTS)/sizeof(int); i++) {
                MOTOR_INPUTS[i] = 0;
                MOTOR_TIME_STAMPS[i] = 0;
            }
            for (int i = 0; i < sizeof(SETPOINTS)/sizeof(double); i++) {
                SETPOINTS[i] = 0;
                SETPOINT_TIME_STAMPS[i] = 0;
            }
            int i = 0;
            int j = 0;
            int k = 0;

            icm_20948_DMP_data_t data;
            double rawYaw = 0.0;
            do {
                myICM.readDMPdataFromFIFO(&data);
                delay(1);
            } while (!((myICM.status == ICM_20948_Stat_Ok) || 
                    (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)));
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    double qw = q0;
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;
                    // pitch (y-axis rotation)
                    double t2 = +2.0 * (qw * qy - qx * qz);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    rawYaw = asin(t2) * 180.0 / PI;
                    if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                        DMP_YAW_READINGS[i] = rawYaw;
                        DMP_TIME_STAMPS[i] = millis();
                        i++;
                    }
                }
            }
            unsigned long prevPidTime = millis();
            unsigned long startTime = millis();
            setPoint += rawYaw;
            prevYaw = rawYaw;
            prevDMPTime = millis();
            float lastGyrReading = 0.0;
            while (millis() - startTime < (unsigned long)maxTime)
            {
                unsigned long now = millis();
                if (now - startTime > (unsigned long)setPointTime) {
                    targetYaw = (double)setPoint;
                }
                if (j < sizeof(SETPOINTS)/sizeof(double)) {
                    SETPOINTS[j] = targetYaw;
                    SETPOINT_TIME_STAMPS[j] = now;
                    j++;
                }
                rawYaw = -1.0;
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                    // We have asked for GRV data so we should receive Quat6
                    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                        double qw = q0;
                        double qx = q2;
                        double qy = q1;
                        double qz = -q3;
                        // pitch (y-axis rotation)
                        double t2 = +2.0 * (qw * qy - qx * qz);
                        t2 = t2 > 1.0 ? 1.0 : t2;
                        t2 = t2 < -1.0 ? -1.0 : t2;
                        rawYaw = asin(t2) * 180.0 / PI;
                        if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                            DMP_YAW_READINGS[i] = rawYaw;
                            DMP_TIME_STAMPS[i] = now;
                            i++;
                        }
                        prevPrevYaw = prevYaw;
                        prevPrevDMPTime = prevDMPTime;
                        prevYaw = rawYaw;
                        prevDMPTime = now;
                        hasSecondSample = true;
                        estimatedYaw = prevYaw;
                    }
                } else if (hasSecondSample) {
                    float dtDMP = (prevDMPTime - prevPrevDMPTime);
                    if (dtDMP > 0.0f) {
                        float slope = (prevYaw - prevPrevYaw) / dtDMP;
                        float dtPredict = (now - prevDMPTime);
                        estimatedYaw = prevYaw + slope * dtPredict;
                    }
                } else {
                    estimatedYaw = prevYaw;
                }

                // icm_20948_DMP_data_t data;
                // myICM.readDMPdataFromFIFO(&data);
                // if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
                //     // We have asked for GRV data so we should receive Quat6
                //     if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                //         double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                //         double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                //         double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                //         double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                //         double qw = q0;
                //         double qx = q2;
                //         double qy = q1;
                //         double qz = -q3;
                //         // pitch (y-axis rotation)
                //         double t2 = +2.0 * (qw * qy - qx * qz);
                //         t2 = t2 > 1.0 ? 1.0 : t2;
                //         t2 = t2 < -1.0 ? -1.0 : t2;
                //         rawPitch = asin(t2) * 180.0 / PI;
                //         if (i < sizeof(DMP_PITCH_READINGS)/sizeof(double)) {
                //             DMP_PITCH_READINGS[i] = rawPitch;
                //             DMP_TIME_STAMPS[i] = now;
                //             i++;
                //         }
                //         prevPrevPitch = prevPitch;
                //         prevPrevDMPTime = prevDMPTime;
                //         prevPitch = rawPitch;
                //         prevDMPTime = now;
                //         hasSecondSample = true;
                //         estimatedPitch = prevPitch;
                //     }
                // } else if (hasSecondSample) {
                //     float dtDMP = (prevDMPTime - prevPrevDMPTime);
                //     if (dtDMP > 0.0f) {
                //         float slope = (prevPitch - prevPrevPitch) / dtDMP;
                //         float dtPredict = (now - prevDMPTime);
                //         estimatedPitch = prevPitch + slope * dtPredict;
                //     }
                // } else {
                //     estimatedPitch = prevPitch;
                // }

                // PID
                float dt = (now - prevPidTime);
                if (dt < 0.1f) {
                    dt = 0.1f;
                }
                prevPidTime = now;
                double error = estimatedYaw - targetYaw;
                if (error > 180.0) {
                    error = estimatedYaw - 180;
                    error = error + (-180 - targetYaw);
                } else if (error < -180.0) {
                    error = estimatedYaw + 180;
                    error = error + (180 - targetYaw);
                }
                if (stopWhenClose == 1) {
                    if (fabs(error) <= stopTolerance) {
                        break;
                    }
                }
                integral += error * dt;
                float derivative;
                // if (myICM.dataReady()) {
                //     myICM.getAGMT();
                //     derivative = myICM.gyrY();
                //     if (i < sizeof(DMP_YAW_READINGS)/sizeof(double)) {
                //         DMP_YAW_READINGS[i] = rawYaw;
                //         DMP_TIME_STAMPS[i] = now;
                //         i++;
                //     }
                //     lastGyrReading = derivative;
                // } else {
                //     derivative = lastGyrReading;
                // }
                derivative = (error - prevError) / dt;
                derivative = alpha * derivative + (1.0 - alpha) * lastDerivative;
                lastDerivative = derivative;
                prevError = error;
                float control = Kp * error + Ki * integral + Kd * derivative;
                int motorCommand = (int)control;
                if (motorCommand > 0) {
                    motorCommand += minCommand;
                } else if (motorCommand < 0) {
                    motorCommand -= minCommand;
                }
                int motorCommandRight = motorCommand * rightMultiplier;

                if (motorCommand > maxCommand) {
                    motorCommand = maxCommand;
                    motorCommandRight = maxCommand;
                }
                if (motorCommand < -maxCommand) {
                    motorCommand = -maxCommand;
                    motorCommandRight = -maxCommand;
                }
                if (k < sizeof(MOTOR_INPUTS)/sizeof(int)) {
                    MOTOR_INPUTS[k] = motorCommand;
                    MOTOR_TIME_STAMPS[k] = now;
                    k++;
                }
                // if (motorCommand > 0 && motorCommand < minCommand) {
                //     motorCommand = minCommand;
                //     motorCommandRight = minCommand * rightMultiplier;
                // }
                // if (motorCommand < 0 && motorCommand > -minCommand) {
                //     motorCommand = -minCommand;
                //     motorCommandRight = -minCommand * rightMultiplier;
                // }
                if (fabs(error) <= stopTolerance) {
                    motorCommand = 0;
                }
                if (motorCommand > 0) {
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, motorCommand);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, motorCommandRight);
                }
                else if (motorCommand < 0) {
                    int reverseCommand = -motorCommand;
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, reverseCommand);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, -motorCommandRight);
                }
                else {
                    analogWrite(LEFT_MOTOR_FORWARD_PIN, activeBrakePWM);
                    analogWrite(LEFT_MOTOR_BACKWARD_PIN, activeBrakePWM);
                    analogWrite(RIGHT_MOTOR_FORWARD_PIN, activeBrakePWM);
                    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, activeBrakePWM);
                }
                // if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
                //     delay(10);
                // }
            }

            analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
            analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
            analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
            break;
        }


        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    //Temperature Stuff included in setup()
    analogReadResolution(RESOLUTION_BITS);
    analogWriteResolution(RESOLUTION_BITS);

    //IMU stuff
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(WIRE_PORT, AD0_VAL);
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println("Trying again...");
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    //ToF stuff
    Wire.begin();
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW);
    distanceSensor2.setI2CAddress(0x2A);
    if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
    {
        Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (1)
        ;
    }
    digitalWrite(SHUTDOWN_PIN, HIGH);
    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    {
        Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (1)
        ;
    }
    // distanceSensor1.setDistanceModeShort();
    // distanceSensor2.setDistanceModeShort();
    distanceSensor1.setDistanceModeLong();
    distanceSensor2.setDistanceModeLong();
    Serial.println("Sensors online!");

    //Motor Driver Stuff
    pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{            
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
