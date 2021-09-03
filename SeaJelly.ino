/** @project SeaJelly v2.1
 *  @author Collin Chiang
 *  @date 6/30/2021
 *  @description The SeaJelly is an Arduino Nano 33 BLE wired to a
 *  soft-robotic, submersible vessel which can navigate and autonomously move
 *  around a liquid of water. The SeaJelly can be controlled through both
 *  bluetooth connection and an IRremote controller.
 *
 *  @requirements
 *      Arduino Nano 33 BLE
 *      ArduinoBLE
 *      Arduino_LSM9DS1
 *      Adafruit_MPRLS
 *      PinDefinitionsAndMore
 *      IRMP
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Adafruit_MPRLS.h>
#include <Arduino_LSM9DS1.h>
#include "PinDefinitionsAndMore.h"
#define IRMP_PROTOCOL_NAMES 1
#include <irmpSelectMain15Protocols.h>
#include <irmp.c.h>

#include "LedController.h"
#include "PumpController.h"
#include "SwimController.h"

// IRremote mapping
const int VOL_DOWN_BUTTON = 0;
const int PLAY_PAUSE = 1;
const int VOL_UP_BUTTON = 2;
const int SETUP_BUTTON = 4;
const int UP_BUTTON = 5;
const int STOP_MODE = 6;
const int LEFT_BUTTON = 8;
const int SAVE_BUTTON = 9;
const int RIGHT_BUTTON = 10;
const int ADD_BUTTON = 12;
const int DOWN_BUTTON = 13;
const int GO_BACK_BUTTON = 14;
const int BUTTON_1 = 16;
const int BUTTON_2 = 17;
const int BUTTON_3 = 18;
const int BUTTON_4 = 20;
const int BUTTON_5 = 21;
const int BUTTON_6 = 22;
const int BUTTON_7 = 24;
const int BUTTON_8 = 25;
const int BUTTON_9 = 26;

// arduino pin mapping
const byte RPUMP_PIN = D11;
const byte BPUMP_PIN = D12;
const byte TEMP_PIN = A2;
const byte VOLT_PIN = A7;

// variable constants
const int LOOP_PERIOD = 50;  // [ms] 1s / 20
const float MAX_DEPTH = 20.0;  // [ft] max pressure reading
const float MIN_DEPTH = 0.0;  // [ft] min pressure reading
const unsigned long VALID_IR_CODES[] = {VOL_DOWN_BUTTON, PLAY_PAUSE,
    VOL_UP_BUTTON, SETUP_BUTTON, UP_BUTTON, STOP_MODE, LEFT_BUTTON, SAVE_BUTTON,
    RIGHT_BUTTON, ADD_BUTTON, DOWN_BUTTON, GO_BACK_BUTTON, BUTTON_1, BUTTON_2,
    BUTTON_3, BUTTON_4, BUTTON_5, BUTTON_6, BUTTON_7, BUTTON_8, BUTTON_9
};

int loopCount;
unsigned long buttonCommand, pastCommand;
float initialHPA = 1013.0;

// class-object declaration
IRMP_DATA IRMPData;
Adafruit_MPRLS mprls(-1, -1);

/*
// BLE class-object declaration
BLEService SeaJelly("19B10000-E8F2-537E-AAAA-D104768A1214");
BLEUnsignedCharCharacteristic buttonCommandBLE("19B10000-E8F2-537E-1111-D104768A1214", BLEWrite);
BLEIntCharacteristic rPumpPowerBLE("19B10000-E8F2-537E-1112-D104768A1214", BLEWrite);
BLEIntCharacteristic bPumpPowerBLE("19B10000-E8F2-537E-1113-D104768A1214", BLEWrite);
BLEIntCharacteristic cyclePowerBLE("19B10000-E8F2-537E-1114-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic cycleRestBLE("19B10000-E8F2-537E-1115-D104768A1214", BLERead | BLEWrite);
BLEFloatCharacteristic currentDepthBLE("19B10000-E8F2-537E-1116-D104768A1214", BLEWrite);
BLEFloatCharacteristic targetDepthBLE("19B10000-E8F2-537E-1117-D104768A1214", BLERead | BLEWrite);
BLEFloatCharacteristic depthToleranceBLE("19B10000-E8F2-537E-1118-D104768A1214", BLEWrite);
BLEFloatCharacteristic currentAngleBLE("19B10000-E8F2-537E-1119-D104768A1214", BLEWrite);
BLEFloatCharacteristic targetAngleBLE("19B10000-E8F2-537E-111A-D104768A1214", BLERead | BLEWrite);
BLEFloatCharacteristic angleToleranceBLE("19B10000-E8F2-537E-111B-D104768A1214", BLEWrite);
BLEFloatCharacteristic batteryVoltageBLE("19B10000-E8F2-537E-111C-D104768A1214", BLEWrite);
BLEFloatCharacteristic batteryLevelBLE("19B10000-E8F2-537E-111D-D104768A1214", BLEWrite);
*/

// custom class-object declaration
LedController ledController(LEDR, LEDG, LEDB);
PumpController rPumpController(RPUMP_PIN);
PumpController bPumpController(BPUMP_PIN);
SwimController swimController(rPumpController, bPumpController);

// function prototypes
unsigned long getIRCommand(IRMP_DATA data, unsigned long pastCommand);
bool isValidIRCode(unsigned long IRCode);


// arduino initialization sequence
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); // for OpenLog
    delay(200);

    mprls.begin();
    Wire.setTimeout(20); // for pressure sensor I2C

    // IRMP initialization
    irmp_init();
    #ifdef ALTERNATIVE_IRMP_FEEDBACK_LED_PIN
    irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at ALTERNATIVE_IRMP_FEEDBACK_LED_PIN
    #endif

    /*
    BLE.begin();
    BLE.setLocalName("SeaJelly");
    BLE.setAdvertisedService(SeaJelly); // add the service UUID
    SeaJelly.addCharacteristic(buttonCommandBLE);
    SeaJelly.addCharacteristic(rPumpPowerBLE);
    SeaJelly.addCharacteristic(bPumpPowerBLE);
    SeaJelly.addCharacteristic(cyclePowerBLE);
    SeaJelly.addCharacteristic(cycleRestBLE);
    SeaJelly.addCharacteristic(currentDepthBLE);
    SeaJelly.addCharacteristic(targetDepthBLE);
    SeaJelly.addCharacteristic(depthToleranceBLE);
    SeaJelly.addCharacteristic(currentAngleBLE);
    SeaJelly.addCharacteristic(targetAngleBLE);
    SeaJelly.addCharacteristic(angleToleranceBLE);
    SeaJelly.addCharacteristic(batteryVoltageBLE);
    SeaJelly.addCharacteristic(batteryLevelBLE);
    BLE.addService(SeaJelly); // Add the battery service

    rPumpController.setPumpPower(0);
    bPumpController.setPumpPower(0);
    swimController.setPowerCycleTimer(500);
    swimController.setRestCycleTimer(500);
    swimController.setDepthTolerance(0.5);
    swimController.setTurnTolerance(5.0);

    buttonCommandBLE.writeValue(PLAY_PAUSE);
    rPumpPowerBLE.writeValue(0);
    bPumpPowerBLE.writeValue(0);
    cyclePowerBLE.writeValue(500);
    cycleRestBLE.writeValue(500);
    currentDepthBLE.writeValue(0.0);
    targetDepthBLE.writeValue(0.0);
    depthToleranceBLE.writeValue(0.5);
    currentAngleBLE.writeValue(0.0);
    targetAngleBLE.writeValue(0.0);
    angleToleranceBLE.writeValue(5.0);
    batteryVoltageBLE.writeValue(6.0);
    batteryLevelBLE.writeValue(100);
    */

    loopCount = 0;
    pastCommand = PLAY_PAUSE;

    pinMode(D6, OUTPUT);
}


void loop() {
    //BLE.poll();

    // loop manager
    int mainLoopStartTime = millis();

    // sets BLE buttonCommand data
    buttonCommand = getIRCommand(IRMPData, pastCommand);
    //buttonCommandBLE.writeValue(buttonCommand);

    /*
    // gets SeaJelly data for logging
    SwimController::Vector3 magneticField = getMagneticField();
    SwimController::Vector3 acceleration = getAcceleration();
    SwimController::Vector3 gyroscope = getGyroscope();
    SwimController::Vector3 currentAngle = getRotationAngle();
    SwimController::BatteryData batteryData = getBatteryData();
    SwimController::Temperature temperature = getTemperature();
    float currentDepth = getDepth();
    */

    /*
    // gets data from BLE
    if (cyclePowerBLE.written()) {
        swimController.setPowerCycleTimer(cyclePowerBLE.value());
    }

    if (cycleRestBLE.written()) {
        swimController.setRestCycleTimer(cycleRestBLE.value());
    }
    */

    unsigned int powerCycleTimer = swimController.getPowerCycleTimer();
    unsigned int restCycleTimer = swimController.getRestCycleTimer();

    /*
    // sets BLE turnTolerance and depthTolerance data
    if (angleToleranceBLE.written()) {
        swimController.setTurnTolerance(angleToleranceBLE.value());
    }

    if (depthToleranceBLE.written()) {
        swimController.setDepthTolerance(depthToleranceBLE.value());
    }
    */

    switch (buttonCommand) {
        case VOL_DOWN_BUTTON: {
            powerCycleTimer -= 100;
            restCycleTimer += 100;

            swimController.setPowerCycleTimer(powerCycleTimer);
            swimController.setRestCycleTimer(restCycleTimer);
            break;
        }

        case VOL_UP_BUTTON: {
            powerCycleTimer += 100;
            restCycleTimer -= 100;

            swimController.setPowerCycleTimer(powerCycleTimer);
            swimController.setRestCycleTimer(restCycleTimer);
            break;
        }

        case PLAY_PAUSE: {
            swimController.rest();
            break;
        }

        case STOP_MODE: {
            swimController.rest();
            break;
        }

        case GO_BACK_BUTTON: {
            break;
        }

        case UP_BUTTON: {
            swimController.swim(75, 75);
            break;
        }

        case DOWN_BUTTON: {
            swimController.rest();
            break;
        }

        case LEFT_BUTTON: {
            swimController.turnLeft();
            break;
        }

        case RIGHT_BUTTON: {
            swimController.turnRight();
            break;
        }

        case BUTTON_1: {
            SwimController::Vector3 currentAngle = getRotationAngle();
            Serial.print("Rotation Angle: ");
            Serial.println(currentAngle.y);
            swimController.turnLeft(60.0, currentAngle);
            break;
        }

        case BUTTON_2: {
            SwimController::Vector3 currentAngle = getRotationAngle();
            Serial.print("Rotation Angle: ");
            Serial.println(currentAngle.y);
            swimController.turnRight(60.0, currentAngle);
            break;
        }

        case BUTTON_3: {
            break;
        }

        case BUTTON_4: {
            break;
        }

        case BUTTON_5: {
            break;
        }

        case BUTTON_6: {
            break;
        }

        case BUTTON_7: {
            ledController.setBlack();
            break;
        }

        case BUTTON_8: {
            ledController.setRainbow();
            break;
        }

        case BUTTON_9: {
            ledController.setDisco();
            break;
        }

        default: {
            break;
        }
    }

    /*
    // updates BLE data values
    rPumpPowerBLE.writeValue(rPumpController.getPumpPower());
    bPumpPowerBLE.writeValue(bPumpController.getPumpPower());
    cyclePowerBLE.writeValue(swimController.getPowerCycleTimer());
    cycleRestBLE.writeValue(swimController.getRestCycleTimer());
    currentDepthBLE.writeValue(currentDepth);
    targetDepthBLE.writeValue(0.0);
    depthToleranceBLE.writeValue(swimController.getDepthTolerance());
    currentAngleBLE.writeValue(currentAngle.y);
    targetAngleBLE.writeValue(0.0);
    angleToleranceBLE.writeValue(swimController.getTurnTolerance());
    batteryVoltageBLE.writeValue(batteryData.voltage);
    batteryLevelBLE.writeValue(batteryData.level);
    */

    Serial.println(buttonCommand);
    Serial1.println(buttonCommand);

    pastCommand = buttonCommand;

    // iterate loopCount and slowdown/speedup looping time
    loopCount ++;
    int loopTime = millis() - mainLoopStartTime;
    int delayTime = LOOP_PERIOD - loopTime;
    if (delayTime > 0) {
        delay(delayTime);
    }
    else {
        Serial.println("[ERROR] Loop overloaded error.");  // log error
    }
}


/** @brief Allows the SeaJelly to recieve and decode incoming IR data.
 *
 *  @param data IRMP_DATA representing the incoming IR data.
 *  @return command long representing the decoded command.
 */
unsigned long getIRCommand(IRMP_DATA data, unsigned long pastCommand) {
    unsigned long command = pastCommand;

    // detect if data is ready
    if (irmp_get_data(&data)) {
        // detect repetitive flags
        if (!(data.flags & IRMP_FLAG_REPETITION)) {
            // validate IRCode
            if (isValidIRCode(data.command)) {
                command = data.command;  // get command
            }
        }
    }
    return command;
}


/** @brief Returns the magnetic field Vector3.
*
*  @return SensorController::Vector3 storing the magnetic field at x, y, z.
*/
SwimController::Vector3 getMagneticField() {
    SwimController::Vector3 magneticField = {0.0, 0.0, 0.0};

    // detects if IMU data is accessable
    if (IMU.magneticFieldAvailable()) {
        // reads magnetic field data
        IMU.readMagneticField(magneticField.x, magneticField.y, magneticField.z);
    }
    return magneticField;
}


/** @brief Returns the accleration Vector3.
*
*  @return SensorController::Vector3 storing the accleration at x, y, z.
*/
SwimController::Vector3 getAcceleration() {
    SwimController::Vector3 acceleration = {0.0, 0.0, 0.0};

    // detects if IMU data is accessible
    if (IMU.accelerationAvailable()) {
        // reads acceleration data
        IMU.readAcceleration(acceleration.x, acceleration.y, acceleration.z);
    }
    return acceleration;
}


/** @brief Returns the gyroscope Vector3.
*
*  @return SensorController::Vector3 storing the gyroscope at x, y, z.
*/
SwimController::Vector3 getGyroscope() {
    SwimController::Vector3 gyroscope = {0.0, 0.0, 0.0};

    // detects if IMU data is accessable
    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope data
        IMU.readGyroscope(gyroscope.x, gyroscope.y, gyroscope.z);
    }
    return gyroscope;
}


/** @brief Returns the rotation angle Vector3.
*
*  @return SensorController::Vector3 storing the rotation angle at x, y, z.
*/
SwimController::Vector3 getRotationAngle() {
    SwimController::Vector3 acceleration = getAcceleration();
    SwimController::Vector3 rotationAngle = {0.0, 0.0, 0.0};

    // scale acceleration at x, y, z to near angle measurements
    rotationAngle.x = acceleration.x * 100;
    rotationAngle.y = acceleration.y * 100;
    rotationAngle.z = acceleration.z * 100;

    // scale acceleration readings to angle measurements
    rotationAngle.x = map(rotationAngle.x, -100, 100, -90, 90);
    rotationAngle.y = map(rotationAngle.y, -100, 100, -90, 90);
    rotationAngle.z = map(rotationAngle.z, -100, 100, -90, 90);
    return rotationAngle;
}


/** @brief Returns the battery data BatteryData.
*
*  @return SensorController::BatteryData storing the battery voltage and level.
*/
SwimController::BatteryData getBatteryData() {
    SwimController::BatteryData batteryData = {0.0, 0};

    // convert battery voltage readings to voltage and a percent level
    batteryData.voltage = analogRead(VOLT_PIN) * 9.24 / 1023.0;
    batteryData.level = map(int(batteryData.voltage * 100.0), 690, 840, 0, 100);
    return batteryData;
}


/** @brief Returns the temperature Temperature.
*
*  @return SensorController::Temperature storing degC and degF floats.
*/
SwimController::Temperature getTemperature() {
    SwimController::Temperature temperature = {0.0, 0.0};

    // convert temperature readings to degC and degF
    temperature.degF = map(analogRead(TEMP_PIN), 0, 1023, -10, 125);
    temperature.degC = (temperature.degF * 9.0 / 5.0) + 32.0;
    return temperature;
}


/** @brief Returns the depth of the SeaJelly.
*
*  @return float storing the current depth of the SeaJelly.
*/
float getDepth() {
    float currentHPA = mprls.readPressure();
    float depth = 0;

    // detects invalid pressure readings
    if (!isnan(currentHPA)) {
        // convert pressure to a depth reading
        depth = ((currentHPA - initialHPA) * 100.0 / (1000.0 * 9.81)) * 3.28084;
    }
    return depth;
}


/** @brief Checks if the incoming IR signal is a valid IR code.
 *
 *  @param IRCode long representing the incoming decoded IR command.
 *  @return bool representing if the incoming signal is a valid signal.
 */
bool isValidIRCode(unsigned long IRCode) {
    for (unsigned int i = 0; i < sizeof(VALID_IR_CODES) / sizeof(unsigned long); i++) {
        if (IRCode == VALID_IR_CODES[i]) {
            return true;
        }
    }
    return false;
}
