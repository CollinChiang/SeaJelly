/** @project SwimController v1.0
 *  @author Collin Chiang
 *  @date 7/27/2021
 *  @description Object for controlling the swimming functions on the SeaJelly.
 */

#include "SwimController.h"


/** @brief SwimController class controls the swimming functions.
 *
 *  SwimController takes controllers from the PumpController and the
 *  SensorController to make percise movements in the water.
 *
 *  @param rPumpPin byte stores the red pump pin.
 *  @param bPumpPin byte stores the blue pump pin.
 *  @param voltPin byte stores the voltage reader pin.
 *  @param tempPin byte storing the temperate reader pin.
 */
SwimController::SwimController(PumpController &rPump, PumpController &bPump) : _rPump(rPump), _bPump(bPump) {
    // stores swimming powered and unpowered variables
    _powerCycleTimer = 500;
    _restCycleTimer = 500;

    // stores swimming tolerance variables
    _depthTolerance = 1.5;
    _turnTolerance = 10.0;

    // stores swim function variables
    _swimStage = 0;
    _swimStartTime = 0;
}


/** @brief Returns the current power cycle timer value.
*
*  @return unsigned int storing the power cycle timer value.
*/
unsigned int SwimController::getPowerCycleTimer() {
    return _powerCycleTimer;
}


/** @brief Returns the current rest cycle timer value.
 *
 *  @return unsigned int storing the rest cycle timer value.
 */
unsigned int SwimController::getRestCycleTimer() {
    return _restCycleTimer;
}


/** @brief Returns the current depth tolerance value.
 *
 *  @return float storing the depth tolerance value.
 */
float SwimController::getDepthTolerance() {
    return _depthTolerance;
}


/** @brief Returns the current turn tolerance value.
 *
 *  @return float storing the turn tolerance value.
 */
float SwimController::getTurnTolerance() {
    return _turnTolerance;
}


/** @brief Sets the power cycle timer class variable to a new unsigned int value.
 *
 *  @param powerCycleTimer unsigned int storing the new power cycle timer value.
 *  @return void.
 */
void SwimController::setPowerCycleTimer(unsigned int powerCycleTimer) {
    _powerCycleTimer = powerCycleTimer;
}


/** @brief Sets the rest cycle timer class variable to a new unsigned int value.
*
*  @param restCycleTimer unsigned int storing the new rest cycle timer value.
*  @return void.
*/
void SwimController::setRestCycleTimer(unsigned int restCycleTimer) {
    _restCycleTimer = restCycleTimer;
}


/** @brief Sets the depth tolerance class variable to a new float value.
*
*  @param depthTolerance float storing the new depth tolerance value.
*  @return void.
*/
void SwimController::setDepthTolerance(float depthTolerance) {
    _depthTolerance = depthTolerance;
}


/** @brief Sets the turn tolerance class variable to a new float value.
*
*  @param turnTolerance float storing the new turn tolerance value.
*  @return void.
*/
void SwimController::setTurnTolerance(float turnTolerance) {
    _turnTolerance = turnTolerance;
}


/** @brief Stops the pumps on the SeaJelly.
 *
 *  Sets the SeaJelly's red and blue pump controller pumping output to the int 0.
 *
 *  @return void.
 */
void SwimController::rest() {
    _swimStage = 0;

    _rPump.setPumpPower(0);
    _bPump.setPumpPower(0);
}


/** @brief Allows the SeaJelly to swim in a set cycle.
 *
 *  Swim cycle consists of 3 stages:
 *      Initialization stage: setting a timer.
 *      Powered stage: setting the pump controller pumping output to the values.
 *      Unpowered stage: setting the pump controller pumping output to the int 0.
 *
 *  @param rPumpPower int storing the desired pumping ouput for the red pump in
 *  the powered stage.
 *  @param bPumpPower int storing the desired pumping output for the blue pump in
 *  the powered stage.
 *  @return void.
 */
void SwimController::swim(int rPumpPower, int bPumpPower) {
    switch (_swimStage) {
        case 0:
            _swimStartTime = millis();
            _swimStage ++;
            break;

        case 1:
            _rPump.setPumpPower(rPumpPower);
            _bPump.setPumpPower(bPumpPower);

            if (millis() - _swimStartTime > _powerCycleTimer) {
                _swimStage ++;
            }
            break;

        case 2:
            _rPump.setPumpPower(0);
            _bPump.setPumpPower(0);

            if (millis() - _swimStartTime > _powerCycleTimer + _restCycleTimer) {
                _swimStage = 0;
            }
            break;
    }
}


/** @brief Allows the SeaJelly to continuously turn left.
 *
 *  swim() function is used to power the red pump to turn left.
 *
 *  @return void.
 */
void SwimController::turnLeft() {
    SwimController::swim(75, 0);
}


/** @brief Allows the SeaJelly to turn to a specific rotation angle (left).
 *
 *  Method overloading of the turnLeft() function.
 *  SensorController object is used to get the rotation angle of the SeaJelly.
 *  turnTolerance is used to create a maximum and minimum bound for the turning
 *  angle.
 *
 *  @param turnAngle float storing the target angle for the SeaJelly.
 *  @return void.
 */
void SwimController::turnLeft(float turnAngle, SwimController::Vector3 rotationAngle) {
    float turnMax = -turnAngle + _turnTolerance;
    float turnMin = -turnAngle - _turnTolerance;

    if (!(-rotationAngle.y <= turnMax && -rotationAngle.y >= turnMin)) {
        SwimController::turnLeft();
    }
}


/** @brief Allows the SeaJelly to continuously turn right.
 *
 *  swim() function is used to power the blue pump to turn right.
 *
 *  @return void.
 */
void SwimController::turnRight() {
    SwimController::swim(0, 75);
}


/** @brief Allows the SeaJelly to turn to a specific rotation angle (right).
 *
 *  Method overloading of the turnRight() function.
 *  SensorController object is used to get the rotation angle of the SeaJelly.
 *  turnTolerance is used to create a maximum and minimum bound for the turning
 *  angle.
 *
 *  @param turnAngle float storing the target angle for the SeaJelly.
 *  @return void.
 */
void SwimController::turnRight(float turnAngle, SwimController::Vector3 rotationAngle) {
    float turnMax = turnAngle + _turnTolerance;
    float turnMin = turnAngle - _turnTolerance;

    if (!(rotationAngle.y <= turnMax && rotationAngle.y >= turnMin)) {
        SwimController::turnRight();
    }
}


/** @brief Allows the SeaJelly to continuously flip over in the water.
 *
 *  SensorController object is used to get the rotation angle of the SeaJelly.
 *  The SeaJelly will turn into the direction it is already turning to flip.
 *
 *  @return void.
 */
void SwimController::flip(SwimController::Vector3 rotationAngle) {
    if (rotationAngle.y < 0.0) {
        SwimController::turnLeft();
    }
    else {
        SwimController::turnRight();
    }
}


/** @brief Allows the SeaJelly to try to stay facing up or down.
 *
 *  SensorController object is used to get the rotation angle of the SeaJelly.
 *  The SeaJelly will turn into the direction it is not turning to stabilize.
 *
 *  @return void.
 */
void SwimController::stablize(SwimController::Vector3 rotationAngle) {
    if (rotationAngle.y < 0.0) {
        SwimController::turnRight();
    }
    else {
        SwimController::turnLeft();
    }
}
