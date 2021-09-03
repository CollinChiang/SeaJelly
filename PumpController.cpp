/** @project PumpController v1.0
 *  @author Collin Chiang
 *  @date 7/27/2021
 *  @description Object for controlling the pumps on the SeaJelly.
 */

#include "PumpController.h"


/** @brief PumpController class controls the pumping output of the pump.
 *
 *  @param pPin byte storing the pump pin byte.
 */
PumpController::PumpController(int pPin) {
    // pin information
    _pPin = pPin;

    // sets default pump power
    _pPower = 0;

    // setting pin output
    pinMode(_pPin, OUTPUT);
}


/** @brief Returns the current pumping power.
 *
 *  @return int storing the pumping power.
 */
int PumpController::getPumpPower() {
    return _pPower;
}


/** @brief Sets the pumping power to a new int value.
 *
 *  analogWrite() is also called to write the pumping value to the pumping pin.
 *
 *  @param pPower int storing the new pumping power.
 *  @return void.
 */
void PumpController::setPumpPower(int pPower) {
    _pPower = pPower;

    // writes new power output to pin
    analogWrite(_pPin, _pPower);
}
