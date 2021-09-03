/** @project LedController v1.0
 *  @author Collin Chiang
 *  @date 7/26/2021
 *  @description Object for controlling the RGB led on the SeaJelly.
 */

#include "LedController.h"


/** @brief LedController class controls the functionalities of the on board RGB
 *  led.
 *
 *  @param rPin int storing the red led pin number.
 *  @param gPin int storing the green led pin number.
 *  @param bPin int storing the blue led pin number.
 */
LedController::LedController(int rPin, int gPin, int bPin) {
    // pin information
    _rPin = rPin;
    _gPin = gPin;
    _bPin = bPin;

    // times color changing intervals
    _colorTimer = 500;

    // method stage and timer variables
    _discoStage = 0;
    _discoStartTime = 0;

    _rainbowStage = 0;
    _rainbowStartTime = 0;

    // setting pin outputs
    pinMode(_rPin, OUTPUT);
    pinMode(_gPin, OUTPUT);
    pinMode(_bPin, OUTPUT);
}


/** @brief Returns the colorTimer value.
 *
 *  @return unsigned long storing the colorTimer class variable.
 */
unsigned long LedController::getColorTimer() {
    return _colorTimer;
}


/** @brief Sets the colorTimer value to a new unsigned long.
 *
 *  @param colorTimer unsigned long storing the new colorTimer value.
 *  @return void.
 */
void LedController::setColorTimer(unsigned long colorTimer) {
    _colorTimer = colorTimer;
}


/** @brief Writes the desired color and bytes into the R, G, and B pins.
 *
 *  @param rByte byte storing the amount of red for the red led.
 *  @param gByte byte storing the amount of green for the green led.
 *  @param bByte byte storing the amount of blue for the blue led.
 *  @return void.
 */
void LedController::setColor(byte rByte, byte gByte, byte bByte) {
    // writes data into the leds
    digitalWrite(_rPin, 1 - rByte);
    digitalWrite(_gPin, 1 - gByte);
    digitalWrite(_bPin, 1 - bByte);
}


/** @brief Sets the RGB led to the color red.
 *
 *  Red is composed of 1, 0, 0.
 *
 *  @return void.
 */
void LedController::setRed() {
    LedController::setColor(1, 0, 0);
}


/** @brief Sets the RGB led to the color green.
 *
 *  Green is composed of 0, 1, 0.
 *
 *  @return void.
 */
void LedController::setGreen() {
    LedController::setColor(0, 1, 0);
}


/** @brief Sets the RGB led to the color blue.
 *
 *  Blue is composed of 0, 0, 1.
 *
 *  @return void.
 */
void LedController::setBlue() {
    LedController::setColor(0, 0, 1);
}


/** @brief Sets the RGB led to the color yellow.
 *
 *  Yellow is composed of 1, 1, 0.
 *
 *  @return void.
 */
void LedController::setYellow() {
    LedController::setColor(1, 1, 0);
}


/** @brief Sets the RGB led to the color aquamarine.
 *
 *  Aquamine is composed of 0, 1, 1.
 *
 *  @return void.
 */
void LedController::setAquamarine() {
    LedController::setColor(0, 1, 1);
}


/** @brief Sets the RGB led to the color purple.
 *
 *  Purple is composed of 1, 0, 1.
 *
 *  @return void.
 */
void LedController::setPurple() {
    LedController::setColor(1, 0, 1);
}


/** @brief Sets the RGB led to the color black.
 *
 *  Black is composed of 0, 0, 0 (turning off all leds).
 *
 *  @return void.
 */
void LedController::setBlack() {
    LedController::setColor(0, 0, 0);
}


/** @brief Sets the RGB led to the color white.
 *
 *  White is composed of 1, 1, 1 (turning on all leds).
 *
 *  @return void.
 */
void LedController::setWhite() {
    LedController::setColor(1, 1, 1);
}


/** @brief Sets the RGB led to generate and show a random set of colors.
 *
 *  Random colors are generated using the random() function which generates an
 *  integer between 0 and 1. This is done 3 times for the red, green, and blue
 *  colors.
 *
 *  _discoStage and _discoStartTime are class variables used as state holders.
 *  _discoStage stores the stage of the method (initialization, display color).
 *  _discoStartTime stores the start millis() of the initialization stage. This
 *  is used for detecting when to change colors.
 *
 *  @return void.
 */
void LedController::setDisco() {
    // declares color variables
    int rColor, gColor, bColor;

    // detects _discoStage stage
    switch (_discoStage) {
        // initialization stage
        case 0:
            _discoStartTime = millis();
            _discoStage ++;
            break;

        // display color stage
        case 1:
            // randomizes RGB colors
            rColor = random(2);
            gColor = random(2);
            bColor = random(2);

            // writes colors to the RGB led
            LedController::setColor(rColor, gColor, bColor);

            // waits for next color display
            if (millis() - _discoStartTime > _colorTimer) {
                _discoStage = 0;
            }
            break;
    }

    // clears other stage data
    _rainbowStage = 0;
}


/** @brief Sets the RGB led to show the colors of the rainbow.
 *
 *  Rainbow colors are shown using the setRed, setGreen, setBlue, etc functions.
 *
 *  _rainbowStage and _rainbowStartTime are class variables used as state
 *  holders.
 *  _rainbowStage stores the stage of the method (initialization, red, yellow,
 *  green, aquamarine, etc).
 *  _rainbowStartTime stores the start millis() of the initialization stage.
 *  This is used for detecting when to change colors.
 *
 *  @return void.
 */
void LedController::setRainbow() {
    // detects _rainbowStage stage
    switch (_rainbowStage) {
        // initialization stage
        case 0:
            _rainbowStartTime = millis();
            _rainbowStage ++;
            break;

        // display red stage
        case 1:
            LedController::setRed();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display yellow stage
        case 2:
            LedController::setYellow();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display green stage
        case 3:
            LedController::setGreen();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display aquamarine stage
        case 4:
            LedController::setAquamarine();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display blue stage
        case 5:
            LedController::setBlue();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display purple stage
        case 6:
            LedController::setPurple();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage ++;
            }
            break;

        // display white stage
        case 7:
            LedController::setWhite();

            if (millis() - _rainbowStartTime > _colorTimer * _rainbowStage) {
                _rainbowStage = 0;

                // removes color after setRainbow() finishes
                LedController::setBlack();
            }
            break;
    }

    // clears other stage data
    _discoStage = 0;
}
