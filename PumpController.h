/** @project PumpController v1.0
 *  @author Collin Chiang
 *  @date 7/27/2021
 *  @description Object for controlling the pumps on the SeaJelly.
 */

#ifndef PumpController_h
#define PumpController_h

#include <Arduino.h>


class PumpController {
    public:
        // class constructor
        PumpController(int pPin);

        // accessor methods
        int getPumpPower();
        void setPumpPower(int pPower);

    private:
        // pin information
        int _pPin;

        // stores pump power
        int _pPower;
};


#endif
