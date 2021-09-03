/** @project LedRGB v1.0
 *  @author Collin Chiang
 *  @date 7/26/2021
 *  @description Object for controlling the RGB led on the SeaJelly.
 */

#ifndef LedController_h
#define LedController_h

#include <Arduino.h>


class LedController {
    public:
        // class constructor
        LedController(int rPin, int gPin, int bPin);

        // accessor methods
        unsigned long getColorTimer();
        void setColorTimer(unsigned long colorTimer);

        // changes RGB led color
        void setColor(byte rByte, byte gByte, byte bByte);
        void setRed();
        void setGreen();
        void setBlue();
        void setYellow();
        void setAquamarine();
        void setPurple();
        void setBlack();
        void setWhite();

        // changes RGB to flash sets of colors
        void setDisco();
        void setRainbow();

    private:
        // pin information
        int _rPin;
        int _gPin;
        int _bPin;

        // times color changing intervals
        unsigned long _colorTimer;

        // method stage and timer variables
        int _discoStage;
        unsigned long _discoStartTime;

        int _rainbowStage;
        unsigned long _rainbowStartTime;
};


#endif
