/** @project SwimController v1.0
 *  @author Collin Chiang
 *  @date 7/27/2021
 *  @description Object for controlling the swimming functions on the SeaJelly.
 */

#ifndef SwimController_h
#define SwimController_h

#include <Arduino.h>
#include "PumpController.h"


class SwimController {
    public:
        struct Temperature {
            float degF, degC;
        };

        struct BatteryData {
            float voltage;
            int level;
        };

        struct Vector3 {
            float x, y, z;
        };

        // class constructor
        SwimController(PumpController &rPump, PumpController &bPump);

        // accessor method
        unsigned int getPowerCycleTimer();
        unsigned int getRestCycleTimer();
        float getDepthTolerance();
        float getTurnTolerance();
        void setPowerCycleTimer(unsigned int powerCycleTimer);
        void setRestCycleTimer(unsigned int restCycleTimer);
        void setDepthTolerance(float depthTolerance);
        void setTurnTolerance(float turnTolerance);

        // swim functionalities (basic)
        void rest();
        void swim(int pumpPower1, int pumpPower2);
        void turnLeft();
        void turnLeft(float turnAngle, Vector3 rotationAngle);
        void turnRight();
        void turnRight(float turnAngle, Vector3 rotationAngle);

        // swim functionalities (advanced)
        void flip(Vector3 rotationAngle);
        void stablize(Vector3 rotationAngle);

    private:
        // controller information
        PumpController _rPump;
        PumpController _bPump;

        // swim cycle timers
        unsigned int _powerCycleTimer;
        unsigned int _restCycleTimer;

        // swim tolerance variables
        float _depthTolerance;
        float _turnTolerance;

        // method stage and timer variables
        int _swimStage;
        unsigned int _swimStartTime;
};


#endif
