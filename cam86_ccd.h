#ifndef Cam86CCD_H
#define Cam86CCD_H

/*
   INDI Developers Manual
   Tutorial #3

   "Simple CCD Driver"

   We develop a simple CCD driver.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simpleccd.h
    \brief Construct a basic INDI CCD device that simulates exposure & temperature settings. It also generates a random pattern and uploads it as a FITS file.
    \author Jasem Mutlaq

    \example simpleccd.h
    A simple CCD device that can capture images and control temperature. It returns a FITS image to the client. To build drivers for complex CCDs, please
    refer to the INDI Generic CCD driver template in INDI SVN (under 3rdparty).
*/


#include <indiccd.h>

class Cam86CCD : public INDI::CCD {
public:
    Cam86CCD();

    bool ISNewNumber ( const char *dev, const char *name, double values[], char *names[], int n );
    bool ISNewSwitch ( const char *dev, const char *name, ISState *states, char *names[], int n );


protected:
    // General device functions
    bool Connect();
    bool Disconnect();
    const char *getDefaultName();
    bool initProperties();
    bool updateProperties();

    // CCD specific functions
    bool StartExposure ( float duration );
    bool AbortExposure();
    int SetTemperature ( double temperature );
    void TimerHit();

private:
    // Utility functions
    float CalcTimeLeft();
    void  setupParams();
    void  grabImage();
    void activateCooler(bool enable);

    // Are we exposing?
    bool InExposure;
    // Struct to keep timing
    struct timeval ExpStart;

    INumber GainN[1];
    INumberVectorProperty GainNP;

    INumber OffsetN[1];
    INumberVectorProperty OffsetNP;

//     INumber BaudrateN[1];
//     INumberVectorProperty BaudrateNP;

    INumber BaudrateAN[1];
    INumberVectorProperty BaudrateANP;

    INumber BaudrateBN[1];
    INumberVectorProperty BaudrateBNP;

    INumber LibftditimerAN[1];
    INumberVectorProperty LibftditimerANP;

    INumber LibftdilatencyAN[1];
    INumberVectorProperty LibftdilatencyANP;

    INumber LibftditimerBN[1];
    INumberVectorProperty LibftditimerBNP;

    INumber LibftdilatencyBN[1];
    INumberVectorProperty LibftdilatencyBNP;

    INumber CoolerN[1];
    INumberVectorProperty CoolerNP;

    ISwitch CoolerS[2];
    ISwitchVectorProperty CoolerSP;

    INumber CoolerStartN[1];
    INumberVectorProperty CoolerStartNP;

    INumber CoolerMaxN[1];
    INumberVectorProperty CoolerMaxNP;

    float ExposureRequest;
    float TemperatureRequest;
    int   TemperatureUpdateCounter;
    float currentCCDTemperature;
    double targetTemperature;
    int   timerID;

};

#endif // Cam86CCD_H
