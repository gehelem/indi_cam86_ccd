/*
   INDI Developers Manual
   Tutorial #3

   "Cam86 Driver"

   We develop a Cam86 driver.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simpleccd.cpp
    \brief Construct a basic INDI CCD device that simulates exposure & temperature settings. It also generates a random pattern and uploads it as a FITS file.
    \author Jasem Mutlaq

    \example simpleccd.cpp
    A Cam86 device that can capture images and control temperature. It returns a FITS image to the client. To build drivers for complex CCDs, please
    refer to the INDI Generic CCD driver template in INDI SVN (under 3rdparty).
*/

#include <sys/time.h>
#include <memory>
#include <math.h>
#include <unistd.h>
#include "cam86_ccd.h"
#include "libcam86.h"

const int POLLMS           = 300;       /* Polling interval 500 ms */
const int MAX_CCD_TEMP     = 45;		/* Max CCD temperature */
const int MIN_CCD_TEMP	   = -55;		/* Min CCD temperature */
const float TEMP_THRESHOLD = .25;		/* Differential temperature threshold (C)*/
#define TEMPERATURE_UPDATE_FREQ 40      /* Update every 40 POLLMS ~ 20 second */

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value
#define LIBFTDI_TAB     "LibFTDI"

std::unique_ptr<Cam86CCD> simpleCCD ( new Cam86CCD() );

void ISGetProperties ( const char *dev )
{
  simpleCCD->ISGetProperties ( dev );
}

void ISNewSwitch ( const char *dev, const char *name, ISState *states, char *names[], int num )
{
  simpleCCD->ISNewSwitch ( dev, name, states, names, num );
}

void ISNewText (	const char *dev, const char *name, char *texts[], char *names[], int num )
{
  simpleCCD->ISNewText ( dev, name, texts, names, num );
}

void ISNewNumber ( const char *dev, const char *name, double values[], char *names[], int num )
{
  simpleCCD->ISNewNumber ( dev, name, values, names, num );
}

void ISNewBLOB ( const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n )
{
  INDI_UNUSED ( dev );
  INDI_UNUSED ( name );
  INDI_UNUSED ( sizes );
  INDI_UNUSED ( blobsizes );
  INDI_UNUSED ( blobs );
  INDI_UNUSED ( formats );
  INDI_UNUSED ( names );
  INDI_UNUSED ( n );
}

void ISSnoopDevice ( XMLEle *root )
{
  simpleCCD->ISSnoopDevice ( root );
}

Cam86CCD::Cam86CCD()
{
  InExposure = false;
}

/*******************************************************************************
** Client is asking us to set a new number
*******************************************************************************/
bool Cam86CCD::ISNewNumber ( const char *dev, const char *name,
                             double values[], char *names[], int n )
{
  if ( !strcmp ( dev, getDeviceName() ) )
    {
      if ( !strcmp ( name, GainNP.name ) )
        {
          IUUpdateNumber ( &GainNP, values, names, n );
          GainNP.s = IPS_OK;
          IDSetNumber ( &GainNP, NULL );
          cameraSetGain ( GainN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set gain = %d", ( int ) GainN[0].value );
          return true;
        }

      if ( !strcmp ( name, OffsetNP.name ) )
        {
          IUUpdateNumber ( &OffsetNP, values, names, n );
          OffsetNP.s = IPS_OK;
          IDSetNumber ( &OffsetNP, NULL );
          cameraSetOffset ( OffsetN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set offset = %d", ( int ) OffsetN[0].value );
          return true;
        }

      /*if ( !strcmp ( name, BaudrateNP.name ) )
        {
          IUUpdateNumber ( &BaudrateNP, values, names, n );
          BaudrateNP.s = IPS_OK;
          IDSetNumber ( &BaudrateNP, NULL );
          cameraSetBaudrate ( BaudrateN[0].value );
          //IDMessage ( getDeviceName(), "Cam86 set baudrate = %d", ( int ) BaudrateN[0].value );
          return true;
        }*/

      if ( !strcmp ( name, BaudrateANP.name ) )
        {
          IUUpdateNumber ( &BaudrateANP, values, names, n );
          BaudrateANP.s = IPS_OK;
          IDSetNumber ( &BaudrateANP, NULL );
          cameraSetBaudrateA ( BaudrateAN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set baudrate A = %d", ( int ) BaudrateAN[0].value );
          return true;
        }

      if ( !strcmp ( name, BaudrateBNP.name ) )
        {
          IUUpdateNumber ( &BaudrateBNP, values, names, n );
          BaudrateBNP.s = IPS_OK;
          IDSetNumber ( &BaudrateBNP, NULL );
          cameraSetBaudrateB ( BaudrateBN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set baudrate B = %d", ( int ) BaudrateBN[0].value );
          return true;
        }
      if ( !strcmp ( name, LibftditimerANP.name ) )
        {
          IUUpdateNumber ( &LibftditimerANP, values, names, n );
          LibftditimerANP.s = IPS_OK;
          IDSetNumber ( &LibftditimerANP, NULL );
          cameraSetLibftdiTimerAR ( LibftditimerAN[0].value );
          cameraSetLibftdiTimerAW ( LibftditimerAN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set libftdi timerA = %d", ( int ) LibftditimerAN[0].value );
          return true;
        }

      if ( !strcmp ( name, LibftdilatencyANP.name ) )
        {
          IUUpdateNumber ( &LibftdilatencyANP, values, names, n );
          LibftdilatencyANP.s = IPS_OK;
          IDSetNumber ( &LibftdilatencyANP, NULL );
          cameraSetLibftdiLatA ( LibftdilatencyAN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set libftdi latencyA = %d", ( int ) LibftdilatencyAN[0].value );
          return true;
        }
      if ( !strcmp ( name, LibftditimerBNP.name ) )
        {
          IUUpdateNumber ( &LibftditimerBNP, values, names, n );
          LibftditimerBNP.s = IPS_OK;
          IDSetNumber ( &LibftditimerBNP, NULL );
          cameraSetLibftdiTimerBR ( LibftditimerBN[0].value );
          cameraSetLibftdiTimerBW ( LibftditimerBN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set libftdi timerB = %d", ( int ) LibftditimerBN[0].value );
          return true;
        }

      if ( !strcmp ( name, LibftdilatencyBNP.name ) )
        {
          IUUpdateNumber ( &LibftdilatencyBNP, values, names, n );
          LibftdilatencyBNP.s = IPS_OK;
          IDSetNumber ( &LibftdilatencyBNP, NULL );
          cameraSetLibftdiLatA ( LibftdilatencyBN[0].value );
          IDMessage ( getDeviceName(), "Cam86 set libftdi latencyA = %d", ( int ) LibftdilatencyBN[0].value );
          return true;
        }

    }

  // If we didn't process anything above, let the parent handle it.
  return INDI::CCD::ISNewNumber ( dev,name,values,names,n );
}

/*******************************************************************************
** Client is asking us to set a new switch
*******************************************************************************/

bool Cam86CCD::ISNewSwitch ( const char *dev, const char *name,
                             ISState *states, char *names[], int n )
{
  return INDI::CCD::ISNewSwitch ( dev, name, states, names,  n );
}


/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool Cam86CCD::Connect()
{


  // Let's set a timer that checks teleCCDs status every POLLMS milliseconds.
  SetTimer ( POLLMS );
  //cameraSetBaudrate(80);
  cameraConnect();
  cameraSetBaudrateA ( BRA );
  cameraSetBaudrateB ( BRB );
  usleep(500*1000);
  cameraSetOffset ( -20 );
  cameraSetGain ( 0 );
  IDMessage ( getDeviceName(), "Cam86 connected successfully!\n" );
  cameraSetCoolingStartingPowerPercentage(60);
  cameraSetCoolingMaximumPowerPercentage(100);
  cameraSetReadingTime(10);
  cameraSetCoolerDuringReading(true);
  //CameraSetTemp(0);
  //CameraCoolingOn();

  return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool Cam86CCD::Disconnect()
{

  cameraDisconnect();
  return true;
  IDMessage ( getDeviceName(), "Cam86 disconnected successfully!\n" );
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * Cam86CCD::getDefaultName()
{
  return "Cam86";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool Cam86CCD::initProperties()
{
  // Must init parent properties first!
  INDI::CCD::initProperties();

  /*const short minGain = 0;
  const short maxGain = 63;
  const short minOffset = -127;
  const short maxOffset = 127;
  const short minBaudrate = 80;
  const short maxBaudrate = 240;*/

  /* Add Gain number property (gs) */
  IUFillNumber ( GainN, "GAIN", "Gain", "%g", 0, 63, 1, 0 );
  IUFillNumberVector ( &GainNP, GainN, 1, getDeviceName(),"GAIN",
                       "Gain", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Offset number property (gs) */
  IUFillNumber ( OffsetN, "OFFSET", "Offset", "%g", -127, 127, 1, -20 );
  IUFillNumberVector ( &OffsetNP, OffsetN, 1, getDeviceName(),"OFFSET",
                       "Offset", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Baudrate number property (gs) */
//  IUFillNumber ( BaudrateN, "BAUDRATE", "Baudrate", "%g", 5, 150, 5, 20 );
//  IUFillNumberVector ( &BaudrateNP, BaudrateN, 1, getDeviceName(),"BAUDRATE",
//                       "Baudrate", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Baudrate A number property (gs) */
  IUFillNumber ( BaudrateAN, "BAUDRATEA", "BaudrateA", "%g", 5, 150, 5, 20 );
  IUFillNumberVector ( &BaudrateANP, BaudrateAN, 1, getDeviceName(),"BAUDRATEA",
                       "BaudrateA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Baudrate B number property (gs) */
  IUFillNumber ( BaudrateBN, "BAUDRATEB", "BaudrateB", "%g", 5, 150, 5, 5 );
  IUFillNumberVector ( &BaudrateBNP, BaudrateBN, 1, getDeviceName(),"BAUDRATEB",
                       "BaudrateB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Latency number property (gs) */
  IUFillNumber ( LibftdilatencyAN, "LATENCYA", "LatencyA", "%g", 1, 50, 1, CAM86_LATENCYA );
  IUFillNumberVector ( &LibftdilatencyANP, LibftdilatencyAN, 1, getDeviceName(),"LATENCYA",
                       "LatencyA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add timers number property (gs) */
  IUFillNumber ( LibftditimerAN, "TIMERA", "TimerA", "%g", 10, 20000, 10, CAM86_TIMERA );
  IUFillNumberVector ( &LibftditimerANP, LibftditimerAN, 1, getDeviceName(),"TIMERA",
                       "TimerA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add Latency number property (gs) */
  IUFillNumber ( LibftdilatencyBN, "LATENCYB", "LatencyB", "%g", 1, 50, 1, CAM86_LATENCYB );
  IUFillNumberVector ( &LibftdilatencyBNP, LibftdilatencyBN, 1, getDeviceName(),"LATENCYB",
                       "LatencyB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

  /* Add timers number property (gs) */
  IUFillNumber ( LibftditimerBN, "TIMERB", "TimerB", "%g",  10, 20000, 10, CAM86_TIMERB );
  IUFillNumberVector ( &LibftditimerBNP, LibftditimerBN, 1, getDeviceName(),"TIMERB",
                       "TimerB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );



  // We set the CCD capabilities
  uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_BAYER | CCD_HAS_COOLER;
  SetCCDCapability ( cap );
  IUSaveText ( &BayerT[2], "GRBG" );
  // Add Debug, Simulator, and Configuration controls
  addAuxControls();

  return true;

}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool Cam86CCD::updateProperties()
{
  //cameraSetBaudrate(80);
  // Call parent update properties first
  INDI::CCD::updateProperties();

  if ( isConnected() )
    {
      // Let's get parameters now from CCD
      setupParams();

      // Start the timer
      SetTimer ( POLLMS );
      defineNumber ( &GainNP );
      defineNumber ( &OffsetNP );
      //defineNumber ( &BaudrateNP );
      defineNumber ( &BaudrateANP );
      defineNumber ( &BaudrateBNP );
      defineNumber ( &LibftditimerANP );
      defineNumber ( &LibftdilatencyANP );
      defineNumber ( &LibftditimerBNP );
      defineNumber ( &LibftdilatencyBNP );
    }
  else
    {
      deleteProperty ( GainNP.name );
      deleteProperty ( OffsetNP.name );
      //deleteProperty ( BaudrateNP.name );
      deleteProperty ( BaudrateANP.name );
      deleteProperty ( BaudrateBNP.name );
      deleteProperty ( LibftditimerANP.name );
      deleteProperty ( LibftdilatencyANP.name );
      deleteProperty ( LibftditimerBNP.name );
      deleteProperty ( LibftdilatencyBNP.name );
    }

  return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void Cam86CCD::setupParams()
{
  // Our CCD is an 8 bit CCD, 1280x1024 resolution, with 5.4um square pixels.
  SetCCDParams ( 3000, 2000, 16, 7.8, 7.8 );

  // Let's calculate how much memory we need for the primary CCD buffer
  int nbuf;
  nbuf=PrimaryCCD.getXRes() *PrimaryCCD.getYRes() * PrimaryCCD.getBPP() /8;
  nbuf+=512;                      //  leave a little extra at the end
  PrimaryCCD.setFrameBufferSize ( nbuf );
}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool Cam86CCD::StartExposure ( float duration )
{


  ExposureRequest=duration;
  IDMessage ( getDeviceName(), "Start exposure %g",duration );
  // Since we have only have one CCD with one chip, we set the exposure duration of the primary CCD
  PrimaryCCD.setExposureDuration ( duration );
  //cameraStartExposure(1,0,0,3000,2000, duration,true);
  //int r = cameraStartExposure ( PrimaryCCD.getBinX(),PrimaryCCD.getSubX(),PrimaryCCD.getSubY(),PrimaryCCD.getSubW(),PrimaryCCD.getSubH(), duration, true );    //int r = cameraStartExposure(1,0,0,3000,2000, 0.4, true);
  int r = cameraStartExposure ( PrimaryCCD.getBinX()-1,PrimaryCCD.getSubX(),PrimaryCCD.getSubY(),PrimaryCCD.getSubW(),PrimaryCCD.getSubH(), duration, true );   //int r = cameraStartExposure(1,0,0,3000,2000, 0.4, true);
  gettimeofday ( &ExpStart,NULL );


  InExposure=true;

  // We're done
  return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool Cam86CCD::AbortExposure()
{
  InExposure = false;
  cameraStopExposure;
  return true;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int Cam86CCD::SetTemperature ( double temperature )
{
  TemperatureRequest = temperature;

  // 0 means it will take a while to change the temperature
  return 0;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float Cam86CCD::CalcTimeLeft()
{
  double timesince;
  double timeleft;
  struct timeval now;
  gettimeofday ( &now,NULL );

  timesince= ( double ) ( now.tv_sec * 1000.0 + now.tv_usec/1000 ) - ( double ) ( ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec/1000 );
  timesince=timesince/1000;

  timeleft=ExposureRequest-timesince;
  return timeleft;
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void Cam86CCD::TimerHit()
{
  long timeleft;

  if ( isConnected() == false )
    return;  //  No need to reset timer if we are not connected anymore

  if ( InExposure )
    {
      timeleft=CalcTimeLeft();

      // Less than a 0.1 second away from exposure completion
      // This is an over simplified timing method, check CCDSimulator and simpleCCD for better timing checks
      if ( timeleft < 0.1 )
        {
          /* We're done exposing */
          IDMessage ( getDeviceName(), "Exposure done, downloading image..." );

          // Set exposure left to zero
          PrimaryCCD.setExposureLeft ( 0 );

          // We're no longer exposing...
          InExposure = false;

          /* grab and save image */
          grabImage();

        }
      else
        // Just update time left in client
        PrimaryCCD.setExposureLeft ( timeleft );

    }

  // TemperatureNP is defined in INDI::CCD
    //if ((TemperatureUpdateCounter++ > TEMPERATURE_UPDATE_FREQ) && !InExposure)
    if ((TemperatureUpdateCounter++ > TEMPERATURE_UPDATE_FREQ) )
    {
	    TemperatureUpdateCounter = 0;
	    currentCCDTemperature = CameraGetTemp();
	    float tempDHT=CameraGetTempDHT();
	    float HUM=CameraGetHum();
	    float coolerpower = cameraGetCoolerPower();
	
            const double a = 17.27;
            const double b = 237.7;
            double dewpoint = 0; 

            double c = log(HUM / 100) + a * tempDHT / (b + tempDHT);
            dewpoint = b * c / (a - c);
    
	    
	    IDMessage ( getDeviceName(), "CCD %.2f°C Ext %.2f Hum %.2f DP %.1f CoolerPower %.2f\n" , currentCCDTemperature,tempDHT,HUM,dewpoint,coolerpower); 
	   // TemperatureN[0].value =currentCCDTemperature;
	    
    }
  switch ( TemperatureNP.s )
    {
    case IPS_IDLE:
    case IPS_OK:
	    if (fabs(currentCCDTemperature - TemperatureN[0].value) > TEMP_THRESHOLD)
	    {
            IDSetNumber(&TemperatureNP, NULL);
	    TemperatureNP.s = IPS_BUSY;
	    }
      break;

    case IPS_BUSY:
      /* If target temperature is higher, then increase current CCD temperature */
      if ( currentCCDTemperature < TemperatureRequest )
      {
        //currentCCDTemperature++;
	      CameraCoolingOff();
      }
      /* If target temperature is lower, then decrese current CCD temperature */
      else if ( currentCCDTemperature > TemperatureRequest ) 
      {
        //currentCCDTemperature--;
	      CameraCoolingOn();
      }
      /* If they're equal, stop updating */
      else
        {
          TemperatureNP.s = IPS_OK;
          IDSetNumber ( &TemperatureNP, "Target temperature reached." );
	  CameraCoolingOff();
          break;
        }

      IDSetNumber ( &TemperatureNP, NULL );

      break;

    case IPS_ALERT:
      break;
    }

  SetTimer ( POLLMS );
  return;
}

/**************************************************************************************
** Create a random image and return it to client
***************************************************************************************/
void Cam86CCD::grabImage()
{
  // Let's get a pointer to the frame buffer
  uint8_t * image = PrimaryCCD.getFrameBuffer();
  int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX()   * ( PrimaryCCD.getBPP() / 8 );
  int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();
  //int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX() * (PrimaryCCD.getBPP() / 8);
  //int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY() * (PrimaryCCD.getBPP() / 8);


  IDMessage ( getDeviceName(), "grabimage width=%d height=%d BPP=%d\n", width/2, height, PrimaryCCD.getBPP() );

  // Fill buffer with random pattern
  while ( !cameraGetImageReady() ); // waiting image

  if ( PrimaryCCD.getBinX() ==1 )
    {
      for ( int j=PrimaryCCD.getSubY(); j < height + PrimaryCCD.getSubY(); j++ )
        for ( int i=PrimaryCCD.getSubX(); i < ( PrimaryCCD.getSubX() +width ) /2; i++ )
          {
            uint16_t pix = cameraGetImageXY ( i,j );
            uint8_t hibyte = ( pix & 0xff00 ) >> 8;
            uint8_t lobyte = ( pix & 0xff );
            image[2*i+  j*width] = hibyte;
            image[2*i+1+j*width] = lobyte;
          };
    }
  else
    {
      for ( int j=0; j < height ; j++ )
        for ( int i=0; i < width/2; i++ )
          {
            uint16_t pix = cameraGetImageXY ( 2*i,2*j );
            uint8_t hibyte = ( pix & 0xff00 ) >> 8;
            uint8_t lobyte = ( pix & 0xff );
            image[2*i+  j*width] = hibyte;
            image[2*i+1+j*width] = lobyte;
          };
    };


  IDMessage ( getDeviceName(), "Download complete.\n" );

  // Let INDI::CCD know we're done filling the image buffer
  ExposureComplete ( &PrimaryCCD );
}


/*

int main(void)
{
    cameraConnect();
    cameraSetBaudrate(80);
    cameraSetOffset(0);
    cameraSetGain(0);
    int r = cameraStartExposure(1,0,0,3000,2000, 1, false);
    while (!cameraGetImageReady())
//        fprintf(stdout,"wait\n");
      ; // waiting image
    //for (int i=0; i < 10 ; i++)
    //  for (int j=0; j < 10; j++)
    //      fprintf(stdout,"img %d/%d:%d\n",i,j,cameraGetImage(i,j));
    cameraDisconnect();

}

*/

