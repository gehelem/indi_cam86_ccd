/*
    Cam86 DIY CCD Camera Library
    See :
    http://www.astroclub.kiev.ua/forum/index.php?topic=28929.0
    http://www.cloudynights.com/topic/497530-diy-astro-ccd-16-bit-color-6mpx-camera/
    http://astroccd.org/

    Copyright (C) 2017 - Gilles Le Maréchal - Gilmanov Rim - Sergiy Vakulenko - Michael "Toups"

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __LIBCAM86_H__
#define __LIBCAM86_H__

#define BRA 20
#define BRB 5
#define CAM86_LATENCYA  1
#define CAM86_LATENCYB  1
#define CAM86_TIMERA    30
#define CAM86_TIMERB    10

#ifdef __cplusplus
extern "C" {
#endif

bool cameraConnect ( void );
bool cameraDisconnect ( void );
bool cameraSetGain ( int val );
bool cameraSetOffset ( int val );
int  cameraStartExposure ( int bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light );
bool cameraStopExposure ( void );
bool CameraSetTemp ( int temp );
float CameraGetTemp ( void );
bool CameraCoolingOn ( void );
bool CameraCoolingOff ( void );
bool cameraGetImageReady ( void );
uint16_t cameraGetImage ( int i, int j );
bool cameraSetBaudrate ( int val );
bool cameraSetBaudrateA ( int val );
bool cameraSetBaudrateB ( int val );
bool cameraSetLibftdiTimerAR ( int tt );
bool cameraSetLibftdiTimerAW ( int tt );
bool cameraSetLibftdiTimerBR ( int tt );
bool cameraSetLibftdiTimerBW ( int tt );
bool cameraSetLibftdiLatA ( int ll );
bool cameraSetLibftdiLatB ( int ll );

#ifdef __cplusplus
}
#endif

#endif
