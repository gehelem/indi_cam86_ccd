#ifndef __LIBCAM86_H__
#define __LIBCAM86_H__

#ifdef __cplusplus
extern "C" {
#endif

bool cameraConnect(void);
bool cameraDisconnect(void); 
bool cameraSetGain (int val);
bool cameraSetOffset (int val);
int  cameraStartExposure(int bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light);
bool cameraStopExposure(void);
bool CameraSetTemp(int temp);
uint16_t CameraGetTemp(void);
bool CameraCoolingOn(void);
bool CameraCoolingOff(void);
bool cameraGetImageReady(void);
uint16_t cameraGetImage(int i, int j);
bool cameraSetBaudrate ( int val );


#ifdef __cplusplus
}
#endif

#endif
