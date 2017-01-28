{
*********************************************************************** 
This file is the original Delphi library written by Gilmanov Rim
All comments have been "google translated" in english 
***********************************************************************
}
unit cam86;

interface

uses Classes, SysUtils, MyD2XX, MMSystem, Windows, SyncObjs, ExtCtrls;

type
      {Class for reading thread}
      posl = class(TThread)
      private
      { Private declarations }
      protected
       procedure Execute; override;
      end;

const CameraWidth  = 3000;    //image width
      CameraHeight = 2000;    //image height
      portfirst = $11;        // Initial value on the output port BDBUS
      xccd = 1500;
      yccd = 1000;
      spusb = 20000;          //bitbang velocity

      TemperatureOffset = 1280;
      MinErrTemp = -120.0;
      MaxErrTemp = 120.0;

{GLobal variables}
var   IsConnected : boolean = false;        //variable-flag indicates the status of the connection with the camera
      adress : integer;                     //pointer to the current address in the output buffer FT2232HL
      mBin : integer;                       //?inning,
      mImageReady : boolean = false;        //variable-flag displays readiness for reading frame
      mCameraState : integer = 0;           //Variable-state camera  0 - ready 1 - longexp 2 - read
      ExposureTimer : integer;              //exposure timer
      co: posl;                            // Variable for the second stream (image reading)
      bufim:array[0..CameraHeight*CameraWidth-1] of word;       // Buffer array image for operations
      bufi2:array[0..3*CameraHeight*CameraWidth-1] of word;     // Buffer array RGB image
      mYn,mdeltY:integer;                   //start reading and the number of the rows
      //mXn,mdeltX:integer;                   //start reading and number of the columns
      zatv:byte;
      kolbyte:integer;
      eexp:integer;

      indval:integer;

      siin:  array[0..3] of byte;
      siout: word;

      //cached values
      sensorTempCache : Double;

procedure Spi_comm(comm:byte;param:word);      
function CameraConnect ()      : WordBool;
function CameraDisConnect ()   : WordBool;
function Qbuf()                : integer;
function Rval()                : integer;
function CameraSetGain (val : integer) : WordBool;
function CameraSetOffset (val : integer) : WordBool;
function CameraStartExposure (Bin,StartX,StartY,NumX,NumY : integer; Duration : double; light : WordBool) : WordBool;
function CameraStopExposure : WordBool;
function CameraSetTemp(temp:double): WordBool;
function CameraGetTemp ()      : Double;
function CameraCoolingOn ()    : WordBool;
function CameraCoolingOff ()   : WordBool;
function CameraReadingTime(val: integer)  : WordBool;

implementation

{A little explanation with FT2232LH.
Always use this technique:
1. First, the buffer is filled and the initial bytes (required pulse sequence BDBUS output port).
This pointer is incremented adress

2. Next, the whole array is passed to the output of the command: n: = Write_USB_Device_Buffer (FT_CAM8B, adress);
Wonderful chip FT2232HL honestly without delay all transfers to your port BDBUS. Transfer 1 byte at the same time takes 65 ns.
Time working out the next command n: = Write_USB_Device_Buffer (FT_CAM8B, adress) depends on the workload of the OSes and is not controlled
us. Therefore, the critical sequence of pulses need to fill the whole, rather than pass on the queue.
Fortunately Programnyj driver buffer it allows (in the program up to 24 MB!) To do this, change the text D2XX.pas, I called him MyD2XX.pas}

function Qbuf():integer;
begin
 Get_USB_Device_QueueStatus(FT_HANDLEA);
 result:=FT_Q_Bytes;
end;

function Rval():integer;
begin
 result:=indval;
end;

procedure sspi;
var
i,j:integer;
b:byte;
n:word;
begin
n:=100;
FillChar(FT_Out_Buffer,n,portfirst);
for j:=0 to 2 do
begin
b:=siin[j];
For i:= 0 to 7 do
begin
 inc(FT_Out_Buffer[2*i+1+16*j],$20);
 if (b and $80) = $80 then begin inc(FT_Out_Buffer[2*i+16*j],$80);inc(FT_Out_Buffer[2*i+1+16*j],$80);end;    //B3
 b:=b*2;
end;
end;
Write_USB_Device_Buffer(FT_HANDLEB,@FT_Out_Buffer,n);
end;

procedure sspo;
var i:integer;
b:word;
n:word;
begin
 n:=100;
 Read_USB_Device_Buffer(FT_HANDLEB,n);
    b:=0;
     for i:=0 to 15 do
      begin
       b:=b*2;
       if (FT_In_Buffer[i+1+8] and $40) <> 0 then inc(b);        //B4
      end;
      siout:=b;
end;

procedure Spi_comm(comm:byte;param:word);
begin
Purge_USB_Device_In(FT_HANDLEB);
Purge_USB_Device_Out(FT_HANDLEB);
siin[0]:=comm;
siin[1]:=hi(param);
siin[2]:=lo(param);
sspi;
sspo;
sleep(20);
end;

procedure ComRead;
begin
  co:=posl.Create(true);
  co.FreeOnTerminate:=true;
  co.Priority:=tpNormal;//Highest;//Lower;//st;//r;//Normal;
  co.Resume;
end;

procedure posl.Execute;                                     // Array itself actually reading through ADBUS port
{Contraption FT2232HL converting the read buffer to the buffer array image
due to the nature AD9822 read the high byte first, then low, and vice versa in delphi.
Use the type integer32, instead word16 due to overflow in subsequent operations}
var
x,y:integer;
begin
 Read_USB_Device_Buffer(FT_HANDLEA,kolbyte);
 if mBin = 0 then
 begin
 for y:= 0 to mdeltY-1 do
  begin
     for x:=0 to 1499 do
      begin
       bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[4*x+4+y*6004]);
       bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[4*x+5+y*6004]);
       bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[4*x+6+y*6004]);
       bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[4*x+7+y*6004]);
      end;
  end;
  end        else
  begin
  for y:= 0 to mdeltY-1 do
  begin
     for x:=0 to 1498 do
      begin
       bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
       bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
       bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
       bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
      end;
     x:=1499;
     bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
     bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
     bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
     bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
  end;
  end;
  mCameraState:=0;
  mImageReady := true;
end;

{Filling the output buffer array for transmission and placing byte val at the address adr-chip AD9822.
 The transfer is in sequential code.}
procedure AD9822(adr:byte;val:word);
const
kol = 64;
var
dan:array[0..kol-1] of byte;
i:integer;
begin
 fillchar(dan,kol,portfirst);                                   // Fill the array initial value on the output port BDBUS
 for i:=1 to 32 do dan[i]:=dan[i] and $fe;
 for i:=0 to 15 do dan[2*i+2]:=dan[2*i+2] + 2;
 if (adr and 4) = 4 then begin dan[3]:=dan[3]+4;dan[4]:=dan[4]+4;end;
 if (adr and 2) = 2 then begin dan[5]:=dan[5]+4;dan[6]:=dan[6]+4;end;
 if (adr and 1) = 1 then begin dan[7]:=dan[7]+4;dan[8]:=dan[8]+4;end;

 if (val and 256) = 256 then begin dan[15]:=dan[15]+4;dan[16]:=dan[16]+4;end;
 if (val and 128) = 128 then begin dan[17]:=dan[17]+4;dan[18]:=dan[18]+4;end;
 if (val and 64) = 64 then begin dan[19]:=dan[19]+4;dan[20]:=dan[20]+4;end;
 if (val and 32) = 32 then begin dan[21]:=dan[21]+4;dan[22]:=dan[22]+4;end;
 if (val and 16) = 16 then begin dan[23]:=dan[23]+4;dan[24]:=dan[24]+4;end;
 if (val and 8) = 8 then begin dan[25]:=dan[25]+4;dan[26]:=dan[26]+4;end;
 if (val and 4) = 4 then begin dan[27]:=dan[27]+4;dan[28]:=dan[28]+4;end;
 if (val and 2) = 2 then begin dan[29]:=dan[29]+4;dan[30]:=dan[30]+4;end;
 if (val and 1) = 1 then begin dan[31]:=dan[31]+4;dan[32]:=dan[32]+4;end;

 Write_USB_Device_Buffer(FT_HANDLEB,@dan, kol);
end;

{Use 2 modes:
 1.Tsvetnoy without binning.
 2.CH / B with 2 * 2 binning.
 Feature ICX453 matrix is that the horizontal register has twice the capacity and
 at one step in a horizontal vertical shift register "falls" just a couple of lines,
 so the number of rows for the two modes are similar.
}

{Filling the output buffer array and the actual frame itself in the read operation mode 1}
procedure readframe;
begin
 mCameraState := 2;
 mImageReady:=false;
 Purge_USB_Device_IN(FT_HANDLEA);
 Purge_USB_Device_OUT(FT_HANDLEB);
 comread;
 Spi_comm($1b,0);//$ffff);
end;

{Set camera gain, return bool result}
function CameraSetGain (val : integer) : WordBool;// stdcall; export;
begin
 AD9822(3,val);           //AD9822 gain
 Result :=true;
end;

{Set camera offset, return bool result}
function CameraSetOffset (val : integer) : WordBool;
var x : integer;
begin
 x:=abs(2*val);
 if val < 0 then x:=x+256;
 AD9822(6,x);                       //offset AD9822
 Result :=true;
end;

function CameraReadingTime(val: integer)  : WordBool;
begin
 Spi_comm($eb, val);
end;

{Connect camera, return bool result}
{Survey attached devices and initialize AD9822}
function CameraConnect () : WordBool;
var  FT_flag, FT_OP_flag : boolean;
I : Integer;
begin
 FT_flag:=false;
 FT_Enable_Error_Report:=false;
 sensorTempCache := 0;
 GetFTDeviceCount;
 I := FT_Device_Count-1;
 while I >= 0 do
  begin
   GetFTDeviceSerialNo(I);
   if pos('CAM86',FT_Device_String) <> 0 then FT_flag:=true;    //if found cam81 - connect
   GetFTDeviceDescription(I);
   Dec(I);
  end;
  FT_OP_flag:=true;
  if FT_flag then
   begin
    if Open_USB_Device_By_Serial_Number(FT_HANDLEA,'CAM86A') <> FT_OK then FT_OP_flag := false;
    if Open_USB_Device_By_Serial_Number(FT_HANDLEB,'CAM86B')  <> FT_OK then FT_OP_flag := false;

    if Set_USB_Device_BitMode(FT_HANDLEB,$bf, $4)  <> FT_OK then FT_OP_flag := false;
    FT_Current_Baud:=spusb;                         // BitMode for B-canal volocity = spusb
    Set_USB_Device_BaudRate(FT_HANDLEB);

    Set_USB_Device_LatencyTimer(FT_HANDLEB,2);       //maximum speed
    Set_USB_Device_LatencyTimer(FT_HANDLEA,2);
    Set_USB_Device_TimeOuts(FT_HANDLEA,6000,100);
    Set_USB_Device_TimeOuts(FT_HANDLEB,100,100);
    Set_USB_Parameters(FT_HANDLEA,65536,0);

    Purge_USB_Device_IN(FT_HANDLEA);
    Purge_USB_Device_OUT(FT_HANDLEA);
    Purge_USB_Device_IN(FT_HANDLEB);
    Purge_USB_Device_OUT(FT_HANDLEB);

    adress:=0;

    AD9822(0,$d8);//$58);             // AD9822 mode - Channel G, 4 volt reference, CDS mode
    AD9822(1,$a0);
    
    CameraSetGain(0);         // Set a gain. that is not full of ADC
    CameraSetOffset(-6);

    sleep(100);
    //send init command
    Spi_comm($db,0);
    sleep(100);

    Purge_USB_Device_IN(FT_HANDLEA);// Remove the 2 bytes that have arisen after the reset
    mBin:=0;

    mCameraState:=0;
   end;
 IsConnected := FT_flag and FT_OP_flag;
 Result := FT_flag and FT_OP_flag;
end;


{Disconnect camera, return bool result}
function CameraDisConnect (): WordBool;
var FT_OP_flag : boolean;
begin
 FT_OP_flag := true;
 if Close_USB_Device(FT_HANDLEA) <> FT_OK then FT_OP_flag := false;   //closing devices
 if Close_USB_Device(FT_HANDLEB) <> FT_OK then FT_OP_flag := false;
 IsConnected := not FT_OP_flag;
 Result:= FT_OP_flag;
end;

procedure ExposureTimerTick(TimerID, Msg: Uint; dwUser, dw1, dw2: DWORD); stdcall;
begin
 dec(indval);
 if indval <= 0 then
 begin
  KillTimer (0,ExposureTimer);
  Spi_comm($cb,0); //clear frame
  sleep(180);                           // for time of clear frame
  readframe;
 end;
end;

{Check camera connection, return bool result}
function CameraIsConnected () : WordBool;// stdcall; export;
begin
  Result := IsConnected;
end;

function CameraStartExposure (Bin,StartX,StartY,NumX,NumY : integer; Duration : double; light : WordBool) : WordBool;// stdcall; export;
var
d0,d1:word;
expoz:integer;
begin

 mYn:=StartY div 2;
 Spi_comm($4b,mYn);
 mdeltY:=NumY div 2;
 Spi_comm($5b,mdeltY);

 mBin := Bin;
 if bin = 1 then
 begin
 kolbyte:=mdeltY*3008;
 Spi_comm($8b,1);  //bining
 mBin:=1;
 end            else
 begin
 kolbyte:=mdeltY*12008;
 Spi_comm($8b,0);  //no bining
 mBin:=0;
 end;

 expoz:=round(Duration*1000);
 if expoz > 1000 then expoz:=1001;
 Spi_comm($6b,expoz);

 mImageReady := false;
 //camera exposing
 mCameraState := 1;
 if Duration > 1.0 then
 begin
  Spi_comm($2b,0); //shift3
  sleep(40);
  //Spi_comm($cb,0); //clear frame
  //sleep(180);                           // for time of clear frame
  Spi_comm($3b,0); //off 15v
  eexp:=round(1000*(Duration-1.0)); //1.2
  indval:=eexp div 1000;
  ExposureTimer := SetTimer(0,0,1000,@ExposureTimerTick);
 end                   else
 begin
  eexp:=0;
  //Spi_comm($cb,0);
  readframe;
 end;
 Result := true;
end;

function CameraStopExposure : WordBool;// stdcall; export;
begin
 indval:=0;
 KillTimer (0,ExposureTimer);
 Spi_comm($cb,0); //clear frame
 sleep(180);                           // for time of clear frame
 if mCamerastate = 1 then readframe;
 indval:=0;
 Result := true;
end;

function cameraGetTemp (): double;
var temp : double;
begin
    Spi_comm($bf,0);
    temp := (siout - TemperatureOffset) / 10.0;
    if ((temp > MaxErrTemp) or (temp < MinErrTemp)) then
    begin
        temp := sensorTempCache;
    end;
    sensorTempCache := temp;
    Result := temp;
end;

function cameraSetTemp(temp : double): WordBool;
var d0:word;
begin
    d0 := TemperatureOffset + round(temp*10);
    Spi_comm($ab,d0);
    Result := true;
end;

function CameraCoolingOn (): WordBool;
begin
 Spi_comm($9b,1);
 Result := true;
end;

function CameraCoolingOff (): WordBool;
begin
 Spi_comm($9b,0);
 Result := true;
end;

end.