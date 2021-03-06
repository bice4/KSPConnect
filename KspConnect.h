#include "Arduino.h"

//Input enums
#define SAS 7
#define RCS 6
#define LIGHTS 5
#define GEAR 4
#define BRAKES 3
#define PRECISION 2
#define ABORT 1
#define STAGE 0

//Action group statuses
#define AGSAS      0
#define AGRCS      1
#define AGLight    2
#define AGGear     3
#define AGBrakes   4
#define AGAbort    5
#define AGCustom01 6
#define AGCustom02 7
#define AGCustom03 8
#define AGCustom04 9
#define AGCustom05 10
#define AGCustom06 11
#define AGCustom07 12
#define AGCustom08 13
#define AGCustom09 14
#define AGCustom10 15

//SAS Modes
#define SMOFF           0
#define SMSAS           1
#define SMPrograde      2
#define SMRetroGrade    3
#define SMNormal        4
#define SMAntinormal    5
#define SMRadialIn      6
#define SMRadialOut     7
#define SMTarget        8
#define SMAntiTarget    9
#define SMManeuverNode  10

//Navball Target Modes
#define NAVBallIGNORE   0
#define NAVBallORBIT    1
#define NAVBallSURFACE  2
#define NAVBallTARGET   3

#define details(name) (uint8_t*)&name,sizeof(name)
#define IDLETIMER 2000
#define CONTROLREFRESH 25

struct HandShakePacket
{
  byte id;
  byte M1;
  byte M2;
  byte M3;
};

struct ControlPacket
{
  byte id;
  byte MainControls;                  //SAS RCS Lights Gear Brakes Precision Abort Stage
  byte Mode;                          //0 = stage, 1 = docking, 2 = map
  unsigned int ControlGroup;          //control groups 1-10 in 2 bytes
  byte NavballSASMode;                //AutoPilot mode
  byte AdditionalControlByte1;
  int Pitch;                          //-1000 -> 1000
  int Roll;                           //-1000 -> 1000
  int Yaw;                            //-1000 -> 1000
  int TX;                             //-1000 -> 1000
  int TY;                             //-1000 -> 1000
  int TZ;                             //-1000 -> 1000
  int WheelSteer;                     //-1000 -> 1000
  int Throttle;                       //    0 -> 1000
  int WheelThrottle;                  //    0 -> 1000
};

struct VesselData
{
  byte id;                //1
  float AP;               //2
  float PE;               //3
  float SemiMajorAxis;    //4
  float SemiMinorAxis;    //5
  float VVI;              //6
  float e;                //7
  float inc;              //8
  float G;                //9
  long TAp;               //10
  long TPe;               //11
  float TrueAnomaly;      //12
  float Density;          //13
  long period;            //14
  float RAlt;             //15
  float Alt;              //16
  float Vsurf;            //17
  float Lat;              //18
  float Lon;              //19
  float LiquidFuelTot;    //20
  float LiquidFuel;       //21
  float OxidizerTot;      //22
  float Oxidizer;         //23
  float EChargeTot;       //24
  float ECharge;          //25
  float MonoPropTot;      //26
  float MonoProp;         //27
  float IntakeAirTot;     //28
  float IntakeAir;        //29
  float SolidFuelTot;     //30
  float SolidFuel;        //31
  float XenonGasTot;      //32
  float XenonGas;         //33
  float LiquidFuelTotS;   //34
  float LiquidFuelS;      //35
  float OxidizerTotS;     //36
  float OxidizerS;        //37
  uint32_t MissionTime;   //38
  float deltaTime;        //39
  float VOrbit;           //40
  uint32_t MNTime;        //41
  float MNDeltaV;         //42
  float Pitch;            //43
  float Roll;             //44
  float Heading;          //45
  uint16_t ActionGroups;  //46  status bit order:SAS, RCS, Light, Gear, Brakes, Abort, Custom01 - 10
  byte SOINumber;         //47  SOI Number (decimal format: sun-planet-moon e.g. 130 = kerbin, 131 = mun)
  byte MaxOverHeat;       //48  Max part overheat (% percent)
  float MachNumber;       //49
  float IAS;              //50  Indicated Air Speed
  byte CurrentStage;      //51  Current stage number
  byte TotalStage;        //52  TotalNumber of stages
  float TargetDist;       //53  Distance to targeted vessel (m)
  float TargetV;          //54  Target vessel relative velocity
  byte NavballSASMode;    //55  Combined byte for navball target mode and SAS mode
                          // First four bits indicate AutoPilot mode:
                          // 0 SAS is off  //1 = Regular Stability Assist //2 = Prograde
                          // 3 = RetroGrade //4 = Normal //5 = Antinormal //6 = Radial In
                          // 7 = Radial Out //8 = Target //9 = Anti-Target //10 = Maneuver node
                          // Last 4 bits set navball mode. (0=ignore,1=ORBIT,2=SURFACE,3=TARGET)
  uint16_t ProgradePitch;  //56 Pitch   Of the Prograde Vector;  int_16 ***Changed: now fix point, actual angle = angle/50*** used to be (-0x8000(-360 degrees) to 0x7FFF(359.99ish degrees));
  uint16_t ProgradeHeading;//57 Heading Of the Prograde Vector;  see above for range   (Prograde vector depends on navball mode, eg Surface/Orbit/Target)
  uint16_t ManeuverPitch;  //58 Pitch   Of the Maneuver Vector;  see above for range;  (0 if no Maneuver node)
  uint16_t ManeuverHeading;//59 Heading Of the Maneuver Vector;  see above for range;  (0 if no Maneuver node)
  uint16_t TargetPitch;    //60 Pitch   Of the Target   Vector;  see above for range;  (0 if no Target)
  uint16_t TargetHeading;  //61 Heading Of the Target   Vector;  see above for range;  (0 if no Target)
  uint16_t NormalHeading;  //62 Normal Of the Prograde Vector;  see above for range;  (Pitch of the Heading Vector is always 0)
};

void SetControlPacketValues(int throttle, int pitch, int yaw);

boolean KSPBoardReceiveData();
void KSPBoardSendData(uint8_t *address, uint8_t len);

VesselData GetVesselData();

void KSPSendControlPacket();
void KSPDataHandler(void (*)());

boolean GetConnect();

void Handshake();
void MainControls(byte n, boolean s);

void ControlGroups(byte n, boolean s);
byte ControlStatus(byte n);

void setNavballMode(byte m);
byte getNavballMode();

void setSASMode(byte m);
byte getSASMode();

void KspConnectInitialize();