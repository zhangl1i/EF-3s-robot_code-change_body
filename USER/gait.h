#ifndef _MC
#define _MC
#include <sys.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"
#include "adc.h"
#ifndef _ENUM_BOOL
#define _ENUM_BOOL
typedef enum bool {true = 1, false = 0}bool;

#endif 

extern bool isProbingMode;
extern bool isSecendTrial;
extern bool isSecendUp;
extern int manualControl;
extern float rate[3];
extern float legv_rate[4];
typedef struct CreepMotionControl struct_MC;
extern struct CreepMotionControl transition;
extern int updateStatus, gaitModeBuffer, statusNum, runFlag,controlRunFlag;
extern float timeGaitBuffer, pressHightBuffer[4], targetCoMVelocityBuffer[3];
extern float ftsZeroSet[12];
struct CreepMotionControl
{	
	float timeGait;  // The time of the whole cycle
	float timePeriod;	// The time of one step
  float presentTime;
	float times;
	float roll;
	float width;
	float length;
	float L1, L2, L3, L2_f, L2_h;  // The length of L
	int legStatus[4];	// leg status: 0-stance, 1-swingUp, 2-swingDown, 3-preAttach, 4-wait, 5-recover
	int statusTimes[4], statusTimesBuffer[4][7];
	float swingStatusTimeFactor[7];	//	6-the number of legStatus
	float pressHight[4];	//	the pressure height in the end of swing phase
	float stepHight[4];		//
// 0-trot_90;	1-amble_90; 2-trot_180; 3-amble_180; 4-tort Outer curved; 5-amble Outer curved; 6-tort Inner curved; 7-amble Inner curved; 8-amble_90 back
	int gaitMode;	
	int legRunTimes[4], legRunTimesBuffer[4];
	int swingLeg;
	Mat comPos;  // 3X1, position of center of mass;	COM to world
	Mat legComPos[4];
	Mat shoulderPos;  //	4X3, LF RF LH RH X, Y  shoulder to COM 
//	Mat shoulderPos_bymotor;
	Mat footPos;  //	4X3, LF RF LH RH X, Y, Z	foot to world
	Mat ftsPos;  //	4X3, LF RF LH RH X, Y, Z foot_to_shoulder
	Mat ftsPosOffset;	// 	4X3
//	Mat ftsPos_bymotor;
	Mat targetCoMVelocity;  // 3X1, X, Y , alpha c in world cordinate
	Mat d_targetCoMVelocity;
	//Matrix<float, 4, 3> shoulderPos_for_real;  // real shoulder position by motor feedback
	//Vector<float, 4> com_position;  // coordinate of center of mass
	//Vector<float, 4> com_position_for_real;
	Mat targetPos;
	Mat motorPos;  //	4X3, LF RF LH RH 1,2,3 motor position command
	Mat motorOffset;	// 4X3
//	Mat motorPosFdb;  // LF RF LH RH 1,2,3 motor position command by motor feedback
	// Mat timeForStancePhase;	// 4X2
	Mat timeForSwingPhase;	// 4X2, the start and end time of swing phase in porportion form
	Mat timeForSwing;	// 4X1, the duration of swing phase
	Mat swingPhaseVelocity;	// 4X3	x,y,z
//	Mat targetCoMPosition;  // X, Y , alpha in world cordinate
	Mat stanceFlag;  // True, False: LF, RF, LH, RH


	Mat stancePhaseStartPos;
	Mat stancePhaseEndPos;
	int isSuction[4];
	float downwardProbingDis[4][3];
  float ftsPosition[12];
	float footposition[12];
	float jointPos[12];
	float shoulderpos[12];
	float angleForShoulder[4]; 
};
void CLASSMC_defMatrix(struct_MC *p);
void CLASSMC_freeMatrix(struct_MC *p);
void CLASSMC_setInitPos(struct_MC *p);
void CLASSMC_setCoMVel(struct_MC *p, float* val);
void CLASSMC_nextStep(struct_MC *p);
void CLASSMC_inverseKinematics(struct_MC *p);
void CLASSMC_forwardKinematics(struct_MC *p);
void CLASSMC_setJointPosition(struct_MC *p);
void CLASSMC_initiation(struct_MC *p, int gaitMode);
void CLASSMC_turn(struct_MC *p);
void fts(struct_MC *p);
void updateGaitParameter(struct_MC *p);
void CLASSMC_transParamDeliver(struct_MC *p, struct_MC *trans, int gaitMode);
void CLASSMC_transitionStep(struct_MC *p);
void runOneStep(struct_MC *p);
void downwardProbing(struct_MC *p);
//	char positionName[12][22] = {"LF0 PositionSensor", "LF1 PositionSensor", "LF2 PositionSensor", "RF0 PositionSensor", "RF1 PositionSensor", "RF2 PositionSensor", "LH0 PositionSensor", "LH1 PositionSensor", "LH2 PositionSensor", "RH0 PositionSensor", "RH1 PositionSensor", "RH2 PositionSensor"};
//	char motorName[12][22] = {"LF0 RotationalMotor", "LF1 RotationalMotor", "LF2 RotationalMotor", "RF0 RotationalMotor", "RF1 RotationalMotor", "RF2 RotationalMotor", "LH0 RotationalMotor", "LH1 RotationalMotor", "LH2 RotationalMotor", "RH0 RotationalMotor", "RH1 RotationalMotor", "RH2 RotationalMotor"};
//	char touchsensorName[4][16] = {"LF_touch_sensor", "RF_touch_sensor", "LH_touch_sensor", "RH_touch_sensor"};
//        PositionSensor *Ps[12];
//        Motor *Tor[12];
//        TouchSensor *Ts[4];
//        InertialUnit *imu;
//        GPS *gps;
        
//        Vector<float, 3> imu_num;        //VMC
#endif