#include "gait.h"
#include "matrix.h"
#include "pwm.h"
#include "uart.h"
#include "timer.h"
#include "SV.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.1415926

#define StancePullHight 6			// 6
#define StancePullHightDiag -4    //-4
#define transferForceX 5
#define TrialCorrectionHeight 5.0

#define ProbingDistanceX 0//5.0
#define ProbingDistanceYL 0//5.0
#define ProbingDistanceYR 0//-5.0
#define ProbingDistanceZ 5.0

#define ProbingDistanceX2 3.0
#define ProbingDistanceYL2 3.0
#define ProbingDistanceYR2 -3.0
#define ProbingDistanceZ2 3.0

//#define pressureThresholdLF 65535;
//#define pressureThresholdRF 65535;
//#define pressureThresholdLH 65535;
//#define pressureThresholdRH 65535;

#define pressureThresholdLF 40000;
#define pressureThresholdRF 40000;
#define pressureThresholdLH 40000;
#define pressureThresholdRH 40000;

#define ProbingTimes 20
struct CreepMotionControl mc, transition;
// 0-trot_90;	1-amble_90; 2-trot_180; 3-amble_180; 
// 4-tort Outer curved; 5-amble Outer curved; 6-tort Inner curved; 7-amble Inner curved;
// 8-amble_90 back; 9-tort_180 turn; 10-amble_180 turn;12-ground,13-20min
int gaitModeBuffer = 0X03;
uint16_t pressureThreshold[4];
int updateStatus, statusNum, runFlag,controlRunFlag;
float timeGaitBuffer, targetCoMVelocityBuffer[3], pressHightBuffer[4];
float lastZ[4];
bool isProbingMode;
bool isSecendTrial;
bool isSecendUp;
int manualControl;
float legv_rate[4]={0};
float ftsZeroSet[12]= {//0	 
2, 0, 6,
0, 0, 0,
0, -4,-2,
-2, 0, 0
};

float offset90[12]= {0

};
float offset180[12]={//0
/*
	涵道内曲面姿态-Y
 */
//-4, -4, -4,
//-4, -8, -4,
//12, -4, 0,
//8, -10, 0
	
/*
	涵道内曲面姿态+X
 */
//-4, -4, 0,
//-4, -10, -8,
//12, -4,4,
//8, -10, -4
//	
	/* 
	前伸
	*/
	
//8, -4, 0,
//8, -10, -8,
//12, -4,4,
//8, -10, -4

/*
//	180°倒置面姿态/圆弧内姿态
//	*/
12, -4, 2,
12, -4, 4,
8, -4, 4,
8, -8, 4
/*
	180°倒置面姿态前伸
	*/
//10, -4, 0,
//10, -8, 0,
//8, -4, 4,
//8, -8, 8

/*圆弧内壁姿态前伸
	*/

};
float offset180Turn[12]={	0

};
float offsetGround[12]={	0

};

float offsetkeep180[12]={	0

};

//=======================================================================
// 全局变量或放在结构体里，用于低通滤波，防止姿态突变
float estimated_roll = 0;
float estimated_pitch = 0;
/**
 * @brief 基于支撑腿位置估计地形法向，并生成期望姿态
 * @param p 机器人控制结构体
 */
void TerrainAdaptation(struct_MC *p)
{
    // 1. 获取四条腿在机身坐标系下的实际位置 (Foot Position relative to Body)
    // P_foot = P_shoulder + P_fts
    float feet_pos[4][3];
    int stance_legs[4];
    int stance_count = 0;

    for(int i=0; i<4; i++) {
		// 还原真实的物理坐标，剔除人为的校准垫片 (offset)
        // 这样算法看到的 LF, RF, LH, RH 的 Z 值在平地上将是完全一致的
        float raw_x = p->ftsPos.element[i][0] - p->ftsPosOffset.element[i][0];
        float raw_y = p->ftsPos.element[i][1] - p->ftsPosOffset.element[i][1];
        float raw_z = p->ftsPos.element[i][2] - p->ftsPosOffset.element[i][2];
        // ===============================================

        // 计算足端相对于机身中心的坐标
        feet_pos[i][0] = p->shoulderPos.element[i][0] + p->ftsPos.element[i][0];
        feet_pos[i][1] = p->shoulderPos.element[i][1] + p->ftsPos.element[i][1];
        feet_pos[i][2] = p->shoulderPos.element[i][2] + p->ftsPos.element[i][2];

        // 记录处于支撑相的腿 (legStatus == 0 通常表示支撑相，或者是吸附状态)
        // 根据你的代码逻辑：0-stance, 4-Attach, 5-recover 等都是接触壁面的
        if(p->legStatus[i] == 0 || p->legStatus[i] == 4 || p->legStatus[i] == 5) {
            stance_legs[stance_count++] = i;
        }
    }

    // 2. 拟合平面法向量 (至少需要3个点)
	// 如果支撑腿少于3个，无法拟合
    if(stance_count < 3) return; 

    // 【核心修改】计算两条对角线的向量，然后求叉积
    // 这种方法平等地利用了四条腿的信息，不仅对称，而且对单腿误差不敏感
    
    // 向量 A: 左后 -> 右前 (LH -> RF)
    // 也就是 P1(RF) - P2(LH)
    float v_diagA[3];
    v_diagA[0] = feet_pos[1][0] - feet_pos[2][0];
    v_diagA[1] = feet_pos[1][1] - feet_pos[2][1];
    v_diagA[2] = feet_pos[1][2] - feet_pos[2][2];

    // 向量 B: 右后 -> 左前 (RH -> LF)
    // 也就是 P0(LF) - P3(RH)
    float v_diagB[3];
    v_diagB[0] = feet_pos[0][0] - feet_pos[3][0];
    v_diagB[1] = feet_pos[0][1] - feet_pos[3][1];
    v_diagB[2] = feet_pos[0][2] - feet_pos[3][2];

    // 法向量 n = v_diagA x v_diagB
    // 这相当于计算了两条对角线构成的平面的法向，能够平均化四条腿的高度
    float normal[3];
    VecCross(v_diagA, v_diagB, normal);
    
    // 确保法向量向上
    if(normal[2] < 0) {
        normal[0] = -normal[0];
        normal[1] = -normal[1];
        normal[2] = -normal[2];
    }
    VecNormalize(normal);

    // 3. 计算对齐所需的欧拉角 (Target Roll & Pitch)
    // 机身Z轴为 (0,0,1)。要让机身Z轴平行于 normal，需要旋转机身。
    // 公式推导：n = R * [0,0,1]^T
    // 简化计算 (假设小角度近似):
    // pitch ~ atan2(n.x, n.z)
    // roll  ~ atan2(-n.y, n.z) 
    
    float target_pitch = atan2f(normal[0], normal[2]);
    float target_roll  = atan2f(-normal[1], normal[2]);

    // 4. 低通滤波 (平滑处理，防止舵机抖动)
    float alpha = 0.1f; // 滤波系数，越小越平滑，但响应越慢
    estimated_pitch = estimated_pitch * (1 - alpha) + target_pitch * alpha;
    estimated_roll  = estimated_roll  * (1 - alpha) + target_roll  * alpha;
    
    // 5. 应用姿态调整
    // 在四足逆运动学中，"调整机身姿态" 等效于 "反向旋转所有足端位置"
    // 我们需要修改 p->roll 和 p->pitch (如果结构体里有) 或者直接作用于 ftsPos
    
    // 这里我们修改 p->roll 和 p->pitch 的等效变量，或者直接更新全局姿态矩阵
    // 假设你还没有全局姿态变量，我们直接在这里修改 p->ftsPos 是不行的，
    // 因为 ftsPos 是相对于机身的。
    
    // 正确的做法：在逆运动学 (Inverse Kinematics) 计算之前，应用这个旋转。
}
//=======================================================================

/**
 * @brief Initial of parameters, calculate offset of motors from offset of foot position and set joint position.
 * 
 * @param p 
 * @param gaitMode 
 */
void CLASSMC_initiation(struct_MC *p, int gaitMode)
{
float offset[12];
p->times = 0;
p->width = 44;       //LF-RF shoulder distance;
p->length = 78.9;    //LF-LH shoulder distance;
p->L1 = 45;
p->L2 = 50;		
p->L2_f = 50;		//f
p->L2_h = 60;	//h
p->L3 = 22;  
p->timePeriod = 0.020;
p->presentTime = 0;
p->gaitMode = gaitMode;
for(int i=0;i<4;i++) p->isSuction[i]=3;
pressureThreshold[0]=pressureThresholdLF;
pressureThreshold[1]=pressureThresholdRF;
pressureThreshold[2]=pressureThresholdLH;
pressureThreshold[3]=pressureThresholdRH;
for(int i=0;i<4;i++)
	for(int j=0;j<3;j++)
		p->downwardProbingDis[i][j] = 0;
switch(p->gaitMode)
{
case 0x00:	//	offset of initial position	in 90 degrees	; Trot
	p->timeGait = 3.2;
	float timeForSwingPhase0[]={ 	8*p->timeGait/16, 	15*p->timeGait/16,	
																0, 		 							7*p->timeGait/16,		
																0, 	 	 							7*p->timeGait/16,		
																8*p->timeGait/16, 	15*p->timeGait/16};
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase0); 
	float pressHightBuffer0[4]={14, 14, 10, 10};	//16, 16, 12, 12
	float stepHightBuffer0[4]={20,20,25,25};		//20, 20, 25, 25
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer0[i];
		p->stepHight[i] = stepHightBuffer0[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i];
	p->targetCoMVelocity.element[0][0] = 13;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x01:	//	offset of initial position	in 90 degrees	; Amble
	p->timeGait = 8;
	float timeForSwingPhase1[]={ 	8*p->timeGait/16, 		11*p->timeGait/16,		
																0,		 		 						3*p->timeGait/16,		
																12*p->timeGait/16, 		15*p->timeGait/16,		
																4*p->timeGait/16, 		7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase1); 
	float pressHightBuffer1[4]={14, 16, 16, 16};//16, 16, 12, 12
	float stepHightBuffer1[4]={35, 35, 25, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer1[i];
		p->stepHight[i] = stepHightBuffer1[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i];		
	p->targetCoMVelocity.element[0][0] = 3;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x02:	//	offset of initial position	in 180 degrees	; Tort
	p->timeGait = 4;
	float timeForSwingPhase2[]={ 	8*p->timeGait/16, 	16*p->timeGait/16,	
																0, 		 							8*p->timeGait/16,		
																0, 	 	 							8*p->timeGait/16,		
																8*p->timeGait/16, 	16*p->timeGait/16};
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase2); 
	float pressHightBuffer2[4]={10, 10, 12, 12};//{16, 18, 16, 18};
	float stepHightBuffer2[4]={12, 20, 22, 14};//{12, 12, 14, 14};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer2[i];
		p->stepHight[i] = stepHightBuffer2[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset180[i];
	p->targetCoMVelocity.element[0][0] = 10;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x03:	//	offset of initial position	in 180 degrees	; Amble
	p->timeGait = 8;
	float timeForSwingPhase3[]={ 	8*p->timeGait/16, 	11*p->timeGait/16,		
																0,		 		 					3*p->timeGait/16,		
																12*p->timeGait/16, 	15*p->timeGait/16,		
																4*p->timeGait/16, 	7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase3); 
	float pressHightBuffer3[4]={14, 14, 20, 16};//{14, 22, 20, 16};
	float stepHightBuffer3[4]={18, 14, 16, 20};//{16, 16, 18, 18};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer3[i];
		p->stepHight[i] = stepHightBuffer3[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset180[i];
	p->targetCoMVelocity.element[0][0] = 4;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x04:	//	outer curved surface with 90 degrees	; Tort
	p->timeGait = 4;
	float timeForSwingPhase4[]={ 	8*p->timeGait/16, 	15*p->timeGait/16,	
																0, 		 							7*p->timeGait/16,		
																0, 	 	 							7*p->timeGait/16,		
																8*p->timeGait/16, 	15*p->timeGait/16};
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase4); 
	float pressHightBuffer4[4]={10, 10, 10, 10};
	float stepHightBuffer4[4]={20, 29, 20, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer4[i];
		p->stepHight[i] = stepHightBuffer4[i];
	}
	float tortOutCurve90[]={
	-2,0,-10,
	-2,-0,-10,
	-2,0,-10,
	-2,-0,-10 };
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i] + tortOutCurve90[i];		
	p->targetCoMVelocity.element[0][0] = 10;
	p->targetCoMVelocity.element[1][0] = 6;
	p->targetCoMVelocity.element[2][0] = 4.2 *3.1415/180.0;
	break;
case 0x05:	//	outer curved surface with 90 degrees	; Amble
	p->timeGait = 8;
	float timeForSwingPhase5[]={ 	8*p->timeGait/16, 		11*p->timeGait/16,		
																0,		 		 						3*p->timeGait/16,		
																12*p->timeGait/16, 		15*p->timeGait/16,		
																4*p->timeGait/16, 		7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase5); 
	float pressHightBuffer5[4]={12, 12, 12, 12};
	float stepHightBuffer5[4]={25, 25, 25, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer5[i];
		p->stepHight[i] = stepHightBuffer5[i];
	}
	float ambleOutCurve90[]={
	-2,0,-10,
	-2,-0,-10,
	-2,0,-10,
	-2,-0,-10 };
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i] + ambleOutCurve90[i];		
	p->targetCoMVelocity.element[0][0] = 3;
	p->targetCoMVelocity.element[1][0] = 1;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x06:	// inner curved surface	; Tort
	p->timeGait = 4;
	float timeForSwingPhase6[]={ 	8*p->timeGait/16, 	15*p->timeGait/16,	
																0, 		 							7*p->timeGait/16,		
																0, 	 	 							7*p->timeGait/16,		
																8*p->timeGait/16, 	15*p->timeGait/16};
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase6); 
	float pressHightBuffer6[4]={12, 12, 12, 12};
	float stepHightBuffer6[4]={25, 25, 25, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer6[i];
		p->stepHight[i] = stepHightBuffer6[i];
	}
	float tortInnerCurve[]={
	0,0,10,
	0,0,10,
	0,6,16,
	0,-6,16};
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i] + tortInnerCurve[i];		
	p->targetCoMVelocity.element[0][0] = 10;
	p->targetCoMVelocity.element[1][0] = 4;
	p->targetCoMVelocity.element[2][0] = 3.0 *3.1415/180.0;
	break;
case 0x07:	//	inner curved surface ; Amble
	p->timeGait = 8;
	float timeForSwingPhase7[]={ 	8*p->timeGait/16, 		11*p->timeGait/16,		
																0,		 		 						3*p->timeGait/16,		
																12*p->timeGait/16, 		15*p->timeGait/16,		
																4*p->timeGait/16, 		7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase7); 
	float pressHightBuffer7[4]={12, 12, 12, 12};
	float stepHightBuffer7[4]={25, 25, 25, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer7[i];
		p->stepHight[i] = stepHightBuffer7[i];
	}
	float ambleInnerCurve[]={
	0,0,10,
	0,0,10,
	0,6,16,
	0,-6,16};
	for (int i = 0; i < 12; i++)
		offset[i] = offset90[i] + ambleInnerCurve[i];		
	p->targetCoMVelocity.element[0][0] = 3;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x08:		//	 90 degrees	; Amble ; back
	p->timeGait = 8;
	float timeForSwingPhase8[]={ 	8*p->timeGait/16, 		11*p->timeGait/16,		
																0,		 		 						3*p->timeGait/16,		
																12*p->timeGait/16, 		15*p->timeGait/16,		
																4*p->timeGait/16, 		7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase8); 
	float pressHightBuffer8[4]={16, 16, 16, 16};//16, 16, 12, 12
	float stepHightBuffer8[4]={25, 25, 25, 25};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer8[i];
		p->stepHight[i] = stepHightBuffer8[i];
	}
	for(int i = 0; i < 12; i++)
		offset[i] = offset90[i];		
	for(int i = 0; i < 4; i++)
		offset[i*3] = -1 * offset[i*3];
	p->targetCoMVelocity.element[0][0] = -3.0;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0x09:			//    180 degrees	; Tort	;	turn
	p->timeGait = 4;
	float timeForSwingPhase9[]={  8*p->timeGait/16, 	15*p->timeGait/16,	
																0, 		 							7*p->timeGait/16,		
																0, 	 	 							7*p->timeGait/16,		
																8*p->timeGait/16, 	15*p->timeGait/16};
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase9); 
	float pressHightBuffer9[4]={14, 14, 12, 12};
	float stepHightBuffer9[4]={19, 19, 8, 8};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer9[i];
		p->stepHight[i] = stepHightBuffer9[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset180Turn[i];
	p->targetCoMVelocity.element[0][0] = 0;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 8 *3.1415/180.0;	// alpha (radian)
	break;
case 0x0A:			//    180 degrees	; Amble	;	turn
	p->timeGait = 8;
	float timeForSwingPhase10[]={ 8*p->timeGait/16, 	11*p->timeGait/16,		
																0,		 		 					3*p->timeGait/16,		
																12*p->timeGait/16, 	15*p->timeGait/16,		
																4*p->timeGait/16, 	7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase10); 
	float pressHightBuffer10[4]={12, 12, 15, 16};	//12, 12, 12, 12
	float stepHightBuffer10[4]={14, 14, 16, 16};	//12, 12, 8, 8
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer10[i];
		p->stepHight[i] = stepHightBuffer10[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset180Turn[i];
	p->targetCoMVelocity.element[0][0] = 0;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0 *3.1415/180.0;	// alpha (radian)
	break;
case 0x0B:			//    ground gait;
p->timeGait = 8;
float timeForSwingPhase11[]={ 8*p->timeGait/16, 	11*p->timeGait/16,		
															0,		 		 					3*p->timeGait/16,		
															12*p->timeGait/16, 	15*p->timeGait/16,		
															4*p->timeGait/16, 	7*p->timeGait/16};	
MatSetVal(&p->timeForSwingPhase, timeForSwingPhase11); 
float pressHightBuffer11[4]={0, 0, 0, 0};	//12, 12, 12, 12
float stepHightBuffer11[4]={20, 20, 20, 20};	//12, 12, 8, 8
for(int i=0; i<4; i++)
{
	p->pressHight[i] = pressHightBuffer11[i];
	p->stepHight[i] = stepHightBuffer11[i];
}
for (int i = 0; i < 12; i++)
	offset[i] = offsetGround[i];
p->targetCoMVelocity.element[0][0] = 45.0;
p->targetCoMVelocity.element[1][0] = 0;
p->targetCoMVelocity.element[2][0] = 0;	// alpha (radian)
break;
case 0x0C:			//    ground gait TROT;
p->timeGait = 1.2;
float timeForSwingPhase12[]={ 8*p->timeGait/16, 	15*p->timeGait/16,	
																0, 		 							7*p->timeGait/16,		
																0, 	 	 							7*p->timeGait/16,		
																8*p->timeGait/16, 	15*p->timeGait/16};
MatSetVal(&p->timeForSwingPhase, timeForSwingPhase12); 
float pressHightBuffer12[4]={0, 0, 0, 0};	//12, 12, 12, 12
float stepHightBuffer12[4]={20, 20, 20, 20};	//12, 12, 8, 8
for(int i=0; i<4; i++)
{
	p->pressHight[i] = pressHightBuffer12[i];
	p->stepHight[i] = stepHightBuffer12[i];
}
for (int i = 0; i < 12; i++)
	offset[i] = offsetGround[i];
p->targetCoMVelocity.element[0][0] = 90.0;
p->targetCoMVelocity.element[1][0] = 0;
p->targetCoMVelocity.element[2][0] = 0.25;	// alpha (radian)
break;

case 0x0D:	//	offset of initial position	in 180 degrees	; Amble---20min test mode
	p->timeGait = 8;
	float timeForSwingPhase14[]={ 	8*p->timeGait/16, 	11*p->timeGait/16,		
																0,		 		 					3*p->timeGait/16,		
																12*p->timeGait/16, 	15*p->timeGait/16,		
																4*p->timeGait/16, 	7*p->timeGait/16};	
	MatSetVal(&p->timeForSwingPhase, timeForSwingPhase3); 
	float pressHightBuffer14[4]={10, 10, 10, 10};//{16, 22, 16, 18};
	float stepHightBuffer14[4]={14, 14, 16, 16};//{16, 16, 18, 18};
	for(int i=0; i<4; i++)
	{
		p->pressHight[i] = pressHightBuffer14[i];
		p->stepHight[i] = stepHightBuffer14[i];
	}
	for (int i = 0; i < 12; i++)
		offset[i] = offset180[i];
	p->targetCoMVelocity.element[0][0] = 1;
	p->targetCoMVelocity.element[1][0] = 0;
	p->targetCoMVelocity.element[2][0] = 0;
	break;
case 0xFF:	//	offset of initial position
	float offsetFF[12]={ 0 };
	for(int i=0; i<12; i++)
		offset[i] = offsetFF[i];
	break;
}

float t_shoulderPos[]={p->length/2, p->width/2, 0, p->length/2, -p->width/2, 0, -p->length/2, p->width/2, 0, -p->length/2, -p->width/2, 0}; // X-Y-Z-alpha: LF, RF, LH, RH
MatSetVal(&p->shoulderPos,t_shoulderPos);

float tempFtsPos[]={		
p->L2_f, p->L1, -p->L3, 
p->L2_f, -p->L1, -p->L3,
-p->L2_h, p->L1, -p->L3,
-p->L2_h, -p->L1, -p->L3};
MatSetVal(&p->ftsPos,tempFtsPos);

if(p->gaitMode==0x0B||p->gaitMode==0x0C)
{
p->swingStatusTimeFactor[0] = 0;
p->swingStatusTimeFactor[1] = 0;
p->swingStatusTimeFactor[2] = 0.5;
p->swingStatusTimeFactor[3] = 1;
p->swingStatusTimeFactor[4] = 1;
// p->swingStatusTimeFactor[1] = 0.5;
// p->swingStatusTimeFactor[2] = 0.9;
// p->swingStatusTimeFactor[3] = 0.94;
// p->swingStatusTimeFactor[4] = 0.96;
p->swingStatusTimeFactor[5] = 1;
p->swingStatusTimeFactor[6] = 1;
}

else if (p->gaitMode==0x0D)
{
p->swingStatusTimeFactor[0] = 0;
p->swingStatusTimeFactor[1] = 0.12;
p->swingStatusTimeFactor[2] = 0.52;
p->swingStatusTimeFactor[3] = 0.72;
p->swingStatusTimeFactor[4] = 0.80;
// p->swingStatusTimeFactor[1] = 0.5;
// p->swingStatusTimeFactor[2] = 0.9;
// p->swingStatusTimeFactor[3] = 0.94;
// p->swingStatusTimeFactor[4] = 0.96;
p->swingStatusTimeFactor[5] = 0.92;
p->swingStatusTimeFactor[6] = 1;
}
else 
{
p->swingStatusTimeFactor[0] = 0;
p->swingStatusTimeFactor[1] = 0;
p->swingStatusTimeFactor[2] = 0.4;
p->swingStatusTimeFactor[3] = 0.6;
p->swingStatusTimeFactor[4] = 0.68;
// p->swingStatusTimeFactor[1] = 0.5;
// p->swingStatusTimeFactor[2] = 0.9;
// p->swingStatusTimeFactor[3] = 0.94;
// p->swingStatusTimeFactor[4] = 0.96;
p->swingStatusTimeFactor[5] = 0.92;
p->swingStatusTimeFactor[6] = 1;
} 
for(int legNum=0; legNum<4; legNum++)	// time for swingphase 
{
	p->timeForSwing.element[legNum][0] = fabs(p->timeForSwingPhase.element[legNum][1] - p->timeForSwingPhase.element[legNum][0]); 
	p->angleForShoulder[legNum] =  0;
	p->legRunTimes[legNum] = 0;
	p->legRunTimesBuffer[legNum] = 0;
	for(int i=1; i<7; i++)
	p->statusTimesBuffer[legNum][i] = floor(p->timeForSwing.element[legNum][0] * (p->swingStatusTimeFactor[i] - p->swingStatusTimeFactor[i-1])/ p->timePeriod );
	p->statusTimesBuffer[legNum][0] = floor((p->timeGait - p->timeForSwing.element[legNum][0]) / p->timePeriod);
	if(p->presentTime - p->timeForSwingPhase.element[legNum][0] >=  0 &&  p->presentTime - p->timeForSwingPhase.element[legNum][0] <  p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[1] + p->timePeriod/2)
	{
		p->legStatus[legNum] = 1;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][0] +  p->timeForSwing.element[legNum][0]*(p->swingStatusTimeFactor[1] - p->swingStatusTimeFactor[0]) - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[1] + p->timePeriod/2 && p->presentTime-p->timeForSwingPhase.element[legNum][0] < p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[2] + p->timePeriod/2)
	{
		p->legStatus[legNum] = 2;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][0] +  p->timeForSwing.element[legNum][0]*(p->swingStatusTimeFactor[2] - p->swingStatusTimeFactor[1]) - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[2] + p->timePeriod/2 && p->presentTime-p->timeForSwingPhase.element[legNum][0] < p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[3] + p->timePeriod/2)
	{
		p->legStatus[legNum] = 3;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][0] +  p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[3]- p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[3] + p->timePeriod/2 && p->presentTime - p->timeForSwingPhase.element[legNum][0] < p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[4] + p->timePeriod/2)
	{
		p->legStatus[legNum] = 4;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][0] +  p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[4] - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[4] + p->timePeriod/2 && p->presentTime < p->timeForSwingPhase.element[legNum][1] + p->timePeriod/2)
	{  
		p->legStatus[legNum] = 5;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][1] - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[5] + p->timePeriod/2 && p->presentTime < p->timeForSwingPhase.element[legNum][1] + p->timePeriod/2)
	{  
		p->legStatus[legNum] = 6;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][1] - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
		else if(p->presentTime - p->timeForSwingPhase.element[legNum][0] > p->timeForSwing.element[legNum][0] * p->swingStatusTimeFactor[6] + p->timePeriod/2 && p->presentTime < p->timeForSwingPhase.element[legNum][1] + p->timePeriod/2)
	{  
		p->legStatus[legNum] = 7;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][1] - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime  < p->timeForSwingPhase.element[legNum][0] + p->timePeriod/2)
	{
		p->legStatus[legNum] = 0;
		p->statusTimes[legNum] = floor((p->timeForSwingPhase.element[legNum][0] - p->presentTime) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	else if(p->presentTime > p->timeForSwingPhase.element[legNum][1] - p->timePeriod/2)
	{
		p->legStatus[legNum] = 0;
		p->statusTimes[legNum] = floor((p->timeGait - p->presentTime + p->timeForSwingPhase.element[legNum][0]) / p->timePeriod);
		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
	}
	for (int k = 0; k < 7; k++)
		p->legRunTimesBuffer[legNum] += p->statusTimesBuffer[legNum][k];
}
/*	calculate initial offset of motors and set initial position	*/
MatSetVal(&p->motorOffset,ftsZeroSet); 
MatAdd(&p->motorOffset, &p->ftsPos, &p->ftsPos);
MatZeros(&p->motorOffset);
CLASSMC_inverseKinematics(p);
MatCopy(&p->motorPos, &p->motorOffset);

MatSetVal(&p->ftsPosOffset,offset); 
MatSetVal(&p->ftsPos,tempFtsPos);
MatAdd(&p->ftsPosOffset, &p->ftsPos, &p->ftsPos);
MatAdd(&p->ftsPos, &p->shoulderPos, &p->footPos);
MatCopy(&p->ftsPos, &p->ftsPosOffset);
MatCopy(&p->ftsPos, &p->stancePhaseStartPos);
MatCopy(&p->ftsPos, &p->stancePhaseEndPos);
MatZeros(&p->comPos);
for(int i=0;i<4;i++)
	MatZeros(&p->legComPos[i]);
}

/**
 * @brief Allocate memory for matrix in struct_MC.
 * Must before initiation.
 * @param p 
 */
void CLASSMC_defMatrix(struct_MC *p)
{
MatCreate(&p->timeForSwingPhase, 4, 2);// startTime, endTime: LF, RF, LH, RH
MatCreate(&p->timeForSwing, 4, 1);  
MatCreate(&p->targetCoMVelocity, 3, 1);  // X, Y , alpha c in world cordinate 3x1
MatCreate(&p->comPos, 3, 1);  
for(int i=0;i<4;i++)
MatCreate(&p->legComPos[i],3,1);
MatCreate(&p->shoulderPos,4,3);  // X-Y: LF, RF, LH, RH
MatCreate(&p->footPos,4,3);
MatCreate(&p->ftsPosOffset, 4, 3);
MatCreate(&p->motorPos,4,3);
MatCreate(&p->motorOffset,4,3);
MatCreate(&p->ftsPos,4,3);
MatCreate(&p->stancePhaseStartPos,4,3);
MatCreate(&p->stancePhaseEndPos,4,3);
MatCreate(&p->swingPhaseVelocity, 4, 3);
}
/**
 * @brief Free the memory of struct_MC.
 * 
 * @param p 
 */
void CLASSMC_freeMatrix(struct_MC *p)
{
MatDelete(&p->timeForSwingPhase);// startTime, endTime: LF, RF, LH, RH
MatDelete(&p->timeForSwing);
MatDelete(&p->targetCoMVelocity);  // X, Y , alpha c in world cordinate
MatDelete(&p->comPos); 
for(int i=0;i<4;i++)
MatDelete(&p->legComPos[i]);
MatDelete(&p->shoulderPos);  // X-Y: LF, RF, LH, RH
MatDelete(&p->footPos);
MatDelete(&p->ftsPosOffset);
MatDelete(&p->motorPos);
MatDelete(&p->motorOffset);
MatDelete(&p->ftsPos);
MatDelete(&p->stancePhaseStartPos);
MatDelete(&p->stancePhaseEndPos);
MatDelete(&p->swingPhaseVelocity);
}
/**
 * @brief Set all three kinds of velocity to struct_MC *p, with amplitude limiting.
 * The limit depends on the max time of stance phase. 
 * @param p struct_MC
 * @param val Velocity of x,y,alpha
 */
void CLASSMC_setCoMVel(struct_MC *p, float* val)
{
float xMax, yMax, aMax, swingTimeMin=p->timeForSwing.element[0][0];
for(int i=1; i<4; i++)
	swingTimeMin = (p->timeForSwing.element[i][0] < swingTimeMin) ? p->timeForSwing.element[i][0] : swingTimeMin;

xMax = 39 / (p->timeGait - swingTimeMin); // 78*1*8/16
yMax = 26.0 / (p->timeGait - swingTimeMin);
aMax = 0.8 / (p->timeGait - swingTimeMin); // 0.8=20*3.14/180*4*9/16

if(val[0] > xMax) val[0] = xMax;
else if(val[0] < -xMax) val[0] = -xMax;
if(val[1] > yMax) val[1] = yMax;
else if(val[1] < -yMax) val[1] = -yMax;
if(val[2] > aMax) val[2] = aMax;
else if(val[2] < -aMax) val[2] = -aMax;

MatSetVal(&p->targetCoMVelocity,val);
}

/**
 * 
 * @brief Regulation and control of attitude in stance phase, with the gaitMode inside struct_MC *p.
 * changing footPos
 * @param p 
 * @param legNum 
 */
void CLASSMC_attitudeRegulation(struct_MC *p, int legNum)
{
//	make body close to the surface in tort 
if( p->gaitMode == 2 || p->gaitMode == 5 || p->gaitMode == 9)//
{
	if(p->legStatus[0] == 3 || p->legStatus[1] == 3 || p->legStatus[2] == 3 || p->legStatus[3] == 3)
		p->footPos.element[legNum][2] += (StancePullHight -2.0) / p->timeForSwing.element[legNum][0] / (p->statusTimesBuffer[legNum][3] + p->statusTimesBuffer[legNum][4]);
	if(p->legStatus[0] == 4 || p->legStatus[1] == 4 || p->legStatus[2] == 4 || p->legStatus[3] == 4)
		p->footPos.element[legNum][2] += (StancePullHight -2.0) / p->timeForSwing.element[legNum][0] / (p->statusTimesBuffer[legNum][3] + p->statusTimesBuffer[legNum][4]);
	if(p->legStatus[0] == 6 || p->legStatus[1] == 6 || p->legStatus[2] == 6 || p->legStatus[3] == 6)	
		p->footPos.element[legNum][2] -= (StancePullHight -2.0) / p->timeForSwing.element[legNum][0] / p->statusTimesBuffer[legNum][6]; 
}


//	make body close to the surface in amble 
if( p->gaitMode == 3 || p->gaitMode == 10||p->gaitMode ==0X0D )	// || p->gaitMode == 5
{
	float stancePull[4]={StancePullHight,StancePullHight,StancePullHight,StancePullHight};
	int swingLeg;
	for(int i=0;i<4;i++)
	{
		if(p->legStatus[i] != 0)
		{
				stancePull[3-i] = StancePullHightDiag;	// [3-legNum] mean to diagnal leg
				swingLeg=i;
		}
	}
	
	if(p->legStatus[swingLeg]==3&&p->statusTimes[swingLeg]>0)
		p->footPos.element[legNum][2] += stancePull[legNum] /  (p->statusTimesBuffer[swingLeg][3] + p->statusTimesBuffer[swingLeg][4]);
	if(p->legStatus[swingLeg]==4&&p->statusTimes[swingLeg]>0)
		p->footPos.element[legNum][2] += stancePull[legNum] /  (p->statusTimesBuffer[swingLeg][3] + p->statusTimesBuffer[swingLeg][4]);
	if(p->legStatus[swingLeg]==6&&p->statusTimes[swingLeg]>0)
		p->footPos.element[legNum][2] -= stancePull[legNum] /  p->statusTimesBuffer[swingLeg][6]; 
}


/*	Don't use p->presentTime.
// transfer force	for trot 90 
if( p->gaitMode == 0 )
if((p->legStatus[0] + p->legStatus[1] + p->legStatus[2] + p->legStatus[3]) == 0) // all foot in stance phase
{
	// cloud run in ini
	float max = p->timeForSwingPhase.element[0][1]; 
	int maxIndex = 0; 
	for(int n=1; n<4; n++)
	{
		if(p->timeForSwingPhase.element[n][1] > max)
		{
			max = p->timeForSwingPhase.element[n][1];
			maxIndex = n;
		}					
	}// cloud run in ini
	// find the leg that would in the swing phase
	if( p->presentTime > p->timeForSwingPhase.element[maxIndex][1] )	
	{
		if(maxIndex<2)
		{
			if(legNum == maxIndex + 2)
				p->footPos.element[legNum][0] +=  transferForceX / p->timeGait * p->timePeriod *16;
		}
		else
		{
			if(legNum == maxIndex - 2)
				p->footPos.element[legNum][0] +=  transferForceX / p->timeGait * p->timePeriod *16;
		}
	}
	else
	{
		float fmin = fabs(p->presentTime - p->timeForSwingPhase.element[0][0]);
		int minIndex=0;
		for(int n=1; n<4; n++)
		{
			if(fabs(p->presentTime - p->timeForSwingPhase.element[n][0]) < fmin)
			{
				fmin = fabs(p->presentTime - p->timeForSwingPhase.element[n][0]);
				minIndex = n;
			}					
		}
		if(legNum == minIndex )
			p->footPos.element[legNum][0] +=  transferForceX / p->timeGait * p->timePeriod *16;
	}
}
// transfer force	for amble 90 	
if( p->gaitMode == 1 )
{
	if((p->legStatus[0] + p->legStatus[1] + p->legStatus[2] + p->legStatus[3]) == 0) // all foot in stance phase
	{
		// cloud run in ini
		float max = p->timeForSwingPhase.element[0][1]; 
		int maxIndex = 0; 
		for(int n=1; n<4; n++)
		{
			if(p->timeForSwingPhase.element[n][1] > max)
			{
				max = p->timeForSwingPhase.element[n][1];
				maxIndex = n;
			}					
		}
		float min = p->timeForSwingPhase.element[0][1]; 
		int minIndex = 0; 
		for(int n=1; n<4; n++)
		{
			if(p->timeForSwingPhase.element[n][1] < min)
			{
				min = p->timeForSwingPhase.element[n][1];
				minIndex = n;
			}					
		}	// cloud run in ini
		// find the leg that would in the swing phase
		if( p->presentTime > p->timeForSwingPhase.element[maxIndex][1] )	
		{
			if(legNum == minIndex)
				p->footPos.element[legNum][0] +=  transferForceX / p->timeGait * p->timePeriod *16;
		}
		else
		{
			float fmin = fabs(p->presentTime - p->timeForSwingPhase.element[0][0]);
			int minIndex=0;
			for(int n=1; n<4; n++)
			{
				if(fabs(p->presentTime - p->timeForSwingPhase.element[n][0]) < fmin)
				{
					fmin = fabs(p->presentTime - p->timeForSwingPhase.element[n][0]);
					minIndex = n;
				}					
			}
			if(legNum == minIndex )
				p->footPos.element[legNum][0] +=  transferForceX / p->timeGait * p->timePeriod *16;
		}
	}
}
*/
}
/**
 * @brief Run next step per timePeriod.
 * 
 * @param p 
 */
void CLASSMC_nextStep(struct_MC *p)
{
int stopTimes=1;
int legNum=0;
if(p->times>=0)
{
float rate[3]={1};	
float v_rate[2]={1};
float velocitybias[2]={0};
for( legNum=0; legNum<4; legNum++)  // run all 4 legs
{   
	if(fabs(p->targetCoMVelocity.element[0][0])-0.1>0&&fabs(p->targetCoMVelocity.element[1][0])-0.1<0) // x
	{
		if(legNum==0)
		{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0; //-0.2y,  -0.7 xy=2,
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
		}
		if(legNum==1)
		{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
		}
		if(legNum==2)
		{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
		}
		if(legNum==3)
		{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;   
		v_rate[1] = 0;   
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
		}
	}
	else if(fabs(p->targetCoMVelocity.element[0][0])-0.1<0&&fabs(p->targetCoMVelocity.element[1][0])-0.1>0){
		if(p->targetCoMVelocity.element[1][0]>0){//+Y
			if(legNum==0)
			{
			rate[0]=1.0;
			rate[1]=1.0;
			v_rate[0] = 0.6;
			v_rate[1] = 0; //-0.2y,  -0.7 xy=2,
			velocitybias[0]=p->timePeriod * 2;
			velocitybias[1]=p->timePeriod * 2;
			}
			if(legNum==1)
			{
			rate[0]=1;
			rate[1]=1.0;
			v_rate[0] = 0;
			v_rate[1] = -0.2;
			velocitybias[0]=p->timePeriod * 2;
			velocitybias[1]=p->timePeriod * 2;
			}
			if(legNum==2)
			{
				rate[0]=1;
				rate[1]=1;
				v_rate[0] = 0.6;
				v_rate[1] = 0.2;
				velocitybias[0]=p->timePeriod * 2;
				velocitybias[1]=p->timePeriod * 2;
			}
			if(legNum==3)
			{
				rate[0]=1;
				rate[1]=1;
				v_rate[0] = 0;   
				v_rate[1] = 0;   
				velocitybias[0]=p->timePeriod * 2;
				velocitybias[1]=p->timePeriod * 2;
			}
		}
	else if(p->targetCoMVelocity.element[1][0]<=0){//-Y
		if(legNum==0)
	{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0.25; //-0.2y,  -0.7 xy=2,
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
		if(legNum==1)
	{
		rate[0]=1;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0.5;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
		if(legNum==2)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0;
		v_rate[1] = 0.5;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
		if(legNum==3)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0;   
		v_rate[1] = 0.5;   
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
	}
}
else if(fabs(p->targetCoMVelocity.element[0][0])-0.1>0&&p->targetCoMVelocity.element[1][0]-0.1<0){
	if(legNum==0)
	{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0.5;
		v_rate[1] = 0; //-0.2y,  -0.7 xy=2,
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
	if(legNum==1)
	{
		rate[0]=1;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
			if(legNum==2)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0.5;
		v_rate[1] = 0.2;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
	if(legNum==3)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0;   
		v_rate[1] = 0.2;   
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
}
else{
			if(legNum==0)
	{
		rate[0]=1.0;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0; //-0.2y,  -0.7 xy=2,
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
	if(legNum==1)
	{
		rate[0]=1;
		rate[1]=1.0;
		v_rate[0] = 0;
		v_rate[1] = 0;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
			if(legNum==2)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0;
		v_rate[1] = 0;
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
	if(legNum==3)
	{
		rate[0]=1;
		rate[1]=1;
		v_rate[0] = 0;   
		v_rate[1] = 0;   
		velocitybias[0]=p->timePeriod * 2;
		velocitybias[1]=p->timePeriod * 2;
	}
}
	for(int i=0; i<2; i++)
	p->legComPos[legNum].element[i][0] += rate[i]*p->timePeriod * p->targetCoMVelocity.element[i][0]+v_rate[i]*velocitybias[i];
	p->legComPos[legNum].element[2][0] += p->timePeriod * p->targetCoMVelocity.element[2][0];
	if(p->legStatus[legNum] == 1)  // swing up, for compensating the distance of the downwardProbing phase
	{	
		p->statusTimes[legNum]--;
		p->isSuction[legNum]=0;
		if(p->statusTimes[legNum] < 0)
		{
			p->legStatus[legNum] = 2;
			p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[2]-p->swingStatusTimeFactor[1]));
			p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
			goto NEXTLEG2;
		}
		p->ftsPos.element[legNum][0] -= p->downwardProbingDis[legNum][0]/p->statusTimesBuffer[legNum][p->legStatus[legNum]];
		p->ftsPos.element[legNum][1] -= p->downwardProbingDis[legNum][1]/p->statusTimesBuffer[legNum][p->legStatus[legNum]];// l-f different // need changed
		p->ftsPos.element[legNum][2] +=  TrialCorrectionHeight / p->statusTimesBuffer[legNum][p->legStatus[legNum]];
	}					
		else if(p->legStatus[legNum] == 2)  // swing down, for compensating the distance of the stance phase
		{
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 3;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[3]-p->swingStatusTimeFactor[2]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto NEXTLEG2;
			}
			float t = p->statusTimesBuffer[legNum][2] + p->statusTimesBuffer[legNum][3];
			for(int i=0; i<3; i++)
				p->swingPhaseVelocity.element[legNum][i] = (p->stancePhaseStartPos.element[legNum][i] - p->stancePhaseEndPos.element[legNum][i]) / t;
			float upStepX = fabsf(p->stancePhaseEndPos.element[legNum][0] - p->stancePhaseStartPos.element[legNum][0]) * p->statusTimesBuffer[legNum][2] / t;   
			float upStepY = fabsf(p->stancePhaseEndPos.element[legNum][1] - p->stancePhaseStartPos.element[legNum][1]) * p->statusTimesBuffer[legNum][2] / t;
			
			p->ftsPos.element[legNum][0] += p->swingPhaseVelocity.element[legNum][0] ;	// compress distance(Velocity) in swing phase to 0.6*Tsw 
			p->ftsPos.element[legNum][1] += p->swingPhaseVelocity.element[legNum][1] ;

			/*	ftsPos.element[legNum][2] absolute value control */
			float y=fabsf(p->ftsPos.element[legNum][1] - p->stancePhaseEndPos.element[legNum][1]) - upStepY;
			float x=fabsf(p->ftsPos.element[legNum][0] - p->stancePhaseEndPos.element[legNum][0]) - upStepX;
			if(upStepY == 0 && upStepX==0)
				p->ftsPos.element[legNum][2] += p->stepHight[legNum] / p->statusTimesBuffer[legNum][p->legStatus[legNum]];
			else
				p->ftsPos.element[legNum][2] = -1 * p->stepHight[legNum]/(pow(upStepX,2)+pow(upStepY,2))*(pow(x,2)+pow(y,2)) + p->stepHight[legNum] + p->stancePhaseEndPos.element[legNum][2];	

		}
		else if(p->legStatus[legNum] == 3)  // preAttach phase
		{	
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 4;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[4]-p->swingStatusTimeFactor[3]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto NEXTLEG2;
			}
			p->ftsPos.element[legNum][0] += p->swingPhaseVelocity.element[legNum][0];
			p->ftsPos.element[legNum][1] += p->swingPhaseVelocity.element[legNum][1];
			p->ftsPos.element[legNum][2] -= p->stepHight[legNum] / p->statusTimesBuffer[legNum][p->legStatus[legNum]];
		}
		else if(p->legStatus[legNum] == 4)  // Attach phase
		{				
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0 )
			{
				p->legStatus[legNum] = 5;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[5]-p->swingStatusTimeFactor[4]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto NEXTLEG2;
			}

			p->ftsPos.element[legNum][0] -= p->timePeriod * p->targetCoMVelocity.element[0][0];	//	make the foot stationary relative to the ground on x, y
			p->ftsPos.element[legNum][1] -= p->timePeriod * p->targetCoMVelocity.element[1][0];
			p->ftsPos.element[legNum][2] -= (p->pressHight[legNum]) / p->statusTimesBuffer[legNum][p->legStatus[legNum]];//	down	
		

			if(adc[legNum]<pressureThreshold[legNum])
			p->isSuction[legNum]++;
			else p->isSuction[legNum]=0; 
		}
		
		 else if(p->legStatus[legNum] == 5)
		{
		 	p->statusTimes[legNum]--;
		 	if(p->statusTimes[legNum] < 0)
		 	{
		 			if(p->gaitMode!=0x0C)
		 		{
		 			if(p->isSuction[legNum]<3)
		 				controlRunFlag=0;
		 			else
						sendAdhesionOnInf(legNum);
		 		}
		 		p->legStatus[legNum] = 6;
		 		p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[6]-p->swingStatusTimeFactor[5]));
		 		p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
		 		goto NEXTLEG2;
		 	}
			if(adc[legNum]<pressureThreshold[legNum]) ///????
		 	p->isSuction[legNum]++;
		 	else p->isSuction[legNum]=0;
		}
		else if(p->legStatus[legNum] == 6)
		{
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 0;
				p->angleForShoulder[legNum] =  0;
				p->statusTimes[legNum] = floor((p->timeGait - p->timeForSwing.element[legNum][0]) / p->timePeriod);
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				/*	record the fst position of stance phase on start		*/
				for(int i=0; i<3; i++)
					p->stancePhaseStartPos.element[legNum][i] = p->ftsPos.element[legNum][i];   
				p->footPos.element[legNum][0] = p->ftsPos.element[legNum][0] + p->shoulderPos.element[legNum][0] + p->legComPos[legNum].element[0][0];
				p->footPos.element[legNum][1] = p->ftsPos.element[legNum][1] + p->shoulderPos.element[legNum][1] + p->legComPos[legNum].element[1][0];	
				p->footPos.element[legNum][2] = p->ftsPos.element[legNum][2] + p->shoulderPos.element[legNum][2];	
				goto NEXTLEG2;
			}	
			p->ftsPos.element[legNum][0] += p->timePeriod * p->targetCoMVelocity.element[0][0];
			p->ftsPos.element[legNum][1] += p->timePeriod * p->targetCoMVelocity.element[1][0];
			p->ftsPos.element[legNum][2] += (p->pressHight[legNum]) / p->statusTimesBuffer[legNum][p->legStatus[legNum]];//					up				
		}		
		else if(p->legStatus[legNum] == 0)
		{  
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 1;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[1]-p->swingStatusTimeFactor[0]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				for(int i=0; i<3; i++)
					p->stancePhaseEndPos.element[legNum][i] = p->ftsPos.element[legNum][i];		
				goto NEXTLEG2;
			}				
			p->angleForShoulder[legNum] += p->timePeriod * p->targetCoMVelocity.element[2][0];
			Mat trans;
			MatCreate(&trans, 4, 4);		
			float t_trans[]={cos(p->angleForShoulder[legNum]), -sin(p->angleForShoulder[legNum]), 0, p->legComPos[legNum].element[0][0],
					sin(p->angleForShoulder[legNum]), cos(p->angleForShoulder[legNum]), 0, p->legComPos[legNum].element[1][0],
					0, 0, 1, 0,
					0, 0, 0, 1};
			MatSetVal(&trans,t_trans);	
											
			Mat tempMat_4x1;
			MatCreate(&tempMat_4x1, 4, 1);
			float oneShoulderPos_4x1[]={p->shoulderPos.element[legNum][0], p->shoulderPos.element[legNum][1], 0, 1};
			MatSetVal(&tempMat_4x1,oneShoulderPos_4x1);
			MatMul(&trans, &tempMat_4x1, &tempMat_4x1);		

		/*	pull back the distance that press on z in the end of swing phase 	*/
		// if(p->times !=0)		
		// {
		// 	if(p->presentTime > p->timeForSwingPhase.element[legNum][1] + (p->timeGait - p->timeForSwing.element[legNum][0]) * 0.1 && p->presentTime  < p->timeForSwingPhase.element[legNum][1] + (p->timeGait - p->timeForSwing.element[legNum][0]) * 0.2)
		// 		p->footPos.element[legNum][2] += p->pressHight[legNum] / (0.2-0.1) / (p->timeGait - p->timeForSwing.element[legNum][0]) * p->timePeriod; // 7.5 / 0.2 / (p->timeGait - p->timeForSwing.element[legNum][0]) 
		// 	else if(p->presentTime > p->timeForSwingPhase.element[legNum][0] - (p->timeGait - p->timeForSwing.element[legNum][0]) * 0.9 && p->presentTime  < p->timeForSwingPhase.element[legNum][0] - (p->timeGait - p->timeForSwing.element[legNum][0]) * 0.8)
		// 		p->footPos.element[legNum][2] += p->pressHight[legNum] / (0.9-0.8) / (p->timeGait - p->timeForSwing.element[legNum][0]) * p->timePeriod;
		// }
			CLASSMC_attitudeRegulation(p, legNum);
			
			/*	update ftsPos	*/
			p->ftsPos.element[legNum][0] = p->footPos.element[legNum][0] - tempMat_4x1.element[0][0];  // X
			p->ftsPos.element[legNum][1] = p->footPos.element[legNum][1] - tempMat_4x1.element[1][0];  // Y
			p->ftsPos.element[legNum][2] = p->footPos.element[legNum][2] - p->shoulderPos.element[legNum][2];	
			
			MatDelete(&tempMat_4x1);
			MatDelete(&trans);
		} // stance phase
		NEXTLEG2:
		p->legRunTimes[legNum]++;
		p->legRunTimesBuffer[legNum] = 0;
		for (int k = 0; k < 7; k++)
			p->legRunTimesBuffer[legNum] += p->statusTimesBuffer[legNum][k];
	} 
}	
p->presentTime += p->timePeriod;
if(p->legRunTimes[0] >= p->legRunTimesBuffer[0])
if(p->legRunTimes[1] >= p->legRunTimesBuffer[1])
if(p->legRunTimes[2] >= p->legRunTimesBuffer[2])
if(p->legRunTimes[3] >= p->legRunTimesBuffer[3])
{
	for (int i = 0; i < 4; i++)
		p->legRunTimes[i] = 0;
	p->presentTime = 0.0;
	p->times++;
}
}


/**
 * @brief Deliver parameters from p to trans, and init the gait of p with gaitMode
 * 
 * @param p 
 * @param trans the struct_MC of transitionStep
 * @param gaitMode init the gait of p
 */
void CLASSMC_transParamDeliver(struct_MC *p, struct_MC *trans, int gaitMode)
{
MatCopy(&p->ftsPos, &trans->ftsPos);
MatAdd(&trans->ftsPos, &trans->shoulderPos, &trans->footPos);
MatCopy(&trans->ftsPos, &trans->stancePhaseEndPos);
CLASSMC_initiation(p, gaitMode);	
MatCopy(&p->ftsPosOffset, &trans->stancePhaseStartPos);
MatCopy(&p->ftsPosOffset, &trans->ftsPosOffset);
for(int i=0; i<3; i++)
	targetCoMVelocityBuffer[i] = trans->targetCoMVelocity.element[i][0];
}

/**
 * @brief Tasnsit ftsPos from stancePhaseEndPos to stancePhaseStartPos.
 * 
 * @param p struct_MC*
 */
void CLASSMC_transitionStep(struct_MC *p)
{ 
	for(int legNum=0; legNum<4; legNum++)  // run all 4 legs
	{   			
		float t = p->statusTimesBuffer[legNum][1] + p->statusTimesBuffer[legNum][2];
		for(int i=0; i<3; i++)
			p->swingPhaseVelocity.element[legNum][i] = (p->stancePhaseStartPos.element[legNum][i] - p->stancePhaseEndPos.element[legNum][i]) / t;				
		if(p->legStatus[legNum] == 1)
		{
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 2;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[2]-p->swingStatusTimeFactor[1]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}
			for(int i=0; i<3; i++)
				p->ftsPos.element[legNum][i] += p->swingPhaseVelocity.element[legNum][i];
			p->ftsPos.element[legNum][2] += p->stepHight[legNum] / p->statusTimesBuffer[legNum][1];
		}
		else if(p->legStatus[legNum] == 2)
		{				
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 3;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[3]-p->swingStatusTimeFactor[2]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}
			for(int i=0; i<3; i++)
				p->ftsPos.element[legNum][i] += p->swingPhaseVelocity.element[legNum][i];
			p->ftsPos.element[legNum][2] -= p->stepHight[legNum] / p->statusTimesBuffer[legNum][2];
		}
		else if(p->legStatus[legNum] == 3)
		{							
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 4;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[4]-p->swingStatusTimeFactor[3]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}	
			p->ftsPos.element[legNum][2] -= p->pressHight[legNum] / p->statusTimesBuffer[legNum][3];//	down			
		}
		else if(p->legStatus[legNum] == 4)
		{
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 5;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[5]-p->swingStatusTimeFactor[4]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}
		}
		else if(p->legStatus[legNum] == 5)
		{
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 0;
				p->angleForShoulder[legNum] =  0;
				p->statusTimes[legNum] = floor((p->timeGait - p->timeForSwing.element[legNum][0]) / p->timePeriod);

				p->footPos.element[legNum][0] = p->ftsPos.element[legNum][0] + p->shoulderPos.element[legNum][0] + p->legComPos[legNum].element[0][0];
				p->footPos.element[legNum][1] = p->ftsPos.element[legNum][1] + p->shoulderPos.element[legNum][1] + p->legComPos[legNum].element[1][0];	
				p->footPos.element[legNum][2] = p->ftsPos.element[legNum][2] + p->shoulderPos.element[legNum][2];	
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}
			p->ftsPos.element[legNum][2] += p->pressHight[legNum] / p->statusTimesBuffer[legNum][5];//					up				
		}		
		else if(p->legStatus[legNum] == 0)
		{      
			p->statusTimes[legNum]--;
			if(p->statusTimes[legNum] < 0)
			{
				p->legStatus[legNum] = 1;
				p->statusTimes[legNum] = floor(p->timeForSwing.element[legNum][0] / p->timePeriod * (p->swingStatusTimeFactor[1]-p->swingStatusTimeFactor[0]));
				p->statusTimesBuffer[legNum][p->legStatus[legNum]] = p->statusTimes[legNum];
				goto T_NEXTLEG;
			}
			CLASSMC_attitudeRegulation(p, legNum);
			/*	update ftsPos	*/
			for(int i=0; i<3; i++)
				p->ftsPos.element[legNum][i] = p->footPos.element[legNum][i] - p->shoulderPos.element[legNum][i];
		} // stance phase
		T_NEXTLEG:
		p->legRunTimes[legNum]++;
		p->legRunTimesBuffer[legNum] = 0;
		for (int k = 0; k <7; k++)
			p->legRunTimesBuffer[legNum] += p->statusTimesBuffer[legNum][k];
	} 
	
p->presentTime += p->timePeriod;
// if(fabs(p->presentTime - p->timeGait) < 1e-3)  // check if present time has reach the gait period                                                               
// {                                                    
// 	p->presentTime = 0.0;
// 	p->times++;
// }
if(p->legRunTimes[0] >= p->legRunTimesBuffer[0])
if(p->legRunTimes[1] >= p->legRunTimesBuffer[1])
if(p->legRunTimes[2] >= p->legRunTimesBuffer[2])
if(p->legRunTimes[3] >= p->legRunTimesBuffer[3])
{
	for (int i = 0; i < 4; i++)
		p->legRunTimes[i] = 0;
	p->presentTime = 0.0;
	p->times++;
}
}

void fts(struct_MC *p)
{
for(int i = 0; i<4; i++)
{
	for(int j = 0; j<3; j++)
	{
		p->ftsPosition[i*3+j] = p->ftsPos.element[i][j];
	}
}
}
/**
 * @brief Calculate motorPos with ftsPos and motorOffset
 * 
 * @param p 
 */
void CLASSMC_inverseKinematics(struct_MC *p) //ftsPos -> motorPos
{   
//===================姿态调节==================//
// 1. 先执行地形估计 (或者放在主循环 runOneStep 里)
TerrainAdaptation(p); 

// 构建旋转矩阵 (基于 estimated_roll, estimated_pitch)
// R_x (Roll) * R_y (Pitch)
float cr = cosf(estimated_roll);
float sr = sinf(estimated_roll);
float cp = cosf(estimated_pitch);
float sp = sinf(estimated_pitch);

// 简化的旋转矩阵应用：
// 新的足端位置 P_new = R_inv * P_old
// 机身转，相当于足端向反方向转
//========================================//

for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
{
	// 原始足端位置 (相对于机身中心)
	float x = p->ftsPos.element[legNum][0] + p->shoulderPos.element[legNum][0];
	float y = p->ftsPos.element[legNum][1] + p->shoulderPos.element[legNum][1];
	float z = p->ftsPos.element[legNum][2] + p->shoulderPos.element[legNum][2];

	// 应用旋转 (绕机身中心旋转)
	// 这里做一个近似的 3D 旋转变换
	// P_rotated = R_y(-pitch) * R_x(-roll) * P_origin
	
	float y_new = y * cr - z * sr;
	float z_temp = y * sr + z * cr;
	
	float x_new = x * cp + z_temp * sp;
	float z_new = -x * sp + z_temp * cp;

	// 转换回 ftsPos (相对于肩部)
	// 这一步计算出的 motor_target_x/y/z 是考虑了机身倾斜后的“虚拟”足端目标
	// 注意：这里不直接修改 p->ftsPos，而是用临时变量计算逆解
	float eff_x = x_new - p->shoulderPos.element[legNum][0];
    float eff_y = y_new - p->shoulderPos.element[legNum][1];
    float eff_z = z_new - p->shoulderPos.element[legNum][2];

	float factor_y, factor_x, factor_xc, factor_yc, factor_zc;  // factor for x/y; factor for whole formula
	if(legNum==0)
	{
		factor_xc=-1;
		factor_yc=1;
		factor_zc=1;
		factor_x=1;
		factor_y=1;
		p->L2 = p->L2_f;
	}
	if(legNum==1)
	{
		factor_xc=1;
		factor_yc=-1;
		factor_zc=-1;
		factor_x=1;
		factor_y=-1;
		p->L2 = p->L2_f;
	}
	if(legNum==2)
	{
		factor_xc=-1;
		factor_yc=-1;
		factor_zc=-1;
		factor_x=-1;
		factor_y=1;
		p->L2 = p->L2_h;
	}
	if(legNum==3)
	{
		factor_xc=1;
		factor_yc=1;
		factor_zc=1;
		factor_x=-1;
		factor_y=-1;
		p->L2 = p->L2_h;
	}
	p->motorPos.element[legNum][1] = p->motorOffset.element[legNum][1] - factor_xc * (asin(p->L3 / sqrtf( eff_z* eff_z + eff_y* eff_y )) + atan2( eff_z, factor_y * eff_y) );     
    p->motorPos.element[legNum][0] = p->motorOffset.element[legNum][0] - factor_yc * (asin(( eff_y * eff_y + eff_x * eff_x + eff_z * eff_z + p->L1 * p->L1 - p->L2 * p->L2 - p->L3 * p->L3) / ( 2 * p->L1 * sqrtf ( eff_y * eff_y + eff_x * eff_x + eff_z * eff_z - p->L3 * p->L3)))
                        - atan2(sqrtf( eff_y * eff_y + eff_z * eff_z - p->L3 * p->L3) , factor_x * eff_x));
    p->motorPos.element[legNum][2] = p->motorOffset.element[legNum][2] - factor_zc * asin((p->L1 * p->L1 + p->L2 * p->L2 + p->L3 * p->L3 - eff_y * eff_y - eff_x * eff_x - eff_z * eff_z) / (2 * p->L1 * p->L2));
	//p->motorPos.element[legNum][1] = p->motorOffset.element[legNum][1] - factor_xc * (asin(p->L3 / sqrtf( p->ftsPos.element[legNum][2]* p->ftsPos.element[legNum][2] + p->ftsPos.element[legNum][1]* p->ftsPos.element[legNum][1] )) + atan2( p->ftsPos.element[legNum][2],factor_y * p->ftsPos.element[legNum][1]) );     
	//p->motorPos.element[legNum][0] = p->motorOffset.element[legNum][0] - factor_yc * (asin(( p->ftsPos.element[legNum][1] * p->ftsPos.element[legNum][1] + p->ftsPos.element[legNum][0] *  p->ftsPos.element[legNum][0] + p->ftsPos.element[legNum][2] * p->ftsPos.element[legNum][2] + p->L1 * p->L1 - p->L2 * p->L2 - p->L3 * p->L3) / ( 2 * p->L1 * sqrtf ( p->ftsPos.element[legNum][1] * p->ftsPos.element[legNum][1] + p->ftsPos.element[legNum][0] * p->ftsPos.element[legNum][0] + p->ftsPos.element[legNum][2] * p->ftsPos.element[legNum][2] - p->L3 * p->L3)))
					//- atan2(sqrtf( p->ftsPos.element[legNum][1] * p->ftsPos.element[legNum][1] + p->ftsPos.element[legNum][2] * p->ftsPos.element[legNum][2] - p->L3 * p->L3) , factor_x * p->ftsPos.element[legNum][0]));
	//p->motorPos.element[legNum][2] = p->motorOffset.element[legNum][2] - factor_zc * asin((p->L1 * p->L1 + p->L2 * p->L2 + p->L3 * p->L3 - p->ftsPos.element[legNum][1] * p->ftsPos.element[legNum][1] - p->ftsPos.element[legNum][0] * p->ftsPos.element[legNum][0] - p->ftsPos.element[legNum][2] * p->ftsPos.element[legNum][2]) / (2 * p->L1 * p->L2));
}
}

void CLASSMC_setJointPosition(struct_MC *p)
{
float theta[4][2]={0};

theta[0][0]=(p->motorPos.element[0][0]+p->motorPos.element[0][1]);
theta[0][1]=(p->motorPos.element[0][0]-p->motorPos.element[0][1]);
theta[1][0]=(p->motorPos.element[1][0]+p->motorPos.element[1][1]);
theta[1][1]=(p->motorPos.element[1][0]-p->motorPos.element[1][1]);
theta[2][0]=(p->motorPos.element[2][0]-p->motorPos.element[2][1]);
theta[2][1]=(p->motorPos.element[2][0]+p->motorPos.element[2][1]);
theta[3][0]=(p->motorPos.element[3][0]-p->motorPos.element[3][1]);
theta[3][1]=(p->motorPos.element[3][0]+p->motorPos.element[3][1]);
int16_t ArrValue[12] = {0};
int16_t angle[12] = {0};
for(int i = 0; i<4; i++)
{
	for(int j = 0; j<2; j++)
	{
		p->jointPos[i*3+j] = theta[i][j];       
	}
}

for(int i = 0; i<4; i++)
{
	p->jointPos[i*3+2] = p->motorPos.element[i][2];  
}

for(int i=0; i<12; i++)
{
	angle[i] =  p->jointPos[i]*180/PI;  // minus for inverse direction
}	


for(int i=0; i<12; i++)
{
	// ArrValue[i] = (int32_t) (angle[i] * 5.56 + 750);	// HS-5065: 0-180, 2.5%-12.5%.  minus
	ArrValue[i] = (int32_t) (angle[i] * 43.05 + 5125);	// Coreless motor A1: 0-180, 12.5%-90%. 
}

/*	PCB_3.3	*/
// TIM_SetCompare1(&TIM4_Handler,ArrValue[2]);	//LF
// TIM_SetCompare2(&TIM4_Handler,ArrValue[1]);
// TIM_SetCompare3(&TIM4_Handler,ArrValue[0]);
// TIM_SetCompare1(&TIM12_Handler,ArrValue[8]);	//LH
// TIM_SetCompare2(&TIM12_Handler,ArrValue[7]);
// TIM_SetCompare1(&TIM15_Handler,ArrValue[6]);
// TIM_SetCompare1(&TIM1_Handler,ArrValue[5]);	//RF
// TIM_SetCompare2(&TIM1_Handler,ArrValue[4]);
// TIM_SetCompare3(&TIM1_Handler,ArrValue[3]);
// TIM_SetCompare1(&TIM3_Handler,ArrValue[11]);	//RH
// TIM_SetCompare2(&TIM3_Handler,ArrValue[10]);
// TIM_SetCompare3(&TIM3_Handler,ArrValue[9]);

/*	PCB_3.4	*/
//	 TIM_SetCompare1(&TIM4_Handler,ArrValue[2]);	//LF
//	 TIM_SetCompare2(&TIM4_Handler,ArrValue[1]);
//	 TIM_SetCompare3(&TIM4_Handler,ArrValue[0]);
//	 TIM_SetCompare1(&TIM12_Handler,ArrValue[5]);	//RF
//	 TIM_SetCompare2(&TIM12_Handler,ArrValue[4]);
//	 TIM_SetCompare1(&TIM15_Handler,ArrValue[3]);
//	 TIM_SetCompare1(&TIM1_Handler,ArrValue[8]);	//LH
//	 TIM_SetCompare2(&TIM1_Handler,ArrValue[7]);
//	 TIM_SetCompare3(&TIM1_Handler,ArrValue[6]);
//	 TIM_SetCompare1(&TIM3_Handler,ArrValue[11]);	//RH
//	 TIM_SetCompare2(&TIM3_Handler,ArrValue[10]);
//	 TIM_SetCompare3(&TIM3_Handler,ArrValue[9]);
	/*	PCB_3.5	*/
//	TIM_SetCompare1(&TIM4_Handler,ArrValue[8]);	//LF
//	TIM_SetCompare2(&TIM4_Handler,ArrValue[7]);
//	TIM_SetCompare3(&TIM4_Handler,ArrValue[6]);
//	TIM_SetCompare1(&TIM12_Handler,ArrValue[11]);	//RF
//	TIM_SetCompare2(&TIM12_Handler,ArrValue[10]);
//	TIM_SetCompare1(&TIM15_Handler,ArrValue[9]);
//	TIM_SetCompare1(&TIM1_Handler,ArrValue[2]);	//LH
//	TIM_SetCompare2(&TIM1_Handler,ArrValue[1]);
//	TIM_SetCompare3(&TIM1_Handler,ArrValue[0]);
//	TIM_SetCompare1(&TIM3_Handler,ArrValue[5]);	//RH
//	TIM_SetCompare2(&TIM3_Handler,ArrValue[4]);
//	TIM_SetCompare3(&TIM3_Handler,ArrValue[3]);
	/* PCB_3.6_V6 */
	TIM_SetCompare1(&TIM4_Handler,ArrValue[5]);	//LF
	TIM_SetCompare2(&TIM4_Handler,ArrValue[4]);
	TIM_SetCompare3(&TIM4_Handler,ArrValue[3]);
	TIM_SetCompare1(&TIM12_Handler,ArrValue[2]);	//RF
	TIM_SetCompare2(&TIM12_Handler,ArrValue[1]);
	TIM_SetCompare1(&TIM15_Handler,ArrValue[0]);
	TIM_SetCompare1(&TIM1_Handler,ArrValue[11]);	//LH
	TIM_SetCompare2(&TIM1_Handler,ArrValue[10]);
	TIM_SetCompare3(&TIM1_Handler,ArrValue[9]);
	TIM_SetCompare1(&TIM3_Handler,ArrValue[8]);	//RH
	TIM_SetCompare2(&TIM3_Handler,ArrValue[7]);
	TIM_SetCompare3(&TIM3_Handler,ArrValue[6]);
}

/**
 * @brief Update parameter of gait with Global Variables which extern declare in gait.h, 
 * at the beginning of a new cycle, and the commands come from VOFA+.
 * 
 * @param p the address of struct CreepMotionControl p
 */
void updateGaitParameter(struct_MC *p)
{
float offset[12];

switch (statusNum)
{
// case 1:
// 	p->targetCoMVelocity.element[0][0] = targetCoMVelocityBuffer[0];	//x
// 	break;
// case 2:
// 	p->targetCoMVelocity.element[1][0] = targetCoMVelocityBuffer[1];	//y
// 	break;
// case 3:
// 	p->targetCoMVelocity.element[2][0] = targetCoMVelocityBuffer[1];	//y
// 	break;
case 4:
	p->timeGait = timeGaitBuffer;
	break;
case 5:
	for(int i=0; i<4; i++)
		p->pressHight[i] = pressHightBuffer[i];
	break;
case 6:
	CLASSMC_initiation(&transition, gaitModeBuffer);
	CLASSMC_transParamDeliver(p, &transition, gaitModeBuffer);
	break;
default:
	break;
}
}
/**
 * @brief air pressure control, inverse kinematics and set joint position of motors 
 * 
 * @param p 
 */
void runOneStep(struct_MC *p)
{
//	if(p->gaitMode == 0 || p->gaitMode == 2 || p->gaitMode == 4 || p->gaitMode == 6
//		|| p->gaitMode == 9 )
//		air_control_trot(p);
//	if(p->gaitMode == 1 || p->gaitMode == 3 || p->gaitMode == 5 || p->gaitMode == 7
//		|| p->gaitMode == 8 || p->gaitMode == 10)
//		air_control_amble(p);
air_control_status(p,0);
CLASSMC_inverseKinematics(p);
CLASSMC_setJointPosition(p);
	
}


void downwardProbing(struct_MC *p)
{
static int probingTimesLeft=ProbingTimes;
if(adc[p->swingLeg]<pressureThreshold[p->swingLeg]) //judge
p->isSuction[p->swingLeg]++;
else p->isSuction[p->swingLeg]=0;
if(p->isSuction[p->swingLeg]<3)
{
	if(isSecendTrial==0)
	{
	p->ftsPos.element[p->swingLeg][0] += ProbingDistanceX/ProbingTimes;
	p->downwardProbingDis[p->swingLeg][0] += ProbingDistanceX/ProbingTimes;
	if(p->swingLeg==0||p->swingLeg==2)
	{
	p->ftsPos.element[p->swingLeg][1] += ProbingDistanceYL/ProbingTimes;
	p->downwardProbingDis[p->swingLeg][1] += ProbingDistanceYL/ProbingTimes;
	}
	else
	{
	p->ftsPos.element[p->swingLeg][1] += ProbingDistanceYR/ProbingTimes;
	p->downwardProbingDis[p->swingLeg][1] += ProbingDistanceYR/ProbingTimes;
	}

	p->ftsPos.element[p->swingLeg][2] -= ProbingDistanceZ/ProbingTimes;
	p->downwardProbingDis[p->swingLeg][2] += ProbingDistanceZ/ProbingTimes;
	probingTimesLeft--;
	if(probingTimesLeft==0) {isSecendTrial=1;isSecendUp=1;probingTimesLeft=ProbingTimes;}
	}
	else
	{
		if(isSecendUp==1)
		{
			p->ftsPos.element[p->swingLeg][0] -= p->downwardProbingDis[p->swingLeg][0]/ProbingTimes;
			p->ftsPos.element[p->swingLeg][1] -= p->downwardProbingDis[p->swingLeg][1] /ProbingTimes;
			p->ftsPos.element[p->swingLeg][2] += p->downwardProbingDis[p->swingLeg][2]/ProbingTimes;
			probingTimesLeft--;
			if(probingTimesLeft==0) {isSecendUp=0;probingTimesLeft=ProbingTimes;
				p->downwardProbingDis[p->swingLeg][0]=0;
				p->downwardProbingDis[p->swingLeg][1]=0;
				p->downwardProbingDis[p->swingLeg][2]=0;
			}
		}
		else
		{
		p->ftsPos.element[p->swingLeg][0] += ProbingDistanceX2/ProbingTimes;
		p->downwardProbingDis[p->swingLeg][0] += ProbingDistanceX2/ProbingTimes;
		if(p->swingLeg==0||p->swingLeg==2)
		{
		p->ftsPos.element[p->swingLeg][1] += ProbingDistanceYL2/ProbingTimes;
		p->downwardProbingDis[p->swingLeg][1] += ProbingDistanceYL2/ProbingTimes;
		}
		else
		{
		p->ftsPos.element[p->swingLeg][1] += ProbingDistanceYR2/ProbingTimes;
		p->downwardProbingDis[p->swingLeg][1] += ProbingDistanceYR2/ProbingTimes;
		}

		p->ftsPos.element[p->swingLeg][2] -= ProbingDistanceZ2/ProbingTimes;
		p->downwardProbingDis[p->swingLeg][2] += ProbingDistanceZ2/ProbingTimes;
		probingTimesLeft--;
		if(probingTimesLeft==0) {isSecendTrial=0;isSecendUp=0;probingTimesLeft=ProbingTimes;controlRunFlag=0;isProbingMode=0;
			//debug open
			//isProbingMode=false;controlRunFlag=1;
		}
	}
}
}
else
{
	isProbingMode=false;
	sendAdhesionOnInf(p->swingLeg);
	probingTimesLeft=ProbingTimes;
	isSecendTrial=0;
	isSecendUp=0;
}
}