#ifndef __GIMBAL_FSM_H
#define __GIMBAL_FSM_H
#include "fsm.h"


FSM_t *Return_Gimbal_FSM(void);
void Gimbal_Fsm_Init(void);

static void Offline_Prepare(void);
static void Offline_State(void);
static void PowerOff_bhv(void);

static void Lift_Prepare(void);
static void Lift_State(void);
static void Lift_bhv(void);

static void Arm_Prepare(void);
static void Arm_State(void);
static void Arm_bhv(void);

static void Mining_Prepare(void);
static void Mining_State(void);
static void Mining_bhv(void);


static void KeyBoard_Prepare(void);
static void KeyBoard_State(void);
static void KeyBoard_bhv(void);

#endif // !__GIMBAL_FSM_H

