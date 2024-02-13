/*
 * ruka_joints.h
 *
 *  Created on: Jan 31, 2024
 *      Author: VR
 */

#ifndef INC_RUKA_JOINTS_H_
#define INC_RUKA_JOINTS_H_

#include "main.h"

#define AGENT_JS_SUB_PORT 1001

#define JOINT_N_1 101
#define JOINT_N_2 102
#define JOINT_N_3 103
#define JOINT_N_4 104
#define JOINT_N_5 105
#define JOINT_N_6 106

#define JS_PUB_1_PORT_ID 1111
#define JS_SUB_1_PORT_ID 1121

#define JS_PUB_2_PORT_ID 1112
#define JS_SUB_2_PORT_ID 1122

#define JS_PUB_3_PORT_ID 1113
#define JS_SUB_3_PORT_ID 1123

#define JS_PUB_4_PORT_ID 1114
#define JS_SUB_4_PORT_ID 1124

#define JS_PUB_5_PORT_ID 1115
#define JS_SUB_5_PORT_ID 1125

#define JS_PUB_6_PORT_ID 1116
#define JS_SUB_6_PORT_ID 1126

#define AGENT_IMU_PORT 1337

//#define JS_PUB_PORT_ID
//#define JS_SUB_PORT_ID

#if JOINT_N == 1
#define JS_PUB_PORT_ID JS_PUB_1_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_1_PORT_ID
#endif

#if JOINT_N == 2
#define JS_PUB_PORT_ID JS_PUB_2_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_2_PORT_ID
#endif

#if JOINT_N == 3
#define JS_PUB_PORT_ID JS_PUB_3_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_3_PORT_ID
#endif

#if JOINT_N == 4
#define JS_PUB_PORT_ID JS_PUB_4_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_4_PORT_ID
#endif

#if JOINT_N == 5
#define JS_PUB_PORT_ID JS_PUB_5_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_5_PORT_ID
#endif


#if JOINT_N == 6
#define JS_PUB_PORT_ID JS_PUB_6_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_6_PORT_ID
#endif


#endif /* INC_RUKA_JOINTS_H_ */
