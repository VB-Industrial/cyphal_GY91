#include "main.h"
#include "ruka_joints.h"

#include <memory>
#include <cstdio>

#include "cyphal/cyphal.h"
#include "cyphal/providers/G4CAN.h"
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/subscriptions/subscription.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/scalar/Integer32_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/Twist_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/State_0_1.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>

#include <uavcan/node/GetInfo_1_0.h>

extern "C" {

extern IWDG_HandleTypeDef hiwdg;

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(State, reg_udral_physics_kinematics_cartesian_State_0_1)


TYPE_ALIAS(RegisterListRequest, uavcan_register_List_Request_1_0)
TYPE_ALIAS(RegisterListResponse, uavcan_register_List_Response_1_0)

TYPE_ALIAS(RegisterAccessRequest, uavcan_register_Access_Request_1_0)
TYPE_ALIAS(RegisterAccessResponse, uavcan_register_Access_Response_1_0)


std::byte buffer[sizeof(CyphalInterface) + sizeof(G4CAN) + sizeof(SystemAllocator)];
std::shared_ptr<CyphalInterface> interface;

void error_handler() { Error_Handler(); }
uint64_t micros_64() { return HAL_GetTick() * 1000; }
UtilityConfig utilities(micros_64, error_handler);

class HBeatReader: public AbstractSubscription<HBeat> {
public:
    HBeatReader(InterfacePtr interface): AbstractSubscription<HBeat>(interface,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const uavcan_node_Heartbeat_1_0& hbeat, CanardRxTransfer* transfer) override {
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    	HAL_IWDG_Refresh(&hiwdg);
    }
};


HBeatReader* h_reader;


class RegisterListReader : public AbstractSubscription<RegisterListRequest> {
public:
    RegisterListReader(InterfacePtr interface): AbstractSubscription<RegisterListRequest>(
        interface,
        uavcan_register_List_1_0_FIXED_PORT_ID_,
        CanardTransferKindRequest
    ) {};
    void handler(const RegisterListRequest::Type&, CanardRxTransfer*) override;
};

class RegisterAccessReader : public AbstractSubscription<RegisterAccessRequest> {
public:
    RegisterAccessReader(InterfacePtr interface): AbstractSubscription<RegisterAccessRequest>(
        interface,
        uavcan_register_Access_1_0_FIXED_PORT_ID_,
        CanardTransferKindRequest
    ) {};
    void handler(const RegisterAccessRequest::Type&, CanardRxTransfer*) override;
};



void send_IMU(float* qw, float* qx, float* qy, float* qz, float* ax, float* ay, float* az, float* gx, float* gy, float* gz)
{
	static uint8_t state_buffer[State::buffer_size];
	static CanardTransferID int_transfer_id = 0;

	reg_udral_physics_kinematics_cartesian_Pose_0_1 imu_pose;
	imu_pose.orientation = {*qw, *qx, *qy, *qz};

	reg_udral_physics_kinematics_cartesian_Twist_0_1 imu_twist;
	imu_twist.angular = {*ax, *ay, *az};
	imu_twist.linear = {*gx, *gy, *gz};

	reg_udral_physics_kinematics_cartesian_State_0_1 state_msg =
	{
			.pose = imu_pose,
			.twist = imu_twist
	};
    interface->send_msg<State>(
		&state_msg,
		state_buffer,
		AGENT_IMU_PORT,
		&int_transfer_id
	);
}

void heartbeat() {
	static uint8_t hbeat_buffer[HBeat::buffer_size];
	static CanardTransferID hbeat_transfer_id = 0;
	static uint32_t uptime = 0;
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {
        .uptime = uptime,
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL}
    };
    interface->send_msg<HBeat>(
		&heartbeat_msg,
		hbeat_buffer,
		uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
		&hbeat_transfer_id
	);
    uptime += 1;

}

void setup_cyphal(FDCAN_HandleTypeDef* handler) {
	interface = std::shared_ptr<CyphalInterface>(
		CyphalInterface::create_bss<G4CAN, SystemAllocator>(buffer, JOINT_N, handler, 400, utilities)
	);
	h_reader = new HBeatReader(interface);
}

void cyphal_loop() {
    interface->loop();
}

void cyphal_can_starter(FDCAN_HandleTypeDef* hfdcan)
{

	CanardFilter cyphal_filter_for_node_id = canardMakeFilterForServices(JOINT_N);
	CanardFilter cyphal_filter_for_JS = canardMakeFilterForSubject(1125);//JS_SUB_PORT_ID
	CanardFilter cyphal_filter_for_HB = canardMakeFilterForSubject(7509);//JS_SUB_PORT_ID
	CanardFilter cyphal_filter_consolidated = canardConsolidateFilters(&cyphal_filter_for_node_id, &cyphal_filter_for_JS);

	static FDCAN_FilterTypeDef sFilterConfig;
	static FDCAN_FilterTypeDef hbFilterConfig;
	static FDCAN_FilterTypeDef niFilterConfig;

	niFilterConfig.IdType = FDCAN_EXTENDED_ID;
	niFilterConfig.FilterIndex = 0;
	niFilterConfig.FilterType = FDCAN_FILTER_MASK;
	niFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	niFilterConfig.FilterID1 =  cyphal_filter_for_node_id.extended_can_id;
	niFilterConfig.FilterID2 =  cyphal_filter_for_node_id.extended_mask;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 =  cyphal_filter_for_JS.extended_can_id;
	sFilterConfig.FilterID2 =  cyphal_filter_for_JS.extended_mask;

	hbFilterConfig.IdType = FDCAN_EXTENDED_ID;
	hbFilterConfig.FilterIndex = 1;
	hbFilterConfig.FilterType = FDCAN_FILTER_MASK;
	hbFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	hbFilterConfig.FilterID1 =  cyphal_filter_for_HB.extended_can_id;
	hbFilterConfig.FilterID2 =  cyphal_filter_for_HB.extended_mask;



	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &niFilterConfig) != HAL_OK) {
	  Error_Handler();
	}
//	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
//	  Error_Handler();
//	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &hbFilterConfig) != HAL_OK) {
	  Error_Handler();
	}

	if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 5, 0) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) {
	  Error_Handler();
	}
//	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//	{
//	  Error_Handler();
//	}

	HAL_FDCAN_Start(hfdcan);
}

}

