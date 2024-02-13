#include "main.h"
#include "ruka_joints.h"

#include <memory>

#include "cyphal/cyphal.h"
#include "cyphal/providers/G4CAN.h"
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/subscriptions/subscription.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/scalar/Integer32_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/Twist_0_1.h"


TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(Twist, reg_udral_physics_kinematics_cartesian_Twist_0_1)

std::byte buffer[sizeof(CyphalInterface) + sizeof(G4CAN) + sizeof(SystemAllocator)];
std::shared_ptr<CyphalInterface> interface;


void error_handler() { Error_Handler(); }
// Тут не нужен точный таймер, поэтому так
uint64_t micros_64() { return HAL_GetTick() * 1000; }
UtilityConfig utilities(micros_64, error_handler);

class HBeatReader: public AbstractSubscription<HBeat> {
public:
    HBeatReader(InterfacePtr interface): AbstractSubscription<HBeat>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const uavcan_node_Heartbeat_1_0& hbeat, CanardRxTransfer* transfer) override {
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
};


HBeatReader* h_reader;

class JSReader: public AbstractSubscription<JS_msg> {
public:
	JSReader(InterfacePtr interface): AbstractSubscription<JS_msg>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
		JS_SUB_PORT_ID
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_in, CanardRxTransfer* transfer) override {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
};

JSReader* js_reader;



extern "C" {

void send_JS(float* pos, float* vel, float* eff) {
	static uint8_t js_buffer[JS_msg::buffer_size];
	static CanardTransferID int_transfer_id = 0;
	reg_udral_physics_kinematics_rotation_Planar_0_1 js_msg =
	{
			.angular_position = *pos,
			.angular_velocity = *vel,
			.angular_acceleration = *eff
	};
    interface->send_cyphal_default_msg<JS_msg>(
		&js_msg,
		js_buffer,
		AGENT_JS_SUB_PORT,
		&int_transfer_id
	);
}

void send_IMU(float* av_1, float* av_2, float* av_3, float* aa_1, float* aa_2, float* aa_3)
{
	static uint8_t twist_buffer[Twist::buffer_size];
	static CanardTransferID int_transfer_id = 0;
	reg_udral_physics_kinematics_cartesian_Twist_0_1 twist_msg =
	{
			.linear = {*av_1, *av_2, *av_3},
			.angular = {*aa_1, *aa_2, *aa_3}
	};
    interface->send_cyphal_default_msg<Twist>(
		&twist_msg,
		twist_buffer,
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
    interface->send_cyphal_default_msg<HBeat>(
		&heartbeat_msg,
		hbeat_buffer,
		uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
		&hbeat_transfer_id
	);
    uptime += 1;

}

void setup_cyphal(FDCAN_HandleTypeDef* handler) {
	interface = std::shared_ptr<CyphalInterface>(
		         // memory location, node_id, fdcan handler, messages memory pool, utils ref
		CyphalInterface::create<G4CAN, SystemAllocator>(buffer, JOINT_N, handler, 400, utilities)
	);
	h_reader = new HBeatReader(interface);
	js_reader = new JSReader(interface);
}

void cyphal_loop() {
    interface->loop();
}

void cyphal_can_starter(FDCAN_HandleTypeDef* hfdcan)
{

	CanardFilter cyphal_filter_for_node_id = canardMakeFilterForServices(JOINT_N);
	CanardFilter cyphal_filter_for_JS = canardMakeFilterForSubject(1125);//JS_SUB_PORT_ID
	CanardFilter cyphal_filter_for_HB = canardMakeFilterForSubject(7509);//JS_SUB_PORT_ID
	CanardFilter cyphal_filter = canardConsolidateFilters(&cyphal_filter_for_node_id, &cyphal_filter_for_JS);

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

