#include "communication.h"

#include "fdcan.h"
#include "ruka_joints.h"

#include <cstddef>
#include <memory>

#include "cyphal/cyphal.h"
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/providers/G4CAN.h"
#include "cyphal/subscriptions/subscription.h"

#include "uavcan/node/Health_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "reg/udral/physics/kinematics/cartesian/State_0_1.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
}

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(State, reg_udral_physics_kinematics_cartesian_State_0_1)

namespace {

alignas(std::max_align_t) std::byte cyphal_buffer[sizeof(CyphalInterface) + sizeof(G4CAN) + sizeof(SystemAllocator)];
std::shared_ptr<CyphalInterface> interface;

uint64_t micros_64() {
    return static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
}

void error_handler() {
    Error_Handler();
}

UtilityConfig utilities(micros_64, error_handler);

class HBeatReader : public AbstractSubscription<HBeat> {
public:
    explicit HBeatReader(InterfacePtr interface)
        : AbstractSubscription<HBeat>(interface, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_) {}

    void handler(const uavcan_node_Heartbeat_1_0&, CanardRxTransfer*) override {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
};

HBeatReader* h_reader = nullptr;

}  // namespace

void send_IMU(
    float* qw,
    float* qx,
    float* qy,
    float* qz,
    float* ax,
    float* ay,
    float* az,
    float* gx,
    float* gy,
    float* gz
) {
    static CanardTransferID transfer_id = 0;
    State::Type state_msg{};

    state_msg.pose.orientation = {*qw, *qx, *qy, *qz};
    state_msg.twist.angular = {*ax, *ay, *az};
    state_msg.twist.linear = {*gx, *gy, *gz};

    if (interface) {
        interface->send_msg<State>(&state_msg, AGENT_IMU_PORT, &transfer_id);
    }
}

void send_2_IMU(
    float* qw,
    float* qx,
    float* qy,
    float* qz,
    float* ax,
    float* ay,
    float* az,
    float* gx,
    float* gy,
    float* gz
) {
    static CanardTransferID transfer_id = 0;
    State::Type state_msg{};

    state_msg.pose.orientation = {*qw, *qx, *qy, *qz};
    state_msg.twist.angular = {*ax, *ay, *az};
    state_msg.twist.linear = {*gx, *gy, *gz};

    if (interface) {
        interface->send_msg<State>(&state_msg, AGENT_IMU_2_PORT, &transfer_id);
    }
}

void heartbeat() {
    static CanardTransferID transfer_id = 0;
    static uint32_t uptime = 0;

    HBeat::Type heartbeat_msg = {
        .uptime = uptime,
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = 0
    };
    uptime += 1;
       //  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    if (interface) {
        interface->send_msg<HBeat>(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &transfer_id
        );
    }
}

void setup_cyphal_comms() {
    interface = std::shared_ptr<CyphalInterface>(
        CyphalInterface::create_bss<G4CAN, SystemAllocator>(
            cyphal_buffer,
            JOINT_N,
            &hfdcan1,
            400,
            utilities
        )
    );
    h_reader = new HBeatReader(interface);
}

void setup_can() {
    CanardFilter filter_for_node_id = canardMakeFilterForServices(JOINT_N);
    CanardFilter filter_for_hbeat = canardMakeFilterForSubject(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);

    FDCAN_FilterTypeDef filter_config{};
    filter_config.IdType = FDCAN_EXTENDED_ID;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE
        ) != HAL_OK) {
        Error_Handler();
    }

    filter_config.FilterIndex = 0;
    filter_config.FilterID1 = filter_for_node_id.extended_can_id;
    filter_config.FilterID2 = filter_for_node_id.extended_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        Error_Handler();
    }

    filter_config.FilterIndex = 1;
    filter_config.FilterID1 = filter_for_hbeat.extended_can_id;
    filter_config.FilterID2 = filter_for_hbeat.extended_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

void cyphal_loop() {
    if (interface) {
        interface->loop();
    }
}
