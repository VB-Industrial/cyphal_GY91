#pragma once

#include "main.h"

void setup_cyphal_comms();
void setup_can();
void cyphal_loop();
void heartbeat();
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
);
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
);
