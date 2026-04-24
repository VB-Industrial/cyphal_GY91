#include "app.h"

#include "IMU.h"
#include "communication.h"

imu_status_t g_debug_imu_status{};
volatile bool g_debug_imu_status_valid = false;

extern "C" void app(void) {
    setup_cyphal_comms();
    setup_can();

    IMU_setup();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    uint32_t last_hbeat = HAL_GetTick();
    uint32_t last_imu_send = HAL_GetTick();

    vec_4ax linear = {0.0f, 0.0f, 0.0f, 0.0f};
    vec_4ax quat = {0.0f, 0.0f, 0.0f, 0.0f};
    vec_4ax gyro = {0.0f, 0.0f, 0.0f, 0.0f};
    vec_4ax g = {0.0f, 0.0f, 0.0f, 0.0f};
    vec_4ax mag = {1.0f, 0.0f, 0.0f, 0.0f};
    vec_4ax accel = {0.0f, 0.0f, 0.0f, 0.0f};

    while (true) {
        const uint32_t now = HAL_GetTick();
        if ((now - last_imu_send) >= 20U) {
            imu_begin_sample();
            imu_get_quat(&quat);
            imu_get_linear(&linear);
            imu_get_gyro(&gyro);
            imu_get_gravity(&g);
            imu_get_accel(&accel);
            imu_get_mag(&mag);
            send_IMU(
                &quat.w,
                &quat.x,
                &quat.y,
                &quat.z,
                &linear.x,
                &linear.y,
                &linear.z,
                &gyro.x,
                &gyro.y,
                &gyro.z
            );
            send_2_IMU(
                &mag.w,
                &mag.x,
                &mag.y,
                &mag.z,
                &accel.x,
                &accel.y,
                &accel.z,
                &g.x,
                &g.y,
                &g.z
            );
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);

            imu_status_t status{};
            if (imu_read_status(&status)) {
                g_debug_imu_status = status;
                g_debug_imu_status_valid = true;
            } else {
                g_debug_imu_status_valid = false;
            }

            last_imu_send = now;
        }

        if ((now - last_hbeat) >= 1000U) {
            heartbeat();
            last_hbeat = now;
        }

        cyphal_loop();
    }
}
