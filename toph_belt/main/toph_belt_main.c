#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"

#define LIDAR_SDA_GPIO GPIO_NUM_21
#define LIDAR_SCL_GPIO GPIO_NUM_22
#define LIDAR_LPN_GPIO GPIO_NUM_18
#define LIDAR_ADDR (VL53L5CX_DEFAULT_I2C_ADDRESS >> 1)
#define I2C_TIMEOUT_MS 2000
#define GRID_WIDTH 8U
#define FOV_TENTHS_DEG 450
#define MOTOR_COUNT 4U
#define PWM_MAX 1023U
#define OFF_DISTANCE_MM 1600U
#define FULL_DISTANCE_MM 100U
#define MIN_PERCENT 35U

static const char *TAG = "lidar_motors";
typedef struct { gpio_num_t pin; ledc_channel_t channel; const char *position; } motor_t;
/*
 * Motor locations, left to right, when the wearer faces the same direction as
 * the lidar.  Each motor independently activates when its sector contains a
 * valid detected object, so several motors can run at the same time.
 *
 *   object far left  -> GPIO 27 ("far left")
 *   object left      -> GPIO 26 ("left")
 *   object right     -> GPIO 25 ("right")
 *   object far right -> GPIO 33 ("far right")
 */
static const motor_t motors[MOTOR_COUNT] = {
    {GPIO_NUM_27, LEDC_CHANNEL_0, "far left"},
    {GPIO_NUM_26, LEDC_CHANNEL_1, "left"},
    {GPIO_NUM_25, LEDC_CHANNEL_2, "right"},
    {GPIO_NUM_33, LEDC_CHANNEL_4, "far right"},
};
static i2c_master_dev_handle_t lidar_device;
static VL53L5CX_Configuration lidar;
static VL53L5CX_ResultsData results;

static int32_t lidar_write(uint16_t address, uint16_t reg, uint8_t *data, uint16_t size) {
    (void)address;
    uint8_t reg_bytes[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    i2c_master_transmit_multi_buffer_info_t buffers[] = {
        {.write_buffer = reg_bytes, .buffer_size = sizeof(reg_bytes)},
        {.write_buffer = data, .buffer_size = size},
    };
    return i2c_master_multi_buffer_transmit(lidar_device, buffers, 2, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}
static int32_t lidar_read(uint16_t address, uint16_t reg, uint8_t *data, uint16_t size) {
    (void)address;
    uint8_t reg_bytes[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return i2c_master_transmit_receive(lidar_device, reg_bytes, sizeof(reg_bytes), data, size,
                                       I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}
static int32_t get_tick_ms(void) { return (int32_t)(esp_timer_get_time() / 1000LL); }

static void motor_set_percent(size_t index, uint8_t percent) {
    if (percent > 100U) percent = 100U;
    uint32_t duty = ((uint32_t)percent * PWM_MAX) / 100U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[index].channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[index].channel));
}
static void motors_off(void) { for (size_t i = 0; i < MOTOR_COUNT; ++i) motor_set_percent(i, 0); }
static void motors_init(void) {
    const ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0,
        .freq_hz = 200, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        const ledc_channel_config_t channel = {.gpio_num = motors[i].pin,
            .speed_mode = LEDC_LOW_SPEED_MODE, .channel = motors[i].channel,
            .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
}
static uint8_t distance_to_percent(uint16_t distance) {
    /*
     * Closer objects feel stronger:
     *   100 mm or closer: selected motor at 100%
     *   101--1599 mm:     selected motor scales from 100% down to 35%
     *   1600 mm or farther: all motors off
     */
    if (distance >= OFF_DISTANCE_MM) return 0;
    if (distance <= FULL_DISTANCE_MM) return 100;
    return MIN_PERCENT + ((OFF_DISTANCE_MM - distance) * (100U - MIN_PERCENT)) /
                         (OFF_DISTANCE_MM - FULL_DISTANCE_MM);
}
static int16_t zone_to_horizontal_angle(uint8_t zone) {
    const uint8_t scene_column = GRID_WIDTH - 1U - (zone % GRID_WIDTH);
    return (int16_t)((((int32_t)(2U * scene_column + 1U) - GRID_WIDTH) * FOV_TENTHS_DEG) /
                     (2 * GRID_WIDTH));
}
static size_t motor_for_angle(int16_t angle);
/*
 * Return the closest valid object in each motor's sector.  A motor's strength
 * is based on the closest object in its own sector, independently of objects
 * in the other three sectors.  UINT16_MAX means that sector has no object.
 */
static void find_nearest_targets_per_motor(const VL53L5CX_ResultsData *frame,
                                           uint16_t nearest[MOTOR_COUNT]) {
    for (size_t motor = 0; motor < MOTOR_COUNT; ++motor) nearest[motor] = UINT16_MAX;
    for (uint8_t zone = 0; zone < VL53L5CX_RESOLUTION_8X8; ++zone) {
        const int16_t distance = frame->distance_mm[zone];
        const uint8_t status = frame->target_status[zone];
        if (frame->nb_target_detected[zone] > 0 && (status == 5 || status == 9) &&
            distance > 0) {
            const size_t motor = motor_for_angle(zone_to_horizontal_angle(zone));
            if ((uint16_t)distance < nearest[motor]) nearest[motor] = (uint16_t)distance;
        }
    }
}
/*
 * Divide the approximately 45-degree horizontal lidar field of view into four
 * equal sectors.  This returns the motor associated with one lidar zone:
 *
 *   far-left quarter  -> motors[0], GPIO 27
 *   left quarter      -> motors[1], GPIO 26
 *   right quarter     -> motors[2], GPIO 25
 *   far-right quarter -> motors[3], GPIO 33
 */
static size_t motor_for_angle(int16_t angle) {
    int32_t index = ((int32_t)angle + FOV_TENTHS_DEG / 2) / (FOV_TENTHS_DEG / MOTOR_COUNT);
    if (index < 0) index = 0;
    if (index >= MOTOR_COUNT) index = MOTOR_COUNT - 1;
    return (size_t)index;
}
static void lidar_init(void) {
    ESP_ERROR_CHECK(gpio_reset_pin(LIDAR_LPN_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(LIDAR_LPN_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(LIDAR_LPN_GPIO, 1));
    vTaskDelay(pdMS_TO_TICKS(10));
    const i2c_master_bus_config_t bus_cfg = {.i2c_port = I2C_NUM_0, .sda_io_num = LIDAR_SDA_GPIO,
        .scl_io_num = LIDAR_SCL_GPIO, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    const i2c_device_config_t device_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LIDAR_ADDR, .scl_speed_hz = 400000};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_cfg, &lidar_device));
    lidar.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    lidar.platform.Write = lidar_write; lidar.platform.Read = lidar_read; lidar.platform.GetTick = get_tick_ms;
    uint8_t alive = 0;
    ESP_ERROR_CHECK((vl53l5cx_is_alive(&lidar, &alive) == VL53L5CX_STATUS_OK && alive) ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(vl53l5cx_init(&lidar) == VL53L5CX_STATUS_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(vl53l5cx_set_resolution(&lidar, VL53L5CX_RESOLUTION_8X8) == VL53L5CX_STATUS_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(vl53l5cx_set_ranging_frequency_hz(&lidar, 10) == VL53L5CX_STATUS_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(vl53l5cx_start_ranging(&lidar) == VL53L5CX_STATUS_OK ? ESP_OK : ESP_FAIL);
}
void app_main(void) {
    motors_init(); motors_off(); lidar_init();
    ESP_LOGI(TAG, "Lidar active: GPIO 27, 26, 25, 33 map left to right");
    while (true) {
        uint8_t ready = 0;
        if (vl53l5cx_check_data_ready(&lidar, &ready) != VL53L5CX_STATUS_OK) {
            motors_off(); ESP_LOGE(TAG, "Lidar communication error; motors off");
            vTaskDelay(pdMS_TO_TICKS(100)); continue;
        }
        if (ready) {
            if (vl53l5cx_get_ranging_data(&lidar, &results) == VL53L5CX_STATUS_OK) {
                uint16_t nearest[MOTOR_COUNT];
                find_nearest_targets_per_motor(&results, nearest);

                /* Set all four motors from this frame.  Several can run at
                 * once; a covered sensor or object across the whole grid can
                 * therefore activate every motor. */
                for (size_t motor = 0; motor < MOTOR_COUNT; ++motor) {
                    const uint8_t percent = nearest[motor] == UINT16_MAX ? 0 :
                                            distance_to_percent(nearest[motor]);
                    motor_set_percent(motor, percent);
                    if (percent > 0) ESP_LOGI(TAG, "%u mm -> %s GPIO %d: %u%%",
                                               nearest[motor], motors[motor].position,
                                               motors[motor].pin, percent);
                }
            } else motors_off();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
