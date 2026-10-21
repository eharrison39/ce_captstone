/**
 * ESP-IDF platform glue for ST's VL53L5CX Ultra Lite Driver.
 *
 * ST keeps these small wrapper functions separate from the sensor algorithm so
 * the same driver can run on many microcontrollers. The function pointers in
 * VL53L5CX_Platform ultimately call the ESP-IDF I2C code in main.c.
 */
#include "platform.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t RdByte(VL53L5CX_Platform *platform, uint16_t address, uint8_t *value)
{
    /* Read one byte using the application-provided I2C callback. */
    return (uint8_t)platform->Read(platform->address, address, value, 1U);
}

uint8_t WrByte(VL53L5CX_Platform *platform, uint16_t address, uint8_t value)
{
    /* Write one byte using the application-provided I2C callback. */
    return (uint8_t)platform->Write(platform->address, address, &value, 1U);
}

uint8_t RdMulti(VL53L5CX_Platform *platform, uint16_t address,
                uint8_t *values, uint32_t size)
{
    /* Read a contiguous block; ST uses this for complete result frames. */
    return (uint8_t)platform->Read(platform->address, address, values,
                                   (uint16_t)size);
}

uint8_t WrMulti(VL53L5CX_Platform *platform, uint16_t address,
                uint8_t *values, uint32_t size)
{
    /* Write a contiguous block; large blocks upload firmware to the sensor. */
    return (uint8_t)platform->Write(platform->address, address, values,
                                    (uint16_t)size);
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    /* Hold one reordered 32-bit word at a time. */
    uint32_t value;

    /* ST's protocol sends these words most-significant byte first. */
    for (uint16_t index = 0; index < size; index += 4U) {
        /* Reassemble four incoming bytes into the ESP32's native word value. */
        value = ((uint32_t)buffer[index] << 24) |
                ((uint32_t)buffer[index + 1U] << 16) |
                ((uint32_t)buffer[index + 2U] << 8) |
                (uint32_t)buffer[index + 3U];
        /* memcpy avoids alignment problems when writing back into byte data. */
        memcpy(&buffer[index], &value, sizeof(value));
    }
}

uint8_t WaitMs(VL53L5CX_Platform *platform, uint32_t time_ms)
{
    /* This platform implementation does not need the configuration pointer. */
    (void)platform;

    /* Sleep the task so other FreeRTOS work can run during sensor waits. */
    vTaskDelay(pdMS_TO_TICKS(time_ms));

    /* ST's platform API uses zero to report a successful delay. */
    return 0U;
}
