#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// The built-in LED on most NodeMCU-32S boards is connected to GPIO 2.
// Change this value if you connected an external LED to another GPIO.
#define LED_GPIO GPIO_NUM_2
#define BLINK_DELAY_MS 500

static const char *TAG = "blink";

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT));

    bool led_on = false;

    ESP_LOGI(TAG, "Blinking LED on GPIO %d", LED_GPIO);

    while (true) {
        led_on = !led_on;
        ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, led_on));
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}
