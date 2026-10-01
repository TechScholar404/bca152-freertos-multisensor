#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define DHT_PIN GPIO_NUM_4

static const char *TAG = "DHT22";


static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    // Set GPIO as output
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);

    // Start signal
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(20000);   // 20 ms

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);      // 30 us

    // Change to input
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    // Wait for DHT22 response
    int timeout = 100;

    while (gpio_get_level(DHT_PIN) == 1 && timeout--)
    {
        esp_rom_delay_us(1);
    }

    timeout = 100;

    while (gpio_get_level(DHT_PIN) == 0 && timeout--)
    {
        esp_rom_delay_us(1);
    }

    timeout = 100;

    while (gpio_get_level(DHT_PIN) == 1 && timeout--)
    {
        esp_rom_delay_us(1);
    }

    if (timeout <= 0)
    {
        return false;
    }

    // Read 40 bits
    for (int i = 0; i < 40; i++)
    {
        timeout = 100;

        while (gpio_get_level(DHT_PIN) == 0 && timeout--)
        {
            esp_rom_delay_us(1);
        }

        if (timeout <= 0)
        {
            return false;
        }

        esp_rom_delay_us(40);

        if (gpio_get_level(DHT_PIN))
        {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }

        timeout = 100;

        while (gpio_get_level(DHT_PIN) == 1 && timeout--)
        {
            esp_rom_delay_us(1);
        }

        if (timeout <= 0)
        {
            return false;
        }
    }

    // Check checksum
    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
    {
        ESP_LOGE(TAG, "DHT22 checksum error");
        return false;
    }

    // Humidity
    *humidity =
        ((data[0] << 8) | data[1]) / 10.0f;

    // Temperature
    int16_t raw_temperature =
        ((data[2] & 0x7F) << 8) | data[3];

    *temperature = raw_temperature / 10.0f;

    if (data[2] & 0x80)
    {
        *temperature = -*temperature;
    }

    return true;
}


static void dht_task(void *pvParameters)
{
    float temperature;
    float humidity;

    while (1)
    {
        if (dht22_read(&temperature, &humidity))
        {
            ESP_LOGI(
                TAG,
                "Temperature: %.2f C",
                temperature
            );

            ESP_LOGI(
                TAG,
                "Humidity: %.2f %%",
                humidity
            );
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read DHT22");
        }

        // DHT22 requires at least about 2 seconds between readings
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System starting...");

    xTaskCreate(
        dht_task,
        "DHT22 Task",
        4096,
        NULL,
        1,
        NULL
    );
}