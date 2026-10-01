#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_log.h"
#include "esp_rom_sys.h"


/* =========================================================
   Pin Configuration
   ========================================================= */

#define DHT_PIN GPIO_NUM_4

// GPIO34 = ADC1 Channel 6 on ESP32
#define LDR_ADC_CHANNEL ADC_CHANNEL_6


static const char *TAG = "SENSOR";

static adc_oneshot_unit_handle_t adc_handle;


/* =========================================================
   DHT22 Sensor
   ========================================================= */

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);

    // Start signal
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(20000);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);

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

        // Sample approximately in the middle of the bit
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

    // Checksum verification
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

    *temperature =
        raw_temperature / 10.0f;

    // Negative temperature
    if (data[2] & 0x80)
    {
        *temperature = -*temperature;
    }

    return true;
}


/* =========================================================
   LDR / ADC
   ========================================================= */

static void ldr_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &config
        )
    );
}


static int ldr_read_percent(void)
{
    int raw_value = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(
            adc_handle,
            LDR_ADC_CHANNEL,
            &raw_value
        )
    );

    // Convert 12-bit ADC value (0-4095) to 0-100%
    int percent =
        (raw_value * 100) / 4095;

    if (percent < 0)
    {
        percent = 0;
    }

    if (percent > 100)
    {
        percent = 100;
    }

    return percent;
}


/* =========================================================
   Sensor Acquisition
   ========================================================= */

static void readSensors(void)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    int light_percent = ldr_read_percent();

    // Read DHT22
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
        ESP_LOGE(
            TAG,
            "DHT22 read failed"
        );
    }

    // Read LDR
    ESP_LOGI(
        TAG,
        "Relative Light Level: %d %%",
        light_percent
    );
}


/* =========================================================
   SensorTask
   ========================================================= */

static void SensorTask(void *pvParameters)
{
    // Store the initial wake time
    TickType_t lastWakeTime =
        xTaskGetTickCount();

    for (;;)
    {
        // Acquire sensor data
        readSensors();

        // Run every 2 seconds
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}


/* =========================================================
   Application Entry Point
   ========================================================= */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "BCA152 FreeRTOS Multisensor"
    );

    ESP_LOGI(
        TAG,
        "System starting..."
    );

    // Initialize LDR ADC
    ldr_init();

    // Create periodic sensor task
    xTaskCreate(
        SensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        NULL
    );
}