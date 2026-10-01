#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/* =========================================================
 * Pin Configuration
 * ========================================================= */

#define DHT_PIN GPIO_NUM_4

/* ESP32 GPIO34 = ADC1 Channel 6 */
#define LDR_ADC_CHANNEL ADC_CHANNEL_6


/* =========================================================
 * Global Variables
 * ========================================================= */

static const char *TAG = "SENSOR";

static adc_oneshot_unit_handle_t adc_handle;

/* Two queues so both consumers receive the sensor data */
static QueueHandle_t displayQueue;
static QueueHandle_t alarmQueue;


/* =========================================================
 * Sensor Data Structure
 * ========================================================= */

typedef struct
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;

} SensorData;


/* =========================================================
 * DHT22 Sensor Reading
 * ========================================================= */

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    /* Start signal */
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(DHT_PIN, 0);

    /* DHT22 requires at least 1 ms LOW */
    esp_rom_delay_us(20000);

    gpio_set_level(DHT_PIN, 1);

    esp_rom_delay_us(30);

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);


    /* Wait for DHT22 response */

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


    /* Read 40 bits */

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

        /*
         * DHT22 uses pulse width to represent
         * 0 or 1.
         */

        esp_rom_delay_us(40);

        if (gpio_get_level(DHT_PIN))
        {
            data[i / 8] |=
                (1 << (7 - (i % 8)));
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


    /* Checksum */

    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
    {
        ESP_LOGE(
            TAG,
            "DHT22 checksum error"
        );

        return false;
    }


    /* Humidity */

    *humidity =
        ((data[0] << 8) | data[1]) / 10.0f;


    /* Temperature */

    int16_t raw_temperature =
        ((data[2] & 0x7F) << 8) |
        data[3];

    *temperature =
        raw_temperature / 10.0f;


    /* Negative temperature */

    if (data[2] & 0x80)
    {
        *temperature =
            -*temperature;
    }

    return true;
}


/* =========================================================
 * LDR ADC Initialization
 * ========================================================= */

static void ldr_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config =
    {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &adc_handle
        )
    );


    adc_oneshot_chan_cfg_t config =
    {
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


    ESP_LOGI(
        TAG,
        "LDR ADC initialized"
    );
}


/* =========================================================
 * LDR Measurement
 * ========================================================= */

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


    /*
     * ESP32 ADC is 12-bit:
     *
     * 0    = minimum
     * 4095 = maximum
     *
     * Convert to 0-100%.
     */

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


    ESP_LOGI(
        TAG,
        "LDR Raw: %d",
        raw_value
    );


    return percent;
}


/* =========================================================
 * Sensor Acquisition
 * ========================================================= */

static SensorData readSensors(void)
{
    SensorData data =
    {
        .temperature = 0.0f,
        .humidity = 0.0f,
        .lightLevel = 0,
        .motionDetected = false
    };


    /* Read LDR */

    data.lightLevel =
        ldr_read_percent();


    /* Read DHT22 */

    if (dht22_read(
            &data.temperature,
            &data.humidity))
    {
        ESP_LOGI(
            TAG,
            "Temperature: %.2f C",
            data.temperature
        );


        ESP_LOGI(
            TAG,
            "Humidity: %.2f %%",
            data.humidity
        );
    }
    else
    {
        ESP_LOGE(
            TAG,
            "DHT22 read failed"
        );
    }


    /* LDR result */

    ESP_LOGI(
        TAG,
        "Relative Light Level: %d %%",
        data.lightLevel
    );


    /*
     * PIR has not been implemented yet.
     * Motion remains false until PIR is added.
     */

    data.motionDetected = false;


    return data;
}


/* =========================================================
 * Sensor Task
 * ========================================================= */

static void SensorTask(void *pvParameters)
{
    /*
     * Save the initial execution time.
     * vTaskDelayUntil() uses this as the
     * reference point for periodic execution.
     */

    TickType_t lastWakeTime =
        xTaskGetTickCount();


    SensorData sensorData;


    for (;;)
    {
        /* Acquire sensor data */

        sensorData =
            readSensors();


        /*
         * Send the SAME SensorData to the
         * Display Queue.
         */

        if (xQueueSend(
                displayQueue,
                &sensorData,
                pdMS_TO_TICKS(100)
            ) != pdPASS)
        {
            ESP_LOGW(
                TAG,
                "Display queue full"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "Data sent to Display Queue"
            );
        }


        /*
         * Send the SAME SensorData to the
         * Alarm Queue.
         */

        if (xQueueSend(
                alarmQueue,
                &sensorData,
                pdMS_TO_TICKS(100)
            ) != pdPASS)
        {
            ESP_LOGW(
                TAG,
                "Alarm queue full"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "Data sent to Alarm Queue"
            );
        }


        /*
         * Execute every 2 seconds.
         */

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}


/* =========================================================
 * Display Task
 * ========================================================= */

static void DisplayTask(void *pvParameters)
{
    SensorData data;


    for (;;)
    {
        /*
         * Wait for sensor data from
         * the Display Queue.
         */

        if (xQueueReceive(
                displayQueue,
                &data,
                portMAX_DELAY
            ) == pdTRUE)
        {
            ESP_LOGI(
                TAG,
                "DISPLAY -> Temp: %.2f C | Humidity: %.2f %% | Light: %d %% | Motion: %s",
                data.temperature,
                data.humidity,
                data.lightLevel,
                data.motionDetected ? "YES" : "NO"
            );
        }
    }
}


/* =========================================================
 * Alarm Task
 * ========================================================= */

static void AlarmTask(void *pvParameters)
{
    SensorData data;


    for (;;)
    {
        /*
         * Wait for sensor data from
         * the Alarm Queue.
         */

        if (xQueueReceive(
                alarmQueue,
                &data,
                portMAX_DELAY
            ) == pdTRUE)
        {
            /*
             * Motion alarm will be implemented
             * when the PIR sensor is added.
             */

            if (data.motionDetected)
            {
                ESP_LOGW(
                    TAG,
                    "ALARM -> Motion detected!"
                );
            }
            else
            {
                ESP_LOGI(
                    TAG,
                    "ALARM -> No motion"
                );
            }
        }
    }
}


/* =========================================================
 * Application Entry Point
 * ========================================================= */

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


    /* Initialize LDR */

    ldr_init();


    /* =====================================================
     * Create Display Queue
     * ===================================================== */

    displayQueue =
        xQueueCreate(
            5,
            sizeof(SensorData)
        );


    /* =====================================================
     * Create Alarm Queue
     * ===================================================== */

    alarmQueue =
        xQueueCreate(
            5,
            sizeof(SensorData)
        );


    /* Check queues */

    if (displayQueue == NULL ||
        alarmQueue == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to create sensor queues"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "Display queue created"
    );


    ESP_LOGI(
        TAG,
        "Alarm queue created"
    );


    /* =====================================================
     * Create SensorTask
     * ===================================================== */

    BaseType_t taskResult =
        xTaskCreate(
            SensorTask,
            "SensorTask",
            4096,
            NULL,
            1,
            NULL
        );


    if (taskResult != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create SensorTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "SensorTask created"
    );


    /* =====================================================
     * Create DisplayTask
     * ===================================================== */

    taskResult =
        xTaskCreate(
            DisplayTask,
            "DisplayTask",
            4096,
            NULL,
            1,
            NULL
        );


    if (taskResult != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create DisplayTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "DisplayTask created"
    );


    /* =====================================================
     * Create AlarmTask
     * ===================================================== */

    taskResult =
        xTaskCreate(
            AlarmTask,
            "AlarmTask",
            4096,
            NULL,
            1,
            NULL
        );


    if (taskResult != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create AlarmTask"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "AlarmTask created"
    );
}