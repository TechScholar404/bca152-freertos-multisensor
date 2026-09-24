#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include "dht.h"
#include "ssd1306.h"

#define DHT_GPIO        GPIO_NUM_4

#define OLED_SDA_GPIO   GPIO_NUM_21
#define OLED_SCL_GPIO   GPIO_NUM_22

#define I2C_PORT        I2C_NUM_0

static const char *TAG = "ROOM_MONITOR";

typedef struct
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;

} SensorData;

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;

static i2c_master_bus_handle_t oled_i2c_bus = NULL;

static ssd1306_handle_t oled = NULL;

static void oled_init(void)
{
    ESP_LOGI(TAG, "Initializing OLED...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &oled_i2c_bus
        )
    );

   ssd1306_config_t oled_config =
    SSD1306_128x64_CONFIG_DEFAULT;
    ESP_ERROR_CHECK(
        ssd1306_init(
            oled_i2c_bus,
            &oled_config,
            &oled
        )
    );

    if (oled == NULL)
    {
        ESP_LOGE(
            TAG,
            "SSD1306 initialization failed!"
        );

        return;
    }

    ssd1306_clear_display(
        oled,
        false
    );

    ssd1306_set_contrast(oled, 0xFF);

    ssd1306_display_text(
        oled,
        0,
        "ROOM MONITOR",
        false
    );

    ssd1306_display_text(
        oled,
        2,
        "Temperature",
        false
    );

    ssd1306_display_text(
        oled,
        4,
        "Starting...",
        false
    );

    ESP_LOGI(
        TAG,
        "OLED initialized successfully."
    );
}

static void readSensors(SensorData *data)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    esp_err_t result = dht_read_float_data(
        DHT_TYPE_AM2301,
        DHT_GPIO,
        &humidity,
        &temperature
    );

    if (result == ESP_OK)
    {
        data->temperature = temperature;
        data->humidity = humidity;
    }
    else
    {
        printf("DHT read failed: %s\n",
               esp_err_to_name(result));

        data->temperature = 0.0f;
        data->humidity = 0.0f;
    }

    data->lightLevel = 500;

    data->motionDetected = false;
}

static void SensorTask(void *pvParameters)
{
    SensorData data;

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    while (1)
    {
        printf("\n");
        printf("SensorTask: Reading sensors...\n");

        readSensors(&data);

        printf(
            "Temperature: %.2f C\n",
            data.temperature
        );

        printf(
            "Humidity: %.2f %%\n",
            data.humidity
        );

        printf(
            "Light Level: %d\n",
            data.lightLevel
        );

        printf(
            "Motion: %s\n",
            data.motionDetected
                ? "DETECTED"
                : "NOT DETECTED"
        );

        if (xQueueSend(
                displayQueue,
                &data,
                pdMS_TO_TICKS(100)
            ) != pdPASS)
        {
            printf(
                "WARNING: Display queue full!\n"
            );
        }

        if (xQueueSend(
                alarmQueue,
                &data,
                pdMS_TO_TICKS(100)
            ) != pdPASS)
        {
            printf(
                "WARNING: Alarm queue full!\n"
            );
        }

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

static void DisplayTask(void *pvParameters)
{
    SensorData data;

    while (1)
    {
        if (xQueueReceive(
                displayQueue,
                &data,
                portMAX_DELAY
            ) == pdPASS)
        {
            char temperatureText[20];

            snprintf(
                temperatureText,
                sizeof(temperatureText),
                "%.1f C",
                data.temperature
            );

            ssd1306_clear_display(
                oled,
                false
            );

            ssd1306_display_text(
                oled,
                0,
                "ROOM MONITOR",
                false
            );

            ssd1306_display_text(
                oled,
                2,
                "Temperature",
                false
            );

            ssd1306_display_text(
                oled,
                4,
                temperatureText,
                false
            );

            printf("\n");
            printf("----- DISPLAY -----\n");

            printf(
                "Temperature: %.2f C\n",
                data.temperature
            );

            printf(
                "Humidity: %.2f %%\n",
                data.humidity
            );

            printf(
                "Light Level: %d\n",
                data.lightLevel
            );

            printf(
                "Motion: %s\n",
                data.motionDetected
                    ? "DETECTED"
                    : "NOT DETECTED"
            );

            printf("-------------------\n");
        }
    }
}

static void AlarmTask(void *pvParameters)
{
    SensorData data;

    while (1)
    {
        if (xQueueReceive(
                alarmQueue,
                &data,
                portMAX_DELAY
            ) == pdPASS)
        {
            if (
                data.motionDetected &&
                data.lightLevel < 100
            )
            {
                printf("\n");
                printf("!!! ALARM !!!\n");
                printf(
                    "Motion detected in darkness!\n"
                );
            }
            else
            {
                printf(
                    "Alarm: NORMAL\n"
                );
            }
        }
    }
}

void app_main(void)
{
    printf("\n");
    printf("================================\n");
    printf(" ESP32 FREERTOS SENSOR SYSTEM\n");
    printf("================================\n");

    displayQueue = xQueueCreate(
        10,
        sizeof(SensorData)
    );

    alarmQueue = xQueueCreate(
        10,
        sizeof(SensorData)
    );

    if (
        displayQueue == NULL ||
        alarmQueue == NULL
    )
    {
        printf(
            "ERROR: Queue creation failed!\n"
        );

        return;
    }

    printf(
        "Queues created successfully.\n"
    );

    oled_init();

    if (oled == NULL)
    {
        printf(
            "ERROR: OLED initialization failed!\n"
        );

        return;
    }

    xTaskCreate(
        SensorTask,
        "SensorTask",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        DisplayTask,
        "DisplayTask",
        4096,
        NULL,
        4,
        NULL
    );

    xTaskCreate(
        AlarmTask,
        "AlarmTask",
        4096,
        NULL,
        4,
        NULL
    );

    printf(
        "All tasks started successfully.\n"
    );
}
