#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "ssd1306.h"
#include "dht.h"

#define DHT_PIN GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 100000
#define OLED_ADDRESS 0x3C

struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

adc_oneshot_unit_handle_t adc_handle;

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;

ssd1306_handle_t oled;

void readSensors(struct SensorData *data)
{
    float temperature = 0;
    float humidity = 0;

    esp_err_t result = dht_read_float_data(
        DHT_TYPE_AM2301,
        DHT_PIN,
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
        printf("DHT22 read failed: %s\n", esp_err_to_name(result));
    }

    int rawLdr = 0;

    adc_oneshot_read(
        adc_handle,
        LDR_ADC_CHANNEL,
        &rawLdr
    );

    int lightLevel = 100 - (((rawLdr - 34) * 100) / (4063 - 34));

    if (lightLevel < 0)
    {
        lightLevel = 0;
    }

    if (lightLevel > 100)
    {
        lightLevel = 100;
    }

    data->lightLevel = lightLevel;
    data->motionDetected = false;

    printf("Temperature: %.2f C\n", data->temperature);
    printf("Humidity: %.2f %%\n", data->humidity);
    printf("LDR Raw: %d\n", rawLdr);
    printf("Relative Light Level: %d %%\n", data->lightLevel);
}

void SensorTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    struct SensorData sensorData;

    for (;;)
    {
        readSensors(&sensorData);

        xQueueSend(
            displayQueue,
            &sensorData,
            portMAX_DELAY
        );

        xQueueSend(
            alarmQueue,
            &sensorData,
            portMAX_DELAY
        );

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

void DisplayTask(void *pvParameters)
{
    struct SensorData sensorData;

    for (;;)
    {
        if (xQueueReceive(
                displayQueue,
                &sensorData,
                portMAX_DELAY) == pdTRUE)
        {
            char line[32];

            ssd1306_clear_screen(oled, 0x00);

            ssd1306_draw_string(
                oled,
                0,
                0,
                (const uint8_t *)"ROOM MONITOR",
                16,
                1
            );

            snprintf(
                line,
                sizeof(line),
                "Temp: %.1f C",
                sensorData.temperature
            );

            ssd1306_draw_string(
                oled,
                0,
                20,
                (const uint8_t *)line,
                16,
                1
            );

            snprintf(
                line,
                sizeof(line),
                "Humidity: %.1f%%",
                sensorData.humidity
            );

            ssd1306_draw_string(
                oled,
                0,
                36,
                (const uint8_t *)line,
                16,
                1
            );

            snprintf(
                line,
                sizeof(line),
                "Light: %d%%",
                sensorData.lightLevel
            );

            ssd1306_draw_string(
                oled,
                0,
                52,
                (const uint8_t *)line,
                16,
                1
            );

            ssd1306_refresh_gram(oled);

            printf(
                "Display Data: %.2f C, %.2f %%, Light %d %%\n",
                sensorData.temperature,
                sensorData.humidity,
                sensorData.lightLevel
            );
        }
    }
}

void AlarmTask(void *pvParameters)
{
    struct SensorData sensorData;

    for (;;)
    {
        if (xQueueReceive(
                alarmQueue,
                &sensorData,
                portMAX_DELAY) == pdTRUE)
        {
            printf(
                "Alarm Data: Temperature %.2f C\n",
                sensorData.temperature
            );
        }
    }
}

void app_main(void)
{
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("Starting system...\n");

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    i2c_param_config(
        I2C_MASTER_NUM,
        &i2c_config
    );

    i2c_driver_install(
        I2C_MASTER_NUM,
        i2c_config.mode,
        0,
        0,
        0
    );

    oled = ssd1306_create(
        I2C_MASTER_NUM,
        OLED_ADDRESS
    );

    if (oled == NULL)
    {
        printf("Failed to initialize OLED\n");
        return;
    }

    ssd1306_clear_screen(oled, 0x00);
    ssd1306_refresh_gram(oled);

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(
        &adc_config,
        &adc_handle
    );

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(
        adc_handle,
        LDR_ADC_CHANNEL,
        &adc_channel_config
    );

    displayQueue = xQueueCreate(
        5,
        sizeof(struct SensorData)
    );

    alarmQueue = xQueueCreate(
        5,
        sizeof(struct SensorData)
    );

    if (displayQueue == NULL || alarmQueue == NULL)
    {
        printf("Failed to create sensor queues\n");
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
}