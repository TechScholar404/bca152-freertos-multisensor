#include <stdio.h>
#include <stdbool.h>

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

#define ENCODER_CLK     GPIO_NUM_18
#define ENCODER_DT      GPIO_NUM_19

static const char *TAG = "ROOM_MONITOR";

typedef struct
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
} SensorData;

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;
QueueHandle_t modeQueue;

static i2c_master_bus_handle_t oled_i2c_bus = NULL;
static ssd1306_handle_t oled = NULL;

static void oled_init(void)
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_PORT;
    bus_config.sda_io_num = OLED_SDA_GPIO;
    bus_config.scl_io_num = OLED_SCL_GPIO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &oled_i2c_bus));

    ssd1306_config_t oled_config = {};
    oled_config.i2c_address = I2C_SSD1306_DEV_ADDR;
    oled_config.i2c_clock_speed = I2C_SSD1306_DEV_CLK_SPD;
    oled_config.panel_size = SSD1306_PANEL_128x64;
    oled_config.offset_x = 0;
    oled_config.flip_enabled = false;
    oled_config.display_enabled = true;

    ESP_ERROR_CHECK(
        ssd1306_init(
            oled_i2c_bus,
            &oled_config,
            &oled
        )
    );

    if (oled == NULL)
    {
        ESP_LOGE(TAG, "SSD1306 initialization failed!");
        return;
    }

    ssd1306_clear_display(oled, false);
    ssd1306_set_contrast(oled, 0xFF);

    ssd1306_display_text(oled, 0, "ROOM MONITOR", false);
    ssd1306_display_text(oled, 2, "Temperature", false);
    ssd1306_display_text(oled, 4, "Starting...", false);
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
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        printf("\nSensorTask: Reading sensors...\n");

        readSensors(&data);

        printf("Temperature: %.2f C\n", data.temperature);
        printf("Humidity: %.2f %%\n", data.humidity);
        printf("Light Level: %d\n", data.lightLevel);
        printf("Motion: %s\n",
               data.motionDetected ? "DETECTED" : "NOT DETECTED");

        if (xQueueSend(displayQueue, &data,
                       pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("WARNING: Display queue full!\n");
        }

        if (xQueueSend(alarmQueue, &data,
                       pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("WARNING: Alarm queue full!\n");
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
    }
}

static void drawDisplay(const SensorData *data, DisplayMode mode)
{
    char valueText[24];

    ssd1306_clear_display(oled, false);

    switch (mode)
    {
        case DisplayMode::TEMPERATURE:
            snprintf(valueText, sizeof(valueText),
                     "%.1f C", data->temperature);
            ssd1306_display_text(oled, 0, "TEMPERATURE", false);
            ssd1306_display_text(oled, 2, valueText, false);
            break;

        case DisplayMode::HUMIDITY:
            snprintf(valueText, sizeof(valueText),
                     "%.1f %%", data->humidity);
            ssd1306_display_text(oled, 0, "HUMIDITY", false);
            ssd1306_display_text(oled, 2, valueText, false);
            break;

        case DisplayMode::LIGHT:
            snprintf(valueText, sizeof(valueText),
                     "%d", data->lightLevel);
            ssd1306_display_text(oled, 0, "LIGHT", false);
            ssd1306_display_text(oled, 2, valueText, false);
            break;

        case DisplayMode::MOTION:
            ssd1306_display_text(oled, 0, "MOTION", false);
            ssd1306_display_text(
                oled,
                2,
                data->motionDetected ? "DETECTED" : "NOT DETECTED",
                false
            );
            break;
    }
}

static void DisplayTask(void *pvParameters)
{
    SensorData data = {0.0f, 0.0f, 0, false};
    DisplayMode mode = DisplayMode::TEMPERATURE;

    drawDisplay(&data, mode);

    while (1)
    {
        bool updateDisplay = false;

        DisplayMode newMode;
        if (xQueueReceive(modeQueue, &newMode, 0) == pdPASS)
        {
            mode = newMode;
            updateDisplay = true;
        }

        SensorData newData;
        if (xQueueReceive(displayQueue, &newData, 0) == pdPASS)
        {
            data = newData;
            updateDisplay = true;
        }

        if (updateDisplay)
        {
            drawDisplay(&data, mode);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void InputTask(void *pvParameters)
{
    DisplayMode currentMode = DisplayMode::TEMPERATURE;
    int lastCLK = gpio_get_level(ENCODER_CLK);

    xQueueOverwrite(modeQueue, &currentMode);

    while (1)
    {
        int currentCLK = gpio_get_level(ENCODER_CLK);

        if (currentCLK != lastCLK && currentCLK == 1)
        {
            if (gpio_get_level(ENCODER_DT) != currentCLK)
            {
                switch (currentMode)
                {
                    case DisplayMode::TEMPERATURE:
                        currentMode = DisplayMode::HUMIDITY;
                        break;
                    case DisplayMode::HUMIDITY:
                        currentMode = DisplayMode::LIGHT;
                        break;
                    case DisplayMode::LIGHT:
                        currentMode = DisplayMode::MOTION;
                        break;
                    case DisplayMode::MOTION:
                        currentMode = DisplayMode::TEMPERATURE;
                        break;
                }
            }
            else
            {
                switch (currentMode)
                {
                    case DisplayMode::TEMPERATURE:
                        currentMode = DisplayMode::MOTION;
                        break;
                    case DisplayMode::HUMIDITY:
                        currentMode = DisplayMode::TEMPERATURE;
                        break;
                    case DisplayMode::LIGHT:
                        currentMode = DisplayMode::HUMIDITY;
                        break;
                    case DisplayMode::MOTION:
                        currentMode = DisplayMode::LIGHT;
                        break;
                }
            }

            xQueueOverwrite(modeQueue, &currentMode);
        }

        lastCLK = currentCLK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void AlarmTask(void *pvParameters)
{
    SensorData data;

    while (1)
    {
        if (xQueueReceive(alarmQueue, &data, portMAX_DELAY) == pdPASS)
        {
            if (data.motionDetected && data.lightLevel < 100)
            {
                printf("\n!!! ALARM !!!\n");
                printf("Motion detected in darkness!\n");
            }
            else
            {
                printf("Alarm: NORMAL\n");
            }
        }
    }
}

extern "C" void app_main(void)
{
    printf("\n================================\n");
    printf(" ESP32 FREERTOS SENSOR SYSTEM\n");
    printf("================================\n");

    displayQueue = xQueueCreate(10, sizeof(SensorData));
    alarmQueue = xQueueCreate(10, sizeof(SensorData));
    modeQueue = xQueueCreate(1, sizeof(DisplayMode));

    if (displayQueue == NULL ||
        alarmQueue == NULL ||
        modeQueue == NULL)
    {
        printf("ERROR: Queue creation failed!\n");
        return;
    }

    printf("Queues created successfully.\n");

    oled_init();

    if (oled == NULL)
    {
        printf("ERROR: OLED initialization failed!\n");
        return;
    }

    gpio_config_t encoder_config = {};
    encoder_config.pin_bit_mask =
        (1ULL << ENCODER_CLK) |
        (1ULL << ENCODER_DT);
    encoder_config.mode = GPIO_MODE_INPUT;
    encoder_config.pull_up_en = GPIO_PULLUP_ENABLE;
    encoder_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    encoder_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&encoder_config));

    xTaskCreate(SensorTask, "SensorTask", 4096, NULL, 5, NULL);
    xTaskCreate(DisplayTask, "DisplayTask", 4096, NULL, 4, NULL);
    xTaskCreate(AlarmTask, "AlarmTask", 4096, NULL, 4, NULL);
    xTaskCreate(InputTask, "InputTask", 4096, NULL, 4, NULL);

    printf("All tasks started successfully.\n");
}
