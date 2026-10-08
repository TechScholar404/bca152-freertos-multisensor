#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "dht.h"
#include "ssd1306.h"

#define DHT_PIN GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6
#define ENCODER_CLK GPIO_NUM_18
#define ENCODER_DT GPIO_NUM_19
#define ENCODER_SW GPIO_NUM_23
#define PIR_PIN GPIO_NUM_27
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

enum class DisplayMode { TEMPERATURE, HUMIDITY, LIGHT, MOTION };
enum class AlarmState { NORMAL, LOW_TEMPERATURE, HIGH_TEMPERATURE };
enum class SystemState { ACTIVE, INACTIVE };

DisplayMode currentMode = DisplayMode::TEMPERATURE;
SystemState systemState = SystemState::ACTIVE;
adc_oneshot_unit_handle_t adc_handle;
QueueHandle_t displayQueue, alarmQueue;
ssd1306_handle_t oled;

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < 18.0f) return AlarmState::LOW_TEMPERATURE;
    if (temperature > 30.0f) return AlarmState::HIGH_TEMPERATURE;
    return AlarmState::NORMAL;
}

void readSensors(SensorData *data)
{
    float temperature = 0.0f, humidity = 0.0f;
    esp_err_t result = dht_read_float_data(
        DHT_TYPE_AM2301, DHT_PIN, &humidity, &temperature);

    if (result == ESP_OK) {
        data->temperature = temperature;
        data->humidity = humidity;
    } else {
        printf("DHT22 read failed: %s\n", esp_err_to_name(result));
    }

    int rawLdr = 0;
    adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &rawLdr);

    int lightLevel = 100 - (((rawLdr - 34) * 100) / (4063 - 34));
    if (lightLevel < 0) lightLevel = 0;
    if (lightLevel > 100) lightLevel = 100;

    data->lightLevel = lightLevel;
    data->motionDetected = gpio_get_level(PIR_PIN);

    printf("\nTemperature: %.2f C\n", data->temperature);
    printf("Humidity: %.2f %%\n", data->humidity);
    printf("LDR Raw: %d\n", rawLdr);
    printf("Relative Light Level: %d %%\n", data->lightLevel);
}

void SensorTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    SensorData sensorData;

    for (;;) {
        readSensors(&sensorData);
        xQueueSend(displayQueue, &sensorData, portMAX_DELAY);
        xQueueSend(alarmQueue, &sensorData, portMAX_DELAY);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
    }
}

void DisplayTask(void *pvParameters)
{
    SensorData sensorData;

    for (;;) {
        if (xQueueReceive(displayQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
            if (systemState == SystemState::INACTIVE) continue;

            char line[32];
            ssd1306_clear_screen(oled, 0x00);

            switch (currentMode) {
                case DisplayMode::TEMPERATURE:
                    ssd1306_draw_string(oled, 0, 0,
                        (const uint8_t *)"TEMPERATURE", 16, 1);
                    snprintf(line, sizeof(line), "%.1f C",
                        sensorData.temperature);
                    ssd1306_draw_string(oled, 0, 24,
                        (const uint8_t *)line, 16, 1);
                    break;

                case DisplayMode::HUMIDITY:
                    ssd1306_draw_string(oled, 0, 0,
                        (const uint8_t *)"HUMIDITY", 16, 1);
                    snprintf(line, sizeof(line), "%.1f%%",
                        sensorData.humidity);
                    ssd1306_draw_string(oled, 0, 24,
                        (const uint8_t *)line, 16, 1);
                    break;

                case DisplayMode::LIGHT:
                    ssd1306_draw_string(oled, 0, 0,
                        (const uint8_t *)"LIGHT", 16, 1);
                    snprintf(line, sizeof(line), "%d%%",
                        sensorData.lightLevel);
                    ssd1306_draw_string(oled, 0, 24,
                        (const uint8_t *)line, 16, 1);
                    break;

                case DisplayMode::MOTION:
                    ssd1306_draw_string(oled, 0, 0,
                        (const uint8_t *)"MOTION", 16, 1);
                    ssd1306_draw_string(oled, 0, 24,
                        (const uint8_t *)(sensorData.motionDetected
                            ? "DETECTED" : "NO MOTION"), 16, 1);
                    break;
            }

            ssd1306_refresh_gram(oled);
        }
    }
}

void AlarmTask(void *pvParameters)
{
    SensorData sensorData;

    for (;;) {
        if (xQueueReceive(alarmQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
            if (systemState == SystemState::INACTIVE) continue;

            AlarmState state = evaluateTemperature(sensorData.temperature);
            printf("Alarm Data: Temperature %.2f C\n",
                sensorData.temperature);

            switch (state) {
                case AlarmState::NORMAL:
                    printf("Alarm State: NORMAL\n");
                    break;
                case AlarmState::LOW_TEMPERATURE:
                    printf("Alarm State: LOW TEMPERATURE\n");
                    break;
                case AlarmState::HIGH_TEMPERATURE:
                    printf("Alarm State: HIGH TEMPERATURE\n");
                    break;
            }
        }
    }
}

void InputTask(void *pvParameters)
{
    int lastCLK = gpio_get_level(ENCODER_CLK);

    for (;;) {
        if (systemState == SystemState::INACTIVE) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int currentCLK = gpio_get_level(ENCODER_CLK);
        int currentDT = gpio_get_level(ENCODER_DT);

        if (currentCLK != lastCLK && currentCLK == 1) {
            int mode;

            if (currentDT == 0) {
                mode = static_cast<int>(currentMode) + 1;
                if (mode > static_cast<int>(DisplayMode::MOTION))
                    mode = static_cast<int>(DisplayMode::TEMPERATURE);
            } else {
                mode = static_cast<int>(currentMode) - 1;
                if (mode < static_cast<int>(DisplayMode::TEMPERATURE))
                    mode = static_cast<int>(DisplayMode::MOTION);
            }

            currentMode = static_cast<DisplayMode>(mode);
        }

        lastCLK = currentCLK;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void MotionTask(void *pvParameters)
{
    TickType_t lastMotionTime = xTaskGetTickCount();
    bool motionActive = false;

    for (;;) {
        int motion = gpio_get_level(PIR_PIN);

        if (motion == 1) {
            lastMotionTime = xTaskGetTickCount();

            if (!motionActive) {
                printf("Motion detected - ACTIVE\n");
                systemState = SystemState::ACTIVE;
                motionActive = true;
            }
        } else {
            if (motionActive) {
                motionActive = false;
                lastMotionTime = xTaskGetTickCount();
            }

            if (systemState == SystemState::ACTIVE &&
                (xTaskGetTickCount() - lastMotionTime) >=
                pdMS_TO_TICKS(15000)) {

                printf("No motion for 15 seconds - INACTIVE\n");
                systemState = SystemState::INACTIVE;
                ssd1306_clear_screen(oled, 0x00);
                ssd1306_refresh_gram(oled);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void)
{
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("System starting...\n");

    gpio_config_t encoder_config = {};
    encoder_config.pin_bit_mask =
        (1ULL << ENCODER_CLK) |
        (1ULL << ENCODER_DT) |
        (1ULL << ENCODER_SW);
    encoder_config.mode = GPIO_MODE_INPUT;
    encoder_config.pull_up_en = GPIO_PULLUP_ENABLE;
    encoder_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    encoder_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&encoder_config);

    gpio_config_t pir_config = {};
    pir_config.pin_bit_mask = (1ULL << PIR_PIN);
    pir_config.mode = GPIO_MODE_INPUT;
    pir_config.pull_up_en = GPIO_PULLUP_DISABLE;
    pir_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pir_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pir_config);

    adc_oneshot_unit_init_cfg_t adc_config = {};
    adc_config.unit_id = ADC_UNIT_1;
    adc_oneshot_new_unit(&adc_config, &adc_handle);

    adc_oneshot_chan_cfg_t adc_channel_config = {};
    adc_channel_config.atten = ADC_ATTEN_DB_12;
    adc_channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    adc_oneshot_config_channel(
        adc_handle, LDR_ADC_CHANNEL, &adc_channel_config);

    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_config.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_config.clk_flags = 0;

    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0);

    oled = ssd1306_create(I2C_MASTER_NUM, OLED_ADDRESS);

    if (oled == NULL) {
        printf("Failed to initialize OLED\n");
        return;
    }

    ssd1306_clear_screen(oled, 0x00);
    ssd1306_refresh_gram(oled);

    displayQueue = xQueueCreate(5, sizeof(SensorData));
    alarmQueue = xQueueCreate(5, sizeof(SensorData));

    if (displayQueue == NULL || alarmQueue == NULL) {
        printf("Failed to create sensor queues\n");
        return;
    }

    xTaskCreate(SensorTask, "SensorTask", 4096, NULL, 5, NULL);
    xTaskCreate(DisplayTask, "DisplayTask", 4096, NULL, 4, NULL);
    xTaskCreate(AlarmTask, "AlarmTask", 4096, NULL, 4, NULL);
    xTaskCreate(InputTask, "InputTask", 4096, NULL, 4, NULL);
    xTaskCreate(MotionTask, "MotionTask", 4096, NULL, 4, NULL);
}