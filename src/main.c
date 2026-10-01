#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "ssd1306.h"

#define DHT_PIN GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 100000
#define OLED_I2C_ADDRESS 0x3C

#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT GPIO_NUM_33

static const char *TAG = "SENSOR";

static adc_oneshot_unit_handle_t adc_handle;
static QueueHandle_t displayQueue;
static QueueHandle_t alarmQueue;
static ssd1306_handle_t oled;

typedef struct {
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
} SensorData;

typedef enum {
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
} DisplayMode;

typedef enum {
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
} AlarmState;

static DisplayMode currentDisplayMode = TEMPERATURE;

static AlarmState evaluateTemperature(float temperature)
{
    if (temperature < 18.0f)
        return LOW_TEMPERATURE;

    if (temperature > 30.0f)
        return HIGH_TEMPERATURE;

    return NORMAL;
}

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};
    int timeout = 100;

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(20000);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    while (gpio_get_level(DHT_PIN) == 1 && timeout--)
        esp_rom_delay_us(1);

    timeout = 100;

    while (gpio_get_level(DHT_PIN) == 0 && timeout--)
        esp_rom_delay_us(1);

    timeout = 100;

    while (gpio_get_level(DHT_PIN) == 1 && timeout--)
        esp_rom_delay_us(1);

    if (timeout <= 0)
        return false;

    for (int i = 0; i < 40; i++) {
        timeout = 100;

        while (gpio_get_level(DHT_PIN) == 0 && timeout--)
            esp_rom_delay_us(1);

        if (timeout <= 0)
            return false;

        esp_rom_delay_us(40);

        if (gpio_get_level(DHT_PIN))
            data[i / 8] |= 1 << (7 - i % 8);

        timeout = 100;

        while (gpio_get_level(DHT_PIN) == 1 && timeout--)
            esp_rom_delay_us(1);

        if (timeout <= 0)
            return false;
    }

    if (data[0] + data[1] + data[2] + data[3] != data[4]) {
        ESP_LOGE(TAG, "DHT22 checksum error");
        return false;
    }

    *humidity = ((data[0] << 8) | data[1]) / 10.0f;

    int16_t rawTemperature =
        ((data[2] & 0x7F) << 8) | data[3];

    *temperature = rawTemperature / 10.0f;

    if (data[2] & 0x80)
        *temperature = -*temperature;

    return true;
}

static void ldr_init(void)
{
    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&initConfig, &adc_handle)
    );

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &config
        )
    );

    ESP_LOGI(TAG, "LDR ADC initialized");
}

static int ldr_read_percent(void)
{
    int rawValue = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(
            adc_handle,
            LDR_ADC_CHANNEL,
            &rawValue
        )
    );

    int percent = rawValue * 100 / 4095;

    if (percent < 0)
        percent = 0;

    if (percent > 100)
        percent = 100;

    return percent;
}

static void oled_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(
        i2c_param_config(I2C_MASTER_NUM, &config)
    );

    ESP_ERROR_CHECK(
        i2c_driver_install(
            I2C_MASTER_NUM,
            config.mode,
            0,
            0,
            0
        )
    );

    oled = ssd1306_create(
        I2C_MASTER_NUM,
        OLED_I2C_ADDRESS
    );

    if (oled == NULL) {
        ESP_LOGE(TAG, "Failed to create SSD1306");
        return;
    }

    ssd1306_clear_screen(oled, 0x00);
    ssd1306_refresh_gram(oled);

    ESP_LOGI(TAG, "OLED initialized");
}

static void encoder_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask =
            (1ULL << ENCODER_CLK) |
            (1ULL << ENCODER_DT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_LOGI(TAG, "Rotary encoder initialized");
}

static SensorData readSensors(void)
{
    SensorData data = {
        .temperature = 0.0f,
        .humidity = 0.0f,
        .lightLevel = 0,
        .motionDetected = false
    };

    data.lightLevel = ldr_read_percent();

    if (dht22_read(&data.temperature, &data.humidity)) {
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
    } else {
        ESP_LOGE(TAG, "DHT22 read failed");
    }

    ESP_LOGI(
        TAG,
        "Relative Light Level: %d %%",
        data.lightLevel
    );

    return data;
}

static void SensorTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    SensorData data;

    for (;;) {
        data = readSensors();

        xQueueSend(
            displayQueue,
            &data,
            pdMS_TO_TICKS(100)
        );

        xQueueSend(
            alarmQueue,
            &data,
            pdMS_TO_TICKS(100)
        );

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

static void InputTask(void *pvParameters)
{
    int lastCLK = gpio_get_level(ENCODER_CLK);

    for (;;) {
        int currentCLK = gpio_get_level(ENCODER_CLK);

        if (currentCLK != lastCLK) {
            int currentDT = gpio_get_level(ENCODER_DT);

            if (currentCLK == 1) {
                if (currentDT != currentCLK) {
                    if (currentDisplayMode == MOTION)
                        currentDisplayMode = TEMPERATURE;
                    else
                        currentDisplayMode++;
                } else {
                    if (currentDisplayMode == TEMPERATURE)
                        currentDisplayMode = MOTION;
                    else
                        currentDisplayMode--;
                }

                ESP_LOGI(
                    TAG,
                    "Display mode: %d",
                    currentDisplayMode
                );
            }

            lastCLK = currentCLK;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void DisplayTask(void *pvParameters)
{
    SensorData data;

    for (;;) {
        if (xQueueReceive(
                displayQueue,
                &data,
                portMAX_DELAY
            )) {

            char label[20];
            char value[20];

            switch (currentDisplayMode) {
                case TEMPERATURE:
                    snprintf(
                        label,
                        sizeof(label),
                        "Temperature"
                    );
                    snprintf(
                        value,
                        sizeof(value),
                        "%.1f C",
                        data.temperature
                    );
                    break;

                case HUMIDITY:
                    snprintf(
                        label,
                        sizeof(label),
                        "Humidity"
                    );
                    snprintf(
                        value,
                        sizeof(value),
                        "%.1f %%",
                        data.humidity
                    );
                    break;

                case LIGHT:
                    snprintf(
                        label,
                        sizeof(label),
                        "Light"
                    );
                    snprintf(
                        value,
                        sizeof(value),
                        "%d %%",
                        data.lightLevel
                    );
                    break;

                case MOTION:
                    snprintf(
                        label,
                        sizeof(label),
                        "Motion"
                    );
                    snprintf(
                        value,
                        sizeof(value),
                        "%s",
                        data.motionDetected ? "YES" : "NO"
                    );
                    break;

                default:
                    snprintf(
                        label,
                        sizeof(label),
                        "Temperature"
                    );
                    snprintf(
                        value,
                        sizeof(value),
                        "%.1f C",
                        data.temperature
                    );
                    break;
            }

            if (oled != NULL) {
                ssd1306_clear_screen(oled, 0x00);

                ssd1306_draw_string(
                    oled,
                    0,
                    0,
                    (const uint8_t *)"ROOM MONITOR",
                    16,
                    1
                );

                ssd1306_draw_string(
                    oled,
                    0,
                    24,
                    (const uint8_t *)label,
                    16,
                    1
                );

                ssd1306_draw_string(
                    oled,
                    0,
                    40,
                    (const uint8_t *)value,
                    16,
                    1
                );

                ssd1306_refresh_gram(oled);
            }
        }
    }
}

static void AlarmTask(void *pvParameters)
{
    SensorData data;

    for (;;) {
        if (xQueueReceive(
                alarmQueue,
                &data,
                portMAX_DELAY
            )) {

            AlarmState state =
                evaluateTemperature(data.temperature);

            switch (state) {
                case LOW_TEMPERATURE:
                    ESP_LOGW(
                        TAG,
                        "ALARM -> Low temperature"
                    );
                    break;

                case HIGH_TEMPERATURE:
                    ESP_LOGW(
                        TAG,
                        "ALARM -> High temperature"
                    );
                    break;

                case NORMAL:
                    ESP_LOGI(
                        TAG,
                        "ALARM -> Normal temperature"
                    );
                    break;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System starting...");

    ldr_init();
    oled_init();
    encoder_init();

    displayQueue = xQueueCreate(5, sizeof(SensorData));
    alarmQueue = xQueueCreate(5, sizeof(SensorData));

    if (displayQueue == NULL || alarmQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queues");
        return;
    }

    xTaskCreate(
        SensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        DisplayTask,
        "DisplayTask",
        4096,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        AlarmTask,
        "AlarmTask",
        4096,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        InputTask,
        "InputTask",
        4096,
        NULL,
        1,
        NULL
    );

    ESP_LOGI(TAG, "All tasks created");
}