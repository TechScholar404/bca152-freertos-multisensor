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

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(20000);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    int timeout = 100;

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
            data[i / 8] |= (1 << (7 - (i % 8)));

        timeout = 100;

        while (gpio_get_level(DHT_PIN) == 1 && timeout--)
            esp_rom_delay_us(1);

        if (timeout <= 0)
            return false;
    }

    uint8_t checksum = data[0] + data[1] + data[2] + data[3];

    if (checksum != data[4]) {
        ESP_LOGE(TAG, "DHT22 checksum error");
        return false;
    }

    *humidity = ((data[0] << 8) | data[1]) / 10.0f;

    int16_t raw_temperature =
        ((data[2] & 0x7F) << 8) | data[3];

    *temperature = raw_temperature / 10.0f;

    if (data[2] & 0x80)
        *temperature = -*temperature;

    return true;
}

static void ldr_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&init_config, &adc_handle)
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
    int raw_value = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(
            adc_handle,
            LDR_ADC_CHANNEL,
            &raw_value
        )
    );

    int percent = (raw_value * 100) / 4095;

    if (percent < 0)
        percent = 0;

    if (percent > 100)
        percent = 100;

    return percent;
}

static void oled_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(
        i2c_param_config(I2C_MASTER_NUM, &conf)
    );

    ESP_ERROR_CHECK(
        i2c_driver_install(
            I2C_MASTER_NUM,
            conf.mode,
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

static void DisplayTask(void *pvParameters)
{
    SensorData data;

    for (;;) {
        if (xQueueReceive(
                displayQueue,
                &data,
                portMAX_DELAY
            ) == pdTRUE) {

            ESP_LOGI(
                TAG,
                "DISPLAY -> Temp: %.2f C | Humidity: %.2f %% | Light: %d %% | Motion: %s",
                data.temperature,
                data.humidity,
                data.lightLevel,
                data.motionDetected ? "YES" : "NO"
            );

            if (oled != NULL) {
                char temp[20];

                snprintf(
                    temp,
                    sizeof(temp),
                    "%.1f C",
                    data.temperature
                );

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
                    (const uint8_t *)"Temperature",
                    16,
                    1
                );

                ssd1306_draw_string(
                    oled,
                    0,
                    40,
                    (const uint8_t *)temp,
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
            ) == pdTRUE) {

            if (data.motionDetected) {
                ESP_LOGW(
                    TAG,
                    "ALARM -> Motion detected!"
                );
            } else {
                ESP_LOGI(
                    TAG,
                    "ALARM -> No motion"
                );
            }
        }
    }
}

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

    ldr_init();
    oled_init();

    displayQueue = xQueueCreate(
        5,
        sizeof(SensorData)
    );

    alarmQueue = xQueueCreate(
        5,
        sizeof(SensorData)
    );

    if (displayQueue == NULL ||
        alarmQueue == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create sensor queues"
        );

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

    ESP_LOGI(
        TAG,
        "All tasks created"
    );
}