#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define DHT_PIN         GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

static const char *TAG = "SENSOR";

static adc_oneshot_unit_handle_t adc_handle;

static int wait_level(int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) != level)
    {
        if ((esp_timer_get_time() - start) >= timeout_us)
        {
            return -1;
        }
    }

    return 0;
}

static int measure_level(int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == level)
    {
        if ((esp_timer_get_time() - start) >= timeout_us)
        {
            return -1;
        }
    }

    return (int)(esp_timer_get_time() - start);
}

static int dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(2000);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);

    if (wait_level(0, 200) != 0)
        return -1;

    if (wait_level(1, 200) != 0)
        return -1;

    if (wait_level(0, 200) != 0)
        return -1;

    for (int i = 0; i < 40; i++)
    {
        if (wait_level(1, 100) != 0)
            return -1;

        int high_time = measure_level(1, 100);

        if (high_time < 0)
            return -1;

        data[i / 8] <<= 1;

        if (high_time > 45)
            data[i / 8] |= 1;
    }

    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
    {
        ESP_LOGE(
            TAG,
            "DHT22 checksum error: calculated=%02X received=%02X",
            checksum,
            data[4]
        );

        return -2;
    }

    uint16_t raw_humidity =
        ((uint16_t)data[0] << 8) | data[1];

    *humidity = raw_humidity / 10.0f;

    uint16_t raw_temperature =
        ((uint16_t)(data[2] & 0x7F) << 8) | data[3];

    *temperature = raw_temperature / 10.0f;

    if (data[2] & 0x80)
        *temperature = -*temperature;

    return 0;
}

static float ldr_read_percent(int *raw_value)
{
    int raw = 0;

    esp_err_t result = adc_oneshot_read(
        adc_handle,
        LDR_ADC_CHANNEL,
        &raw
    );

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "LDR ADC read failed");
        return 0.0f;
    }

    if (raw < 0)
        raw = 0;

    if (raw > 4095)
        raw = 4095;

    if (raw_value != NULL)
        *raw_value = raw;

    float light_percent =
        100.0f -
        ((float)raw / 4095.0f * 100.0f);

    if (light_percent < 0.0f)
        light_percent = 0.0f;

    if (light_percent > 100.0f)
        light_percent = 100.0f;

    return light_percent;
}

static void dht22_init(void)
{
    gpio_reset_pin(DHT_PIN);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_set_pull_mode(
        DHT_PIN,
        GPIO_PULLUP_ONLY
    );
}

static void ldr_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_config,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &channel_config
        )
    );
}

void app_main(void)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    int ldr_raw = 0;
    float light_percent = 0.0f;

    printf("\n");
    printf("========================================\n");
    printf("       PART IV - SENSOR SUBSYSTEM\n");
    printf("========================================\n");
    printf("DHT22 : GPIO4\n");
    printf("LDR   : GPIO34 / ADC1_CH6\n");
    printf("========================================\n");

    dht22_init();
    ldr_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        printf("\n");
        printf("========================================\n");
        printf("           SENSOR READINGS\n");
        printf("========================================\n");

        int dht_result = dht22_read(
            &temperature,
            &humidity
        );

        if (dht_result == 0)
        {
            printf(
                "Temperature : %.2f C\n",
                temperature
            );

            printf(
                "Humidity    : %.2f %%\n",
                humidity
            );
        }
        else
        {
            printf(
                "DHT22       : READ ERROR (%d)\n",
                dht_result
            );
        }

        light_percent =
            ldr_read_percent(&ldr_raw);

        printf(
            "LDR Raw     : %d\n",
            ldr_raw
        );

        printf(
            "Light Level : %.1f %%\n",
            light_percent
        );

        printf("========================================\n");

        vTaskDelay(
            pdMS_TO_TICKS(2000)
        );
    }
}