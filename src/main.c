#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define DHT_PIN GPIO_NUM_4

static const char *TAG = "DHT22";


/*
 * Wait until GPIO reaches a level.
 */
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


/*
 * Measure how long GPIO stays at a level.
 */
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


/*
 * Read DHT22.
 */
static int dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};


    /*
     * -----------------------------------------
     * START SIGNAL
     * -----------------------------------------
     */

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(DHT_PIN, 0);

    /* DHT22 requires at least 1 ms LOW */
    esp_rom_delay_us(2000);

    gpio_set_level(DHT_PIN, 1);

    esp_rom_delay_us(30);


    /*
     * Release DATA line.
     */
    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_set_pull_mode(
        DHT_PIN,
        GPIO_PULLUP_ONLY
    );


    /*
     * -----------------------------------------
     * DHT22 RESPONSE
     * -----------------------------------------
     */

    /*
     * Sensor pulls LOW for ~80 us.
     */
    if (wait_level(0, 200) != 0)
    {
        ESP_LOGE(TAG, "No response from DHT22");
        return -1;
    }


    /*
     * Sensor pulls HIGH for ~80 us.
     */
    if (wait_level(1, 200) != 0)
    {
        ESP_LOGE(TAG, "DHT22 response HIGH timeout");
        return -1;
    }


    /*
     * Sensor starts data with LOW ~50 us.
     */
    if (wait_level(0, 200) != 0)
    {
        ESP_LOGE(TAG, "DHT22 data start timeout");
        return -1;
    }


    /*
     * -----------------------------------------
     * READ 40 BITS
     * -----------------------------------------
     */

    for (int i = 0; i < 40; i++)
    {
        /*
         * Every bit starts LOW.
         */
        if (wait_level(1, 100) != 0)
        {
            ESP_LOGE(
                TAG,
                "Timeout waiting for bit %d",
                i
            );

            return -1;
        }


        /*
         * Measure HIGH duration.
         *
         * 0 = approximately 26 us
         * 1 = approximately 70 us
         */
        int high_time = measure_level(1, 100);

        if (high_time < 0)
        {
            ESP_LOGE(
                TAG,
                "Invalid pulse at bit %d",
                i
            );

            return -1;
        }


        data[i / 8] <<= 1;

        if (high_time > 45)
        {
            data[i / 8] |= 1;
        }
    }


    /*
     * -----------------------------------------
     * CHECKSUM
     * -----------------------------------------
     */

    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];


    if (checksum != data[4])
    {
        ESP_LOGE(
            TAG,
            "Checksum error: calc=%02X received=%02X",
            checksum,
            data[4]
        );

        return -2;
    }


    /*
     * -----------------------------------------
     * CONVERT HUMIDITY
     * -----------------------------------------
     */

    uint16_t raw_humidity =
        ((uint16_t)data[0] << 8) |
        data[1];

    *humidity =
        raw_humidity / 10.0f;


    /*
     * -----------------------------------------
     * CONVERT TEMPERATURE
     * -----------------------------------------
     */

    uint16_t raw_temperature =
        ((uint16_t)(data[2] & 0x7F) << 8) |
        data[3];

    *temperature =
        raw_temperature / 10.0f;


    /*
     * Negative temperature.
     */
    if (data[2] & 0x80)
    {
        *temperature =
            -*temperature;
    }


    return 0;
}


/*
 * =========================================
 * ESP-IDF APPLICATION
 * =========================================
 */

void app_main(void)
{
    float temperature = 0.0f;
    float humidity = 0.0f;


    printf("\n");
    printf("====================================\n");
    printf("       PART IV - DHT22 SENSOR\n");
    printf("====================================\n");
    printf("DHT22 DATA PIN: GPIO 4\n");
    printf("====================================\n");


    /*
     * Configure GPIO4.
     */
    gpio_reset_pin(DHT_PIN);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_set_pull_mode(
        DHT_PIN,
        GPIO_PULLUP_ONLY
    );


    /*
     * Wait for sensor startup.
     */
    vTaskDelay(
        pdMS_TO_TICKS(2000)
    );


    while (1)
    {
        int result =
            dht22_read(
                &temperature,
                &humidity
            );


        if (result == 0)
        {
            printf("\n");
            printf("==============================\n");
            printf("DHT22 SENSOR READING\n");
            printf("==============================\n");

            printf(
                "Temperature: %.2f C\n",
                temperature
            );

            printf(
                "Humidity: %.2f %%\n",
                humidity
            );

            printf("==============================\n");
        }
        else
        {
            printf(
                "\nDHT22 READ ERROR: %d\n",
                result
            );
        }


        /*
         * DHT22 should be read no faster
         * than about once every 2 seconds.
         */
        vTaskDelay(
            pdMS_TO_TICKS(2500)
        );
    }
}