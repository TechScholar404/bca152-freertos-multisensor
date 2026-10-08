#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dht.h"

#define DHT_PIN GPIO_NUM_4

void app_main(void)
{
    printf("BCA152 DHT22 Test\n");

    while (1)
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
            printf("Temperature: %.2f C\n", temperature);
            printf("Humidity: %.2f %%\n", humidity);
        }
        else
        {
            printf("DHT22 read failed: %s\n", esp_err_to_name(result));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}