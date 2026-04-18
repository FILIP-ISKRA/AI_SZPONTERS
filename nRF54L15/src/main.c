#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    /* Pobieramy czujnik BME280 z naszego pliku overlay */
    const struct device *const bme280_dev = DEVICE_DT_GET_ANY(bosch_bme280);

    if (bme280_dev == NULL || !device_is_ready(bme280_dev)) {
        LOG_ERR("Blad! Nie mozna zainicjowac BME280.");
        return 0; 
    }

    LOG_INF("Sukces! BME280 gotowy. Rozpoczynam pomiary...\n");

    struct sensor_value temp, press, humidity;

    /* Nieskończona pętla odczytująca dane co 2 sekundy */
    while (1) {
        /* Pobranie nowej próbki z czujnika */
        sensor_sample_fetch(bme280_dev);

        /* Wyciągnięcie konkretnych wartości */
        sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press);
        sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &humidity);

        /* Wyświetlenie w terminalu (ciśnienie mnożymy x10, bo Zephyr podaje w kPa) */
        LOG_INF("Temperatura: %.2f C | Cisnienie: %.2f hPa | Wilgotnosc: %.2f %%",
                sensor_value_to_double(&temp),
                sensor_value_to_double(&press) * 10.0,
                sensor_value_to_double(&humidity));

        k_sleep(K_SECONDS(0.5));
    }
    
    return 0;
}