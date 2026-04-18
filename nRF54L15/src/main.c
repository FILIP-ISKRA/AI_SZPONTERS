#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    /* Pobieramy czujnik BME280 z naszego pliku overlay */
    const struct device *const bme280_dev = DEVICE_DT_GET_ANY(bosch_bme280);
    const struct device *const vl53l0x_dev = DEVICE_DT_GET_ANY(st_vl53l0x);

    if (bme280_dev == NULL || !device_is_ready(bme280_dev)) {
        LOG_ERR("Blad! Nie mozna zainicjowac BME280.");
    } else {
        LOG_INF("Sukces! BME280 gotowy.");
    }

    if (vl53l0x_dev == NULL || !device_is_ready(vl53l0x_dev)) {
        LOG_ERR("Blad! Nie mozna zainicjowac VL53L0X. Sprawdz piny P1.12, P1.13.");
    } else {
        LOG_INF("Sukces! VL53L0X gotowy.");
    }

    LOG_INF("Rozpoczynam pomiary...\n");

    struct sensor_value temp, press, humidity;
    struct sensor_value distance;

    /* Nieskończona pętla odczytująca dane co 2 sekundy */
    while (1) {
        if (device_is_ready(bme280_dev)) {
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
        }

        if (device_is_ready(vl53l0x_dev)) {
            sensor_sample_fetch(vl53l0x_dev);
            sensor_channel_get(vl53l0x_dev, SENSOR_CHAN_DISTANCE, &distance);
            LOG_INF("Dystans VL53L0X: %.3f m", sensor_value_to_double(&distance));
        }

        k_sleep(K_MSEC(100)); // Czekamy 100 ms przed kolejnym pomiarem
    }
    
    return 0;
}