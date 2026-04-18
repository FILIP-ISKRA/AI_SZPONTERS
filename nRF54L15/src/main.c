#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "Brak zdefiniowanego zephyr,user io-channels dla ADC w pliku .overlay!"
#endif

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int main(void)
{
    /* Pobieramy podzespoly z naszego pliku overlay */
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

    /* Konfiguracja ADC */
    int16_t adc_sample_buffer;
    struct adc_sequence adc_seq = {
        .buffer = &adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
    };

    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("Blad! Urzadzenie ADC nie gotowe.");
    } else {
        int err = adc_channel_setup_dt(&adc_channel);
        if (err < 0) {
            LOG_ERR("Nie mozna skonfigurowac kanalu ADC: %d", err);
        } else {
            LOG_INF("Sukces! Kanal ADC skonfigurowany.");
        }
        adc_sequence_init_dt(&adc_channel, &adc_seq);
    }

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

        if (adc_is_ready_dt(&adc_channel)) {
            int err = adc_read_dt(&adc_channel, &adc_seq);
            if (err == 0) {
                int32_t val_mv = adc_sample_buffer;
                LOG_INF("ADC: wartosc RAW: %d", adc_sample_buffer);
                
                adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
                LOG_INF("ADC: %.3f V", val_mv / 1000.0); // albo mV
            } else {
                LOG_ERR("Blad pomiaru ADC: %d", err);
            }
        }

        k_sleep(K_MSEC(100)); // Czekamy 100 ms przed kolejnym pomiarem
    }
    
    return 0;
}