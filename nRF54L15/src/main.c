#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

/* Sensory */
const struct device *const bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *const vl53l0x = DEVICE_DT_GET_ANY(st_vl53l0x);

/* UUID Serwisu i Charakterystyki dla danych sensorów */
static struct bt_uuid_128 sensor_svc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678));

static struct bt_uuid_128 distance_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679));

static uint16_t last_distance_mm = 0;

/* GATT: Odczyt dystansu */
static ssize_t read_distance(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_distance_mm, sizeof(last_distance_mm));
}

/* Definicja serwisu GATT */
BT_GATT_SERVICE_DEFINE(sensor_svc,
                       BT_GATT_PRIMARY_SERVICE(&sensor_svc_uuid),
                       BT_GATT_CHARACTERISTIC(&distance_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_distance, NULL, &last_distance_mm), );

/* Rozgłaszanie (Advertising) */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'n', 'R', 'F', '5', '4', 'L', '1', '5'),
};

static void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    /* Używamy standardowego makra dla urządzeń połączalnych */
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started");
}

int main(void)
{
    int err;
    struct sensor_value distance;

    LOG_INF("Starting Bluetooth Sensor App on nRF54L15");

    /* Inicjalizacja BT */
    err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
        return 0;
    }

    /* Sprawdzamy czy laser działa (nie zatrzymujemy programu jeśli nie, żeby BT działało) */
    if (!device_is_ready(vl53l0x))
    {
        LOG_ERR("VL53L0X nie jest gotowy. Sprawdz polaczenia na P1.12 i P1.13");
    }

    while (1)
    {
        if (device_is_ready(vl53l0x))
        {
            sensor_sample_fetch(vl53l0x);
            sensor_channel_get(vl53l0x, SENSOR_CHAN_DISTANCE, &distance);

            last_distance_mm = (uint16_t)(distance.val1 * 1000 + distance.val2 / 1000);
            LOG_INF("Distance: %d mm", last_distance_mm);

            /* Wypychamy powiadomienie (Notify) do połączonych urządzeń */
            bt_gatt_notify(NULL, &sensor_svc.attrs[2], &last_distance_mm, sizeof(last_distance_mm));
        }

        k_sleep(K_MSEC(1000));
    }
    return 0;
}