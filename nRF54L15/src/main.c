#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>
#include <stdio.h>
#include <string.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/uri_msg.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

#define SENSOR_LOG_INF(...)                 \
    do                                      \
    {                                       \
        if (IS_ENABLED(CONFIG_LOG_SENSORS)) \
        {                                   \
            LOG_INF(__VA_ARGS__);           \
        }                                   \
    } while (0)

/* Sensory */
const struct device *const bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *const vl53l0x = DEVICE_DT_GET_ANY(st_vl53l0x);

/* UUID Serwisu i Charakterystyki dla danych sensorów */
static struct bt_uuid_128 sensor_svc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678));

static struct bt_uuid_128 distance_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679));

static struct bt_uuid_128 light_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567A));

static struct bt_uuid_128 temp_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567B));

static struct bt_uuid_128 humidity_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567C));

static struct bt_uuid_128 pressure_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567D));

static uint16_t last_distance_mm = 0;
static uint16_t last_light_raw = 0;
static int16_t last_temp_centi = 0;
static uint16_t last_humidity_centi = 0;
static uint32_t last_pressure_pa = 0;

#define NDEF_MSG_BUF_SIZE 256
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];
#define NFC_PAIR_URL_BASE "http://heimdall.local/api/pair"
static char ble_mac_str[BT_ADDR_LE_STR_LEN] = "00:00:00:00:00:00";
static char nfc_pair_url[256];

static bool notify_distance_enabled;
static bool notify_temp_enabled;
static bool notify_humidity_enabled;
static bool notify_pressure_enabled;
static bool notify_light_enabled;

static bool sensor_distance_available;
static bool sensor_temp_available;
static bool sensor_humidity_available;
static bool sensor_pressure_available;
static bool sensor_light_available;

static const struct adc_dt_spec light_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static int16_t light_sample_buffer;

/* GATT: Odczyt dystansu */
static ssize_t read_distance(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_distance_mm, sizeof(last_distance_mm));
}

static ssize_t read_light(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_light_raw, sizeof(last_light_raw));
}

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_temp_centi, sizeof(last_temp_centi));
}

static ssize_t read_humidity(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_humidity_centi, sizeof(last_humidity_centi));
}

static ssize_t read_pressure(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_pressure_pa, sizeof(last_pressure_pa));
}

static void distance_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_distance_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_temp_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void humidity_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_humidity_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void pressure_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_pressure_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void light_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_light_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void nfc_callback(void *context, nfc_t2t_event_t event,
                         const uint8_t *data, size_t data_length)
{
    ARG_UNUSED(context);
    ARG_UNUSED(data);
    ARG_UNUSED(data_length);

    switch (event)
    {
    case NFC_T2T_EVENT_FIELD_ON:
        LOG_INF("NFC field detected");
        break;
    case NFC_T2T_EVENT_FIELD_OFF:
        LOG_INF("NFC field lost");
        break;
    default:
        break;
    }
}

static void append_sensor_name(char *buffer, size_t buffer_size,
                               const char *name, bool *first)
{
    size_t used = strlen(buffer);

    if (used >= buffer_size - 1)
    {
        return;
    }

    if (!(*first))
    {
        snprintf(buffer + used, buffer_size - used, ",");
        used = strlen(buffer);
        if (used >= buffer_size - 1)
        {
            return;
        }
    }

    snprintf(buffer + used, buffer_size - used, "%s", name);
    *first = false;
}

static void build_nfc_pair_url_from_available_sensors(void)
{
    char sensors_param[96] = {0};
    bool first = true;

    if (sensor_distance_available)
    {
        append_sensor_name(sensors_param, sizeof(sensors_param), "distance", &first);
    }
    if (sensor_temp_available)
    {
        append_sensor_name(sensors_param, sizeof(sensors_param), "temp", &first);
    }
    if (sensor_humidity_available)
    {
        append_sensor_name(sensors_param, sizeof(sensors_param), "humidity", &first);
    }
    if (sensor_pressure_available)
    {
        append_sensor_name(sensors_param, sizeof(sensors_param), "pressure", &first);
    }
    if (sensor_light_available)
    {
        append_sensor_name(sensors_param, sizeof(sensors_param), "light", &first);
    }

    if (first)
    {
        snprintf(nfc_pair_url, sizeof(nfc_pair_url), "%s?mac=%s&sensors=none",
                 NFC_PAIR_URL_BASE, ble_mac_str);
    }
    else
    {
        snprintf(nfc_pair_url, sizeof(nfc_pair_url), "%s?mac=%s&sensors=%s",
                 NFC_PAIR_URL_BASE, ble_mac_str, sensors_param);
    }
}

static void update_local_ble_mac(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = ARRAY_SIZE(addrs);
    char addr_le_str[BT_ADDR_LE_STR_LEN];
    char *type_sep;

    bt_id_get(addrs, &count);
    if (count == 0)
    {
        LOG_ERR("Cannot read local BLE identity address");
        return;
    }

    bt_addr_le_to_str(&addrs[0], addr_le_str, sizeof(addr_le_str));
    type_sep = strchr(addr_le_str, ' ');
    if (type_sep)
    {
        *type_sep = '\0';
    }

    snprintf(ble_mac_str, sizeof(ble_mac_str), "%s", addr_le_str);
    LOG_INF("Local BLE MAC: %s", ble_mac_str);
}

static int nfc_update_url_payload(void)
{
    int err;
    uint32_t len = sizeof(ndef_msg_buf);

    err = nfc_ndef_uri_msg_encode(NFC_URI_NONE,
                                  (const uint8_t *)nfc_pair_url,
                                  strlen(nfc_pair_url),
                                  ndef_msg_buf,
                                  &len);
    if (err)
    {
        LOG_ERR("NFC URI encode failed (%d)", err);
        return err;
    }

    err = nfc_t2t_payload_set(ndef_msg_buf, len);
    if (err)
    {
        LOG_ERR("NFC payload set failed (%d)", err);
        return err;
    }

    return 0;
}

static int nfc_init_url_tag(void)
{
    int err;

    err = nfc_t2t_setup(nfc_callback, NULL);
    if (err)
    {
        LOG_ERR("NFC setup failed (%d)", err);
        return err;
    }

    err = nfc_update_url_payload();
    if (err)
    {
        return err;
    }

    err = nfc_t2t_emulation_start();
    if (err)
    {
        LOG_ERR("NFC emulation start failed (%d)", err);
        return err;
    }

    LOG_INF("NFC URI ready: %s", nfc_pair_url);
    return 0;
}

static void detect_and_store_available_sensors(bool adc_ready)
{
    sensor_distance_available = device_is_ready(vl53l0x);

    sensor_temp_available = device_is_ready(bme280);
    sensor_humidity_available = sensor_temp_available;
    sensor_pressure_available = sensor_temp_available;

    sensor_light_available = adc_ready;

    SENSOR_LOG_INF("Sensors available: distance=%d temp=%d humidity=%d pressure=%d light=%d",
                   sensor_distance_available,
                   sensor_temp_available,
                   sensor_humidity_available,
                   sensor_pressure_available,
                   sensor_light_available);
}

/* Definicja serwisu GATT */
BT_GATT_SERVICE_DEFINE(sensor_svc,
                       BT_GATT_PRIMARY_SERVICE(&sensor_svc_uuid),
                       BT_GATT_CHARACTERISTIC(&distance_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_distance, NULL, &last_distance_mm),
                       BT_GATT_CCC(distance_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_temp, NULL, &last_temp_centi),
                       BT_GATT_CCC(temp_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(&humidity_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_humidity, NULL, &last_humidity_centi),
                       BT_GATT_CCC(humidity_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(&pressure_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_pressure, NULL, &last_pressure_pa),
                       BT_GATT_CCC(pressure_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(&light_char_uuid.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              read_light, NULL, &last_light_raw),
                       BT_GATT_CCC(light_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

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
    int ret;
    struct sensor_value distance;
    struct sensor_value temp;
    struct sensor_value humidity;
    struct sensor_value pressure;
    struct adc_sequence light_sequence = {
        .buffer = &light_sample_buffer,
        .buffer_size = sizeof(light_sample_buffer),
    };
    bool adc_ready = false;

    LOG_INF("Starting Bluetooth Sensor App on nRF54L15");

    /* Inicjalizacja BT */
    err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
        return 0;
    }

    update_local_ble_mac();

    /* Sprawdzamy czy laser działa (nie zatrzymujemy programu jeśli nie, żeby BT działało) */
    if (!device_is_ready(vl53l0x))
    {
        LOG_ERR("VL53L0X nie jest gotowy. Sprawdz polaczenia na P1.12 i P1.13");
    }

    if (!device_is_ready(bme280))
    {
        LOG_ERR("BME280 nie jest gotowy. Sprawdz polaczenia na P1.10 i P1.11");
    }

    if (!adc_is_ready_dt(&light_adc))
    {
        LOG_ERR("ADC dla czujnika swiatla nie jest gotowy");
    }
    else
    {
        adc_ready = true;
        ret = adc_channel_setup_dt(&light_adc);
        if (ret < 0)
        {
            LOG_ERR("Blad konfiguracji ADC kanału: %d", ret);
            adc_ready = false;
        }

        light_sequence.channels = BIT(light_adc.channel_id);
        light_sequence.resolution = light_adc.resolution;
        light_sequence.oversampling = light_adc.oversampling;
    }

    detect_and_store_available_sensors(adc_ready);
    build_nfc_pair_url_from_available_sensors();

    err = nfc_init_url_tag();
    if (err)
    {
        LOG_ERR("NFC init error (%d), BLE sensor app will continue", err);
    }

    while (1)
    {
        if (device_is_ready(vl53l0x))
        {
            sensor_sample_fetch(vl53l0x);
            sensor_channel_get(vl53l0x, SENSOR_CHAN_DISTANCE, &distance);

            last_distance_mm = (uint16_t)(distance.val1 * 1000 + distance.val2 / 1000);
            SENSOR_LOG_INF("Distance: %d mm", last_distance_mm);

            /* Wypychamy powiadomienie (Notify) do połączonych urządzeń */
            if (notify_distance_enabled)
            {
                bt_gatt_notify_uuid(NULL, &distance_char_uuid.uuid, &sensor_svc.attrs[0],
                                    &last_distance_mm, sizeof(last_distance_mm));
            }
        }

        if (device_is_ready(bme280))
        {
            sensor_sample_fetch(bme280);
            sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            sensor_channel_get(bme280, SENSOR_CHAN_HUMIDITY, &humidity);
            sensor_channel_get(bme280, SENSOR_CHAN_PRESS, &pressure);

            last_temp_centi = (int16_t)(temp.val1 * 100 + temp.val2 / 10000);
            last_humidity_centi = (uint16_t)(humidity.val1 * 100 + humidity.val2 / 10000);
            last_pressure_pa = (uint32_t)(pressure.val1 * 1000 + pressure.val2 / 1000);

            SENSOR_LOG_INF("Temp(cC): %d, Hum(c%%): %u, Press(Pa): %u",
                           last_temp_centi, last_humidity_centi, last_pressure_pa);
            if (notify_temp_enabled)
            {
                bt_gatt_notify_uuid(NULL, &temp_char_uuid.uuid, &sensor_svc.attrs[0],
                                    &last_temp_centi, sizeof(last_temp_centi));
            }
            if (notify_humidity_enabled)
            {
                bt_gatt_notify_uuid(NULL, &humidity_char_uuid.uuid, &sensor_svc.attrs[0],
                                    &last_humidity_centi, sizeof(last_humidity_centi));
            }
            if (notify_pressure_enabled)
            {
                bt_gatt_notify_uuid(NULL, &pressure_char_uuid.uuid, &sensor_svc.attrs[0],
                                    &last_pressure_pa, sizeof(last_pressure_pa));
            }
        }

        if (adc_is_ready_dt(&light_adc))
        {
            ret = adc_read_dt(&light_adc, &light_sequence);
            if (ret == 0)
            {
                last_light_raw = (uint16_t)light_sample_buffer;
                SENSOR_LOG_INF("Light(raw): %u", last_light_raw);
                if (notify_light_enabled)
                {
                    bt_gatt_notify_uuid(NULL, &light_char_uuid.uuid, &sensor_svc.attrs[0],
                                        &last_light_raw, sizeof(last_light_raw));
                }
            }
            else
            {
                LOG_ERR("Blad odczytu ADC: %d", ret);
            }
        }

        k_sleep(K_MSEC(1000));
    }
    return 0;
}