/*
 * Minimal BLE Central receiver for nRF54L15 sensor stream.
 */

#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_central_rx.h"
#include "data.h"
#include "paired_devices.h"

LOG_MODULE_REGISTER(ble_central_rx, LOG_LEVEL_INF);

enum sensor_uuid_idx {
	SENSOR_IDX_DISTANCE,
	SENSOR_IDX_LIGHT,
	SENSOR_IDX_TEMP,
	SENSOR_IDX_HUM,
	SENSOR_IDX_PRESS,
	SENSOR_COUNT
};

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static uint16_t value_handles[SENSOR_COUNT];
static struct k_work_delayable poll_work;
static struct bt_gatt_read_params read_params;
static int current_read_idx = -1;
static struct sensor_sample cycle_sample;
static uint8_t cycle_mask;

#define SENSOR_MASK_TEMP BIT(0)
#define SENSOR_MASK_HUM BIT(1)
#define SENSOR_MASK_PRESS BIT(2)
#define SENSOR_MASK_LIGHT BIT(3)
#define SENSOR_MASK_REQUIRED (SENSOR_MASK_TEMP | SENSOR_MASK_HUM | SENSOR_MASK_PRESS | SENSOR_MASK_LIGHT)

static struct bt_uuid_128 sensor_service_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678));
static struct bt_uuid_128 distance_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679));
static struct bt_uuid_128 light_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567A));
static struct bt_uuid_128 temp_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567B));
static struct bt_uuid_128 hum_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567C));
static struct bt_uuid_128 press_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56781234567D));

static const struct bt_uuid *sensor_uuids[SENSOR_COUNT] = {
	[SENSOR_IDX_DISTANCE] = &distance_uuid.uuid,
	[SENSOR_IDX_LIGHT] = &light_uuid.uuid,
	[SENSOR_IDX_TEMP] = &temp_uuid.uuid,
	[SENSOR_IDX_HUM] = &hum_uuid.uuid,
	[SENSOR_IDX_PRESS] = &press_uuid.uuid,
};

static const char *sensor_names[SENSOR_COUNT] = {
	[SENSOR_IDX_DISTANCE] = "distance",
	[SENSOR_IDX_LIGHT] = "light",
	[SENSOR_IDX_TEMP] = "temp",
	[SENSOR_IDX_HUM] = "humidity",
	[SENSOR_IDX_PRESS] = "pressure",
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad);
static void start_next_read_cycle(int start_idx);

static void log_sensor_value(int idx, const void *data, uint16_t length)
{
	switch (idx) {
	case SENSOR_IDX_DISTANCE:
		if (length >= 2) {
			uint16_t mm = sys_get_le16(data);

			LOG_INF("Distance: %u mm", mm);
		}
		break;
	case SENSOR_IDX_LIGHT:
		if (length >= 2) {
			uint16_t raw = sys_get_le16(data);

			LOG_INF("Light(raw): %u", raw);
		}
		break;
	case SENSOR_IDX_TEMP:
		if (length >= 2) {
			int16_t c_centi = (int16_t)sys_get_le16(data);
			int16_t abs_val = c_centi >= 0 ? c_centi : -c_centi;

			LOG_INF("Temp: %d.%02d C", c_centi / 100, abs_val % 100);
		}
		break;
	case SENSOR_IDX_HUM:
		if (length >= 2) {
			uint16_t h_centi = sys_get_le16(data);

			LOG_INF("Humidity: %u.%02u %%", h_centi / 100, h_centi % 100);
		}
		break;
	case SENSOR_IDX_PRESS:
		if (length >= 4) {
			uint32_t pa = sys_get_le32(data);

			LOG_INF("Pressure: %u.%02u hPa", pa / 100, pa % 100);
		}
		break;
	default:
		break;
	}
}

static void store_sensor_value(int idx, const void *data, uint16_t length)
{
	switch (idx) {
	case SENSOR_IDX_LIGHT:
		if (length >= 2) {
			uint16_t raw = sys_get_le16(data);

			cycle_sample.light = (double)raw;
			cycle_mask |= SENSOR_MASK_LIGHT;
		}
		break;
	case SENSOR_IDX_TEMP:
		if (length >= 2) {
			int16_t c_centi = (int16_t)sys_get_le16(data);

			cycle_sample.temperature = ((double)c_centi) / 100.0;
			cycle_mask |= SENSOR_MASK_TEMP;
		}
		break;
	case SENSOR_IDX_HUM:
		if (length >= 2) {
			uint16_t h_centi = sys_get_le16(data);

			cycle_sample.humidity = ((double)h_centi) / 100.0;
			cycle_mask |= SENSOR_MASK_HUM;
		}
		break;
	case SENSOR_IDX_PRESS:
		if (length >= 4) {
			uint32_t pa = sys_get_le32(data);

			cycle_sample.pressure = ((double)pa) / 100.0;
			cycle_mask |= SENSOR_MASK_PRESS;
		}
		break;
	default:
		break;
	}
}

static int sensor_index_from_uuid(const struct bt_uuid *uuid)
{
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (bt_uuid_cmp(uuid, sensor_uuids[i]) == 0) {
			return i;
		}
	}

	return -1;
}

static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
			      struct bt_gatt_read_params *params,
			      const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (err) {
		if (current_read_idx >= 0 && current_read_idx < SENSOR_COUNT) {
			LOG_ERR("Read failed for %s (err 0x%02x)",
				sensor_names[current_read_idx], err);
		} else {
			LOG_ERR("Read failed (err 0x%02x)", err);
		}

		start_next_read_cycle(current_read_idx + 1);
		return BT_GATT_ITER_STOP;
	}

	if (!data) {
		start_next_read_cycle(current_read_idx + 1);
		return BT_GATT_ITER_STOP;
	}

	if (current_read_idx >= 0 && current_read_idx < SENSOR_COUNT) {
		store_sensor_value(current_read_idx, data, length);
		log_sensor_value(current_read_idx, data, length);
	}

	start_next_read_cycle(current_read_idx + 1);
	return BT_GATT_ITER_STOP;
}

static void start_next_read_cycle(int start_idx)
{
	if (!default_conn) {
		return;
	}

	if (start_idx == 0) {
		struct sensor_sample newest;

		if (sensor_ringbuffer_get_latest(&sensor_data, 0U, &newest)) {
			cycle_sample = newest;
		} else {
			cycle_sample = (struct sensor_sample){ 0 };
		}

		cycle_mask = 0U;
	}

	for (int i = start_idx; i < SENSOR_COUNT; i++) {
		int err;

		if (!value_handles[i]) {
			continue;
		}

		current_read_idx = i;
		(void)memset(&read_params, 0, sizeof(read_params));
		read_params.func = read_cb;
		read_params.handle_count = 1;
		read_params.single.handle = value_handles[i];
		read_params.single.offset = 0;

		err = bt_gatt_read(default_conn, &read_params);
		if (err) {
			LOG_ERR("Read start failed for %s: %d", sensor_names[i], err);
			continue;
		}

		return;
	}

	if ((cycle_mask & SENSOR_MASK_REQUIRED) == SENSOR_MASK_REQUIRED) {
		sensor_ringbuffer_push(&sensor_data, cycle_sample);
		LOG_INF("Buffered sample T=%.2fC H=%.2f%% P=%.2fhPa L=%.0f",
			cycle_sample.temperature,
			cycle_sample.humidity,
			cycle_sample.pressure,
			cycle_sample.light);
	} else {
		LOG_WRN("Skipping buffer push, incomplete sensor set (mask 0x%02x)", cycle_mask);
	}

	k_work_reschedule(&poll_work, K_SECONDS(1));
}

static void poll_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!default_conn) {
		return;
	}

	start_next_read_cycle(0);
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	if (!attr) {
		LOG_INF("Discovery finished");

		if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
			k_work_reschedule(&poll_work, K_NO_WAIT);
		}

		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		const struct bt_gatt_service_val *svc = attr->user_data;

		discover_params.uuid = NULL;
		discover_params.start_handle = attr->handle + 1;
		discover_params.end_handle = svc->end_handle;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		if (bt_gatt_discover(conn, &discover_params) == 0) {
			return BT_GATT_ITER_STOP;
		}

		LOG_ERR("Characteristic discovery start failed");
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		const struct bt_gatt_chrc *chrc = attr->user_data;
		int idx = sensor_index_from_uuid(chrc->uuid);

		if (idx >= 0) {
			LOG_INF("Found char: %s (value_handle 0x%04x)",
				sensor_names[idx], chrc->value_handle);
			value_handles[idx] = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void start_scan(void)
{
	int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);

	if (err) {
		LOG_ERR("Scanning failed to start (%d)", err);
		return;
	}

	LOG_INF("Scanning for paired BLE devices...");
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	ARG_UNUSED(ad);

	if (default_conn) {
		return;
	}

	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
	    type != BT_GAP_ADV_TYPE_EXT_ADV) {
		return;
	}

	if (!paired_devices_is_paired_addr(addr)) {
		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
		LOG_DBG("Ignoring unpaired advertiser %s", addr_str);
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("Found paired advertiser %s, RSSI %d", addr_str, rssi);

	(void)bt_le_scan_stop();

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		LOG_ERR("Create conn failed (%d)", err);
		start_scan();
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	if (conn != default_conn) {
		return;
	}

	if (conn_err) {
		LOG_ERR("Connection failed (err 0x%02x)", conn_err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		start_scan();
		return;
	}

	LOG_INF("Connected, discovering service...");

	(void)memset(value_handles, 0, sizeof(value_handles));
	current_read_idx = -1;

	discover_params.uuid = &sensor_service_uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	if (bt_gatt_discover(conn, &discover_params)) {
		LOG_ERR("Service discovery start failed");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != default_conn) {
		return;
	}

	LOG_INF("Disconnected (reason 0x%02x)", reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	(void)memset(value_handles, 0, sizeof(value_handles));
	current_read_idx = -1;
	k_work_cancel_delayable(&poll_work);
	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int ble_central_rx_start(void)
{
	int err;

	k_work_init_delayable(&poll_work, poll_work_handler);

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (%d)", err);
		return err;
	}

	start_scan();

	return 0;
}