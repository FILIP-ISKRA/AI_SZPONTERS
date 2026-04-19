#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/addr.h>

#include "paired_devices.h"

LOG_MODULE_REGISTER(paired_devices, LOG_LEVEL_INF);

#define MAX_PAIRED_DEVICES 8

static bt_addr_t paired_addrs[MAX_PAIRED_DEVICES];
static size_t paired_count;
static K_MUTEX_DEFINE(paired_devices_mutex);

static bool addr_exists_locked(const bt_addr_t *addr)
{
	for (size_t i = 0U; i < paired_count; i++) {
		if (bt_addr_cmp(&paired_addrs[i], addr) == 0) {
			return true;
		}
	}

	return false;
}

bool paired_devices_add_mac_string(const char *mac)
{
	bt_addr_t parsed;
	int ret;
	char addr_str[BT_ADDR_STR_LEN];

	if (mac == NULL) {
		return false;
	}

	ret = bt_addr_from_str(mac, &parsed);
	if (ret) {
		LOG_WRN("Invalid pair MAC string: %s", mac);
		return false;
	}

	k_mutex_lock(&paired_devices_mutex, K_FOREVER);

	if (addr_exists_locked(&parsed)) {
		k_mutex_unlock(&paired_devices_mutex);
		return true;
	}

	if (paired_count >= MAX_PAIRED_DEVICES) {
		k_mutex_unlock(&paired_devices_mutex);
		LOG_WRN("Paired device list full");
		return false;
	}

	paired_addrs[paired_count++] = parsed;
	k_mutex_unlock(&paired_devices_mutex);

	bt_addr_to_str(&parsed, addr_str, sizeof(addr_str));
	LOG_INF("Added paired BLE MAC %s", addr_str);
	return true;
}

bool paired_devices_is_paired_addr(const bt_addr_le_t *addr)
{
	bool found = false;

	if (addr == NULL) {
		return false;
	}

	k_mutex_lock(&paired_devices_mutex, K_FOREVER);
	for (size_t i = 0U; i < paired_count; i++) {
		if (bt_addr_cmp(&paired_addrs[i], &addr->a) == 0) {
			found = true;
			break;
		}
	}
	k_mutex_unlock(&paired_devices_mutex);

	return found;
}
