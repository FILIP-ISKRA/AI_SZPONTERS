/*
 * Paired BLE device registry.
 */

#ifndef PAIRED_DEVICES_H_
#define PAIRED_DEVICES_H_

#include <stdbool.h>

#include <zephyr/bluetooth/addr.h>

bool paired_devices_add_mac_string(const char *mac);
bool paired_devices_is_paired_addr(const bt_addr_le_t *addr);

#endif /* PAIRED_DEVICES_H_ */
