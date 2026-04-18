/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/net/net_config.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/conn_mgr_monitor.h>
#include <zephyr/net/conn_mgr_connectivity.h>

#include <zephyr/net/http/client.h>
#include <zephyr/net/http/parser.h>

#if defined(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#endif

#if defined(CONFIG_DK_LIBRARY)
#include <dk_buttons_and_leds.h>
#endif /* defined(CONFIG_DK_LIBRARY) */

#if defined(CONFIG_POSIX_API)
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/netdb.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/sys/socket.h>
#endif

#include "credentials_provision.h"
#include "data.h"
#include "predictor.h"

LOG_MODULE_REGISTER(http_server, CONFIG_HTTP_SERVER_SAMPLE_LOG_LEVEL);

#ifndef CONFIG_HTTP_SERVER_SAMPLE_PAIRED_MAX
#define CONFIG_HTTP_SERVER_SAMPLE_PAIRED_MAX 16
#endif

#ifndef CONFIG_HTTP_SERVER_SAMPLE_CLOUD_HOST
#define CONFIG_HTTP_SERVER_SAMPLE_CLOUD_HOST "example.com"
#endif

#ifndef CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PORT
#define CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PORT 80
#endif

#ifndef CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PATH
#define CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PATH "/api/temperature"
#endif

#define SERVER_PORT			CONFIG_HTTP_SERVER_SAMPLE_PORT
#define MAX_CLIENT_QUEUE		CONFIG_HTTP_SERVER_SAMPLE_CLIENTS_MAX
#define STACK_SIZE			CONFIG_HTTP_SERVER_SAMPLE_STACK_SIZE
#define THREAD_PRIORITY			K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES - 1)

/* Macro used to subscribe to specific Zephyr NET management events. */
#define L4_EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)
#define CONN_LAYER_EVENT_MASK (NET_EVENT_CONN_IF_FATAL_ERROR)

/* Macro called upon a fatal error, reboots the device. */
#define FATAL_ERROR()								\
	LOG_ERR("Fatal error!%s", IS_ENABLED(CONFIG_RESET_ON_FATAL_ERROR) ?	\
				  " Rebooting the device" : "");		\
	LOG_PANIC();								\
	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

#if defined(CONFIG_NET_HOSTNAME)
/* Register service */
DNS_SD_REGISTER_TCP_SERVICE(http_server_sd, CONFIG_NET_HOSTNAME, "_http", "local",
			    DNS_SD_EMPTY_TXT, SERVER_PORT);
#endif /* CONFIG_NET_HOSTNAME */

/* Zephyr NET management event callback structures. */
static struct net_mgmt_event_callback l4_cb;
static struct net_mgmt_event_callback conn_cb;

struct http_req {
	struct http_parser parser;
	int socket;
	bool received_all;
	enum http_method method;
	const char *url;
	size_t url_len;
	const char *body;
	size_t body_len;
};

/* Forward declarations */
static void process_tcp4(void);
static void process_tcp6(void);
static bool is_mac_paired(const char *mac);
static void push_temperature_sample(double temperature);
static int forward_temperature_to_cloud(const char *mac, double temperature, int8_t rssi);

/* Keep track of the current LED states. 0 = LED OFF, 1 = LED ON.
 * Index 0 corresponds to LED1, index 1 to LED2.
 */
static uint8_t led_states[2];
static char paired_runes[CONFIG_HTTP_SERVER_SAMPLE_PAIRED_MAX][18];
static size_t paired_runes_count;
static K_MUTEX_DEFINE(paired_runes_lock);
static char cloud_host[96] = CONFIG_HTTP_SERVER_SAMPLE_CLOUD_HOST;
static int cloud_port = CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PORT;
static char cloud_path[128] = CONFIG_HTTP_SERVER_SAMPLE_CLOUD_PATH;
static K_MUTEX_DEFINE(cloud_cfg_lock);

/* HTTP responses for demonstration */
#define RESPONSE_200 "HTTP/1.1 200 OK\r\n"
#define RESPONSE_400 "HTTP/1.1 400 Bad Request\r\n\r\n"
#define RESPONSE_403 "HTTP/1.1 403 Forbidden\r\n\r\n"
#define RESPONSE_404 "HTTP/1.1 404 Not Found\r\n\r\n"
#define RESPONSE_405 "HTTP/1.1 405 Method Not Allowed\r\n\r\n"
#define RESPONSE_500 "HTTP/1.1 500 Internal Server Error\r\n\r\n"
#define RESPONSE_LINE_200 "HTTP/1.1 200 OK\r\n"
#define RESPONSE_LINE_400 "HTTP/1.1 400 Bad Request\r\n"
#define SENSOR_JSON_MAX_POINTS 64U
#define SENSOR_BIN_MAGIC 0x31534E53U /* "SNS1" (little-endian) */
#define SENSOR_BIN_HEADER_SIZE 8U
#define SENSOR_BIN_SAMPLE_SIZE 16U
#define PRED_BIN_MAGIC 0x31445250U /* "PRD1" (little-endian) */
#define PRED_BIN_HEADER_SIZE 12U
#define PRED_BIN_SAMPLE_SIZE SENSOR_BIN_SAMPLE_SIZE
#define PREDICTION_STEPS_AHEAD 6U

static const unsigned char index_html[] = {
#if defined(HTTP_SERVER_INDEX_HTML_INC)
#include HTTP_SERVER_INDEX_HTML_INC

	/* Null terminate page */
	(0x00)
#else
""
#endif
};

#define INDEX_HTML_LEN (sizeof(index_html) - 1)

/* Processing threads for incoming connections */
K_THREAD_STACK_ARRAY_DEFINE(tcp4_handler_stack, MAX_CLIENT_QUEUE, STACK_SIZE);
static struct k_thread tcp4_handler_thread[MAX_CLIENT_QUEUE];
static k_tid_t tcp4_handler_tid[MAX_CLIENT_QUEUE];
K_THREAD_DEFINE(tcp4_thread_id, STACK_SIZE,
		process_tcp4, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

K_THREAD_STACK_ARRAY_DEFINE(tcp6_handler_stack, MAX_CLIENT_QUEUE, STACK_SIZE);
static struct k_thread tcp6_handler_thread[MAX_CLIENT_QUEUE];
static k_tid_t tcp6_handler_tid[MAX_CLIENT_QUEUE];
K_THREAD_DEFINE(tcp6_thread_id, STACK_SIZE,
		process_tcp6, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

static K_SEM_DEFINE(network_connected_sem, 0, 1);
static K_SEM_DEFINE(ipv6_setup_sem, 0, 1);

static int tcp4_sock;
static int tcp4_accepted[MAX_CLIENT_QUEUE];
static int tcp6_sock;
static int tcp6_accepted[MAX_CLIENT_QUEUE];

static struct http_parser_settings parser_settings;

#if defined(CONFIG_HTTP_SERVER_SAMPLE_BLE_SCAN) && defined(CONFIG_BT)
struct ble_scan_result {
	bool name_match;
	bool company_id_match;
	bool has_temperature;
	double temperature;
	char name[BT_GAP_ADV_MAX_ADV_DATA_LEN + 1];
};

static void bt_addr_to_mac_string(const bt_addr_le_t *addr, char output[18])
{
	snprintk(output, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
		 addr->a.val[5], addr->a.val[4], addr->a.val[3],
		 addr->a.val[2], addr->a.val[1], addr->a.val[0]);
}

static bool ble_name_matches(const char *name)
{
#if defined(CONFIG_HTTP_SERVER_SAMPLE_BLE_FILTER_BY_NAME)
	const char *name_filter = CONFIG_HTTP_SERVER_SAMPLE_BLE_NAME_FILTER;

	if (name_filter[0] == '\0' || name[0] == '\0') {
		return false;
	}

	return strstr(name, name_filter) != NULL;
#else
	ARG_UNUSED(name);
	return false;
#endif
}

static bool ble_ad_parse_cb(struct bt_data *data, void *user_data)
{
	struct ble_scan_result *result = user_data;

	switch (data->type) {
	case BT_DATA_NAME_COMPLETE:
	case BT_DATA_NAME_SHORTENED: {
		size_t len = MIN(data->data_len, sizeof(result->name) - 1);

		memcpy(result->name, data->data, len);
		result->name[len] = '\0';
		result->name_match = ble_name_matches(result->name);
		break;
	}

	case BT_DATA_MANUFACTURER_DATA:
		if (data->data_len >= 4U) {
			int16_t raw_temp = (int16_t)sys_get_le16(data->data + 2U);

			result->temperature = (double)raw_temp / 100.0;
			result->has_temperature = true;
		}
#if defined(CONFIG_HTTP_SERVER_SAMPLE_BLE_FILTER_BY_COMPANY_ID)
		if (data->data_len >= 2U) {
			uint16_t company_id = sys_get_le16(data->data);

			result->company_id_match =
				(company_id == (uint16_t)CONFIG_HTTP_SERVER_SAMPLE_BLE_COMPANY_ID);
		}
#endif
		break;

	default:
		break;
	}

	return true;
}

static void ble_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
			     struct net_buf_simple *ad)
{
	struct ble_scan_result result = {0};
	char mac[18];
	int ret;

	ARG_UNUSED(adv_type);

	bt_data_parse(ad, ble_ad_parse_cb, &result);
	bt_addr_to_mac_string(addr, mac);

	if (!is_mac_paired(mac)) {
		return;
	}

	if (!result.has_temperature) {
		LOG_WRN("Paired rune %s without temperature payload", mac);
		return;
	}

	push_temperature_sample(result.temperature);

	ret = forward_temperature_to_cloud(mac, result.temperature, rssi);
	if (ret) {
		LOG_ERR("Cloud forward failed for %s (%d)", mac, ret);
		return;
	}

	LOG_INF("Forwarded temperature %.2fC from %s (rssi=%d)", result.temperature, mac, rssi);
}

static int start_ble_scan(void)
{
	int ret;
	static const struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("Bluetooth init failed (%d)", ret);
		return ret;
	}

	ret = bt_le_scan_start(&scan_param, ble_device_found);
	if (ret) {
		LOG_ERR("Bluetooth scan start failed (%d)", ret);
		return ret;
	}

	LOG_INF("Bluetooth scan started (name filter: %s, company id: 0x%04X)",
		IS_ENABLED(CONFIG_HTTP_SERVER_SAMPLE_BLE_FILTER_BY_NAME) ?
		CONFIG_HTTP_SERVER_SAMPLE_BLE_NAME_FILTER : "disabled",
		IS_ENABLED(CONFIG_HTTP_SERVER_SAMPLE_BLE_FILTER_BY_COMPANY_ID) ?
		(uint16_t)CONFIG_HTTP_SERVER_SAMPLE_BLE_COMPANY_ID : 0U);

	return 0;
}
#endif /* defined(CONFIG_HTTP_SERVER_SAMPLE_BLE_SCAN) && defined(CONFIG_BT) */

static void l4_event_handler(struct net_mgmt_event_callback *cb,
			     uint64_t event,
			     struct net_if *iface)
{
	switch (event) {
	case NET_EVENT_L4_CONNECTED:
		LOG_INF("Network connected");

		k_sem_give(&network_connected_sem);
		break;
	case NET_EVENT_L4_DISCONNECTED:
		LOG_INF("Network disconnected");
		break;
	default:
		/* Don't care */
		return;
	}
}

static void connectivity_event_handler(struct net_mgmt_event_callback *cb,
				       uint64_t event,
				       struct net_if *iface)
{
	if (event == NET_EVENT_CONN_IF_FATAL_ERROR) {
		LOG_ERR("NET_EVENT_CONN_IF_FATAL_ERROR");
		FATAL_ERROR();
		return;
	}
}

/* Update the LED states. Returns 0 if it was updated, otherwise -1. */
static int led_update(uint8_t index, uint8_t state)
{
	if (state <= 1) {
		led_states[index] = state;
	} else {
		LOG_WRN("Illegal value %d written to LED %d", state, index);
		return -EBADMSG;
	}

#if defined(CONFIG_DK_LIBRARY)
	int ret;

	ret = dk_set_led(index, led_states[index]);
	if (ret) {
		LOG_ERR("Failed to update LED %d state to %d", index, led_states[index]);
		FATAL_ERROR();
		return -EIO;
	}
#endif /* defined(CONFIG_DK_LIBRARY) */

	LOG_INF("LED %d state updated to %d", index, led_states[index]);

	return 0;
}

static size_t url_path_len(const struct http_req *request)
{
	for (size_t i = 0U; i < request->url_len; i++) {
		if (request->url[i] == '?') {
			return i;
		}
	}

	return request->url_len;
}

static bool get_query_param(struct http_req *request,
				    const char *name,
				    const char **value,
				    size_t *value_len)
{
	size_t path_len = url_path_len(request);
	const char *query;
	size_t query_len;
	size_t name_len = strlen(name);

	if (path_len >= request->url_len) {
		return false;
	}

	query = request->url + path_len + 1U;
	query_len = request->url_len - path_len - 1U;

	for (size_t i = 0U; i < query_len;) {
		size_t key_start = i;
		size_t key_end = i;
		size_t val_start;
		size_t val_end;

		while (key_end < query_len && query[key_end] != '=' && query[key_end] != '&') {
			key_end++;
		}

		if (key_end >= query_len || query[key_end] != '=') {
			while (key_end < query_len && query[key_end] != '&') {
				key_end++;
			}
			i = (key_end < query_len) ? key_end + 1U : query_len;
			continue;
		}

		val_start = key_end + 1U;
		val_end = val_start;
		while (val_end < query_len && query[val_end] != '&') {
			val_end++;
		}

		if ((key_end - key_start) == name_len &&
		    memcmp(query + key_start, name, name_len) == 0) {
			*value = query + val_start;
			*value_len = val_end - val_start;
			return true;
		}

		i = (val_end < query_len) ? val_end + 1U : query_len;
	}

	return false;
}

static bool url_decode_component(const char *input, size_t input_len,
					 char *output, size_t output_size)
{
	size_t out = 0U;

	if (output_size == 0U) {
		return false;
	}

	for (size_t i = 0U; i < input_len; i++) {
		char ch = input[i];

		if (ch == '%' && (i + 2U) < input_len) {
			char h = input[i + 1U];
			char l = input[i + 2U];
			int hi;
			int lo;

			if (!isxdigit((unsigned char)h) || !isxdigit((unsigned char)l)) {
				return false;
			}

			hi = (h <= '9') ? (h - '0') : (10 + (toupper((unsigned char)h) - 'A'));
			lo = (l <= '9') ? (l - '0') : (10 + (toupper((unsigned char)l) - 'A'));

			if ((out + 1U) >= output_size) {
				return false;
			}

			output[out++] = (char)((hi << 4) | lo);
			i += 2U;
			continue;
		}

		if (ch == '+') {
			ch = ' ';
		}

		if ((out + 1U) >= output_size) {
			return false;
		}

		output[out++] = ch;
	}

	output[out] = '\0';
	return true;
}

static bool normalize_mac_string(const char *input, size_t input_len, char output[18])
{
	if (input_len != 17U) {
		return false;
	}

	for (size_t i = 0U; i < 17U; i++) {
		char ch = input[i];

		if ((i % 3U) == 2U) {
			if (ch != ':' && ch != '-') {
				return false;
			}
			output[i] = ':';
			continue;
		}

		if (!isxdigit((unsigned char)ch)) {
			return false;
		}

		output[i] = (char)toupper((unsigned char)ch);
	}

	output[17] = '\0';
	return true;
}

static bool is_mac_paired(const char *mac)
{
	bool found = false;

	k_mutex_lock(&paired_runes_lock, K_FOREVER);
	for (size_t i = 0U; i < paired_runes_count; i++) {
		if (strcmp(paired_runes[i], mac) == 0) {
			found = true;
			break;
		}
	}
	k_mutex_unlock(&paired_runes_lock);

	return found;
}

static int add_paired_mac(const char *mac)
{
	int ret = -ENOMEM;

	k_mutex_lock(&paired_runes_lock, K_FOREVER);
	for (size_t i = 0U; i < paired_runes_count; i++) {
		if (strcmp(paired_runes[i], mac) == 0) {
			ret = 0;
			goto out;
		}
	}

	if (paired_runes_count < ARRAY_SIZE(paired_runes)) {
		strncpy(paired_runes[paired_runes_count], mac, sizeof(paired_runes[0]) - 1U);
		paired_runes[paired_runes_count][sizeof(paired_runes[0]) - 1U] = '\0';
		paired_runes_count++;
		ret = 0;
	}

out:
	k_mutex_unlock(&paired_runes_lock);
	return ret;
}

static size_t get_paired_count(void)
{
	size_t count;

	k_mutex_lock(&paired_runes_lock, K_FOREVER);
	count = paired_runes_count;
	k_mutex_unlock(&paired_runes_lock);

	return count;
}

static bool url_matches(struct http_req *request, const char *path)
{
	size_t path_len = strlen(path);
	size_t req_path_len = url_path_len(request);

	if (req_path_len != path_len) {
		return false;
	}

	return (memcmp(request->url, path, path_len) == 0);
}

static int get_led_id_from_url(struct http_req *request, uint8_t *led_id)
{
	if (url_path_len(request) != 6U) {
		return -EINVAL;
	}

	if (memcmp(request->url, "/led/", 5) != 0) {
		return -EINVAL;
	}

	if (request->url[5] < '0' || request->url[5] > '9') {
		return -EINVAL;
	}

	*led_id = request->url[5] - '0';

	if (*led_id < 1 || *led_id > 2) {
		return -EINVAL;
	}

	return 0;
}

static int handle_put(struct http_req *request, char *response, size_t response_size)
{
	int ret;
	uint8_t led_id, led_index;

	if (request->body_len < 1) {
		LOG_ERR("No request body");
		return -EBADMSG;
	}

	ret = get_led_id_from_url(request, &led_id);
	if (ret) {
		LOG_WRN("Invalid URL for PUT");
		return -EINVAL;
	}

	led_index = led_id - 1;

	ret = led_update(led_index, (uint8_t)atoi(request->body));
	if (ret) {
		LOG_WRN("Update failed for LED %d", led_index);
		return ret;
	}

	ret = snprintk(response, response_size, "%sContent-Length: %d\r\n\r\n",
		       RESPONSE_200, 0);
	if ((ret < 0) || (ret >= response_size)) {
		return -ENOBUFS;
	}

	return 0;
}

static int handle_get(struct http_req *request, char *response, size_t response_size)
{
	int ret;
	char body[2];
	uint8_t led_id, led_index;

	ret = get_led_id_from_url(request, &led_id);
	if (ret) {
		LOG_WRN("Invalid URL for GET");
		return -EINVAL;
	}

	led_index = led_id - 1;

	ret = snprintk(body, sizeof(body), "%d", led_states[led_index]);
	if ((ret < 0) || (ret >= sizeof(body))) {
		return -ENOBUFS;
	}

	ret = snprintk(response, response_size,
		       "%sContent-Type: text/plain\r\nContent-Length: %d\r\n\r\n%s",
		       RESPONSE_200, strlen(body), body);
	if ((ret < 0) || (ret >= response_size)) {
		return -ENOBUFS;
	}

	return 0;
}

static int send_response_raw(struct http_req *request, const char *response, size_t len)
{
	ssize_t out_len;
	size_t sent = 0;
	int retry_count = 0;
	const int max_retries = 20;

	while (sent < len) {
		out_len = send(request->socket, response + sent, len - sent, 0);
		if (out_len < 0) {
			int err = errno;

			if ((err == ENOMEM || err == EAGAIN) && retry_count < max_retries) {
				retry_count++;
				k_msleep(5);
				continue;
			}

			LOG_ERR("send, error: %d", -err);
			return -err;
		}

		if (out_len == 0) {
			return -ECONNRESET;
		}

		retry_count = 0;
		sent += out_len;
	}

	return 0;
}

static int send_response(struct http_req *request, char *response)
{
	return send_response_raw(request, response, strlen(response));
}

static int send_json_response(struct http_req *request,
			      const char *status_line,
			      const char *json)
{
	int ret;
	char header[192];

	ret = snprintk(header, sizeof(header),
		       "%s"
		       "Content-Type: application/json; charset=utf-8\r\n"
		       "Cache-Control: no-store\r\n"
		       "Content-Length: %u\r\n\r\n",
		       status_line,
		       (unsigned int)strlen(json));
	if ((ret < 0) || (ret >= sizeof(header))) {
		return -ENOBUFS;
	}

	ret = send_response_raw(request, header, (size_t)ret);
	if (ret) {
		return ret;
	}

	return send_response_raw(request, json, strlen(json));
}

static void push_temperature_sample(double temperature)
{
	struct sensor_sample latest = {
		.temperature = temperature,
		.humidity = 0.0,
		.pressure = 0.0,
		.light = 0.0,
	};

	if (sensor_ringbuffer_get_latest(&sensor_data, 0U, &latest)) {
		latest.temperature = temperature;
	}

	sensor_ringbuffer_push(&sensor_data, latest);
}

static int socket_send_all(int socket_fd, const char *data, size_t data_len)
{
	size_t sent = 0U;

	while (sent < data_len) {
		ssize_t out_len = send(socket_fd, data + sent, data_len - sent, 0);

		if (out_len < 0) {
			return -errno;
		}

		if (out_len == 0) {
			return -ECONNRESET;
		}

		sent += (size_t)out_len;
	}

	return 0;
}

static int connect_cloud_socket(char *host_out, size_t host_out_len,
				int *port_out, char *path_out, size_t path_out_len)
{
	struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = IPPROTO_TCP,
	};
	struct addrinfo *res = NULL;
	struct addrinfo *rp;
	char service[8];
	int sock = -1;
	int ret;

	k_mutex_lock(&cloud_cfg_lock, K_FOREVER);
	strncpy(host_out, cloud_host, host_out_len - 1U);
	host_out[host_out_len - 1U] = '\0';
	strncpy(path_out, cloud_path, path_out_len - 1U);
	path_out[path_out_len - 1U] = '\0';
	*port_out = cloud_port;
	k_mutex_unlock(&cloud_cfg_lock);

	if ((*port_out <= 0) || (*port_out > 65535)) {
		return -EINVAL;
	}

	ret = snprintk(service, sizeof(service), "%d", *port_out);
	if ((ret < 0) || (ret >= sizeof(service))) {
		return -EINVAL;
	}

	ret = getaddrinfo(host_out, service, &hints, &res);
	if (ret != 0 || res == NULL) {
		LOG_ERR("Cloud DNS resolve failed for %s:%s (%d)", host_out, service, ret);
		return -EHOSTUNREACH;
	}

	for (rp = res; rp != NULL; rp = rp->ai_next) {
		sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (sock < 0) {
			continue;
		}

		if (connect(sock, rp->ai_addr, rp->ai_addrlen) == 0) {
			break;
		}

		close(sock);
		sock = -1;
	}

	freeaddrinfo(res);

	if (sock < 0) {
		LOG_ERR("Cloud connect failed %s:%d", host_out, *port_out);
		return -ECONNREFUSED;
	}

	return sock;
}

static int forward_temperature_to_cloud(const char *mac, double temperature, int8_t rssi)
{
	char host[96];
	char path[128];
	char body[192];
	char request[512];
	char response_buf[96];
	int port;
	int sock;
	int ret;

	sock = connect_cloud_socket(host, sizeof(host), &port, path, sizeof(path));
	if (sock < 0) {
		return sock;
	}

	ret = snprintk(body, sizeof(body),
		       "{\"mac\":\"%s\",\"temperature\":%.2f,\"rssi\":%d}",
		       mac, temperature, rssi);
	if ((ret < 0) || (ret >= sizeof(body))) {
		close(sock);
		return -ENOBUFS;
	}

	ret = snprintk(request, sizeof(request),
		       "POST %s HTTP/1.1\r\n"
		       "Host: %s\r\n"
		       "Content-Type: application/json\r\n"
		       "Connection: close\r\n"
		       "Content-Length: %d\r\n\r\n"
		       "%s",
		       path, host, strlen(body), body);
	if ((ret < 0) || (ret >= sizeof(request))) {
		close(sock);
		return -ENOBUFS;
	}

	ret = socket_send_all(sock, request, (size_t)ret);
	if (ret) {
		close(sock);
		return ret;
	}

	while (recv(sock, response_buf, sizeof(response_buf), 0) > 0) {
		/* Drain response. */
	}

	close(sock);
	return 0;
}

static int handle_pair_request(struct http_req *request)
{
	const char *mac_param;
	size_t mac_param_len;
	char decoded_mac[32];
	char normalized_mac[18];
	char response[160];
	int ret;

	if (!get_query_param(request, "mac", &mac_param, &mac_param_len)) {
		return send_json_response(request, RESPONSE_LINE_400,
			"{\"ok\":false,\"error\":\"missing_query_mac\"}");
	}

	if (!url_decode_component(mac_param, mac_param_len, decoded_mac, sizeof(decoded_mac)) ||
	    !normalize_mac_string(decoded_mac, strlen(decoded_mac), normalized_mac)) {
		return send_json_response(request, RESPONSE_LINE_400,
			"{\"ok\":false,\"error\":\"invalid_mac_format\"}");
	}

	ret = add_paired_mac(normalized_mac);
	if (ret) {
		return send_json_response(request, RESPONSE_LINE_400,
			"{\"ok\":false,\"error\":\"pairing_memory_full\"}");
	}

	ret = snprintk(response, sizeof(response),
		       "{\"ok\":true,\"paired\":\"%s\",\"paired_count\":%u}",
		       normalized_mac,
		       (unsigned int)get_paired_count());
	if ((ret < 0) || (ret >= sizeof(response))) {
		return -ENOBUFS;
	}

	LOG_INF("Paired rune: %s", normalized_mac);
	return send_json_response(request, RESPONSE_LINE_200, response);
}

static int handle_cloud_config_request(struct http_req *request)
{
	const char *host_param;
	const char *port_param;
	const char *path_param;
	size_t host_len;
	size_t port_len;
	size_t path_len;
	char host_buf[96];
	char port_buf[8];
	char path_buf[128];
	char response[256];
	bool host_present;
	bool port_present;
	bool path_present;

	host_present = get_query_param(request, "host", &host_param, &host_len);
	port_present = get_query_param(request, "port", &port_param, &port_len);
	path_present = get_query_param(request, "path", &path_param, &path_len);

	if (host_present) {
		if (!url_decode_component(host_param, host_len, host_buf, sizeof(host_buf)) ||
		    host_buf[0] == '\0') {
			return send_json_response(request, RESPONSE_LINE_400,
				"{\"ok\":false,\"error\":\"invalid_host\"}");
		}
	}

	if (path_present) {
		if (!url_decode_component(path_param, path_len, path_buf, sizeof(path_buf)) ||
		    path_buf[0] != '/') {
			return send_json_response(request, RESPONSE_LINE_400,
				"{\"ok\":false,\"error\":\"invalid_path\"}");
		}
	}

	if (port_present) {
		char *endptr;
		long parsed;

		if (!url_decode_component(port_param, port_len, port_buf, sizeof(port_buf))) {
			return send_json_response(request, RESPONSE_LINE_400,
				"{\"ok\":false,\"error\":\"invalid_port\"}");
		}

		parsed = strtol(port_buf, &endptr, 10);
		if (endptr == port_buf || *endptr != '\0' || parsed < 1L || parsed > 65535L) {
			return send_json_response(request, RESPONSE_LINE_400,
				"{\"ok\":false,\"error\":\"invalid_port\"}");
		}

		k_mutex_lock(&cloud_cfg_lock, K_FOREVER);
		cloud_port = (int)parsed;
		k_mutex_unlock(&cloud_cfg_lock);
	}

	if (host_present) {
		k_mutex_lock(&cloud_cfg_lock, K_FOREVER);
		strncpy(cloud_host, host_buf, sizeof(cloud_host) - 1U);
		cloud_host[sizeof(cloud_host) - 1U] = '\0';
		k_mutex_unlock(&cloud_cfg_lock);
	}

	if (path_present) {
		k_mutex_lock(&cloud_cfg_lock, K_FOREVER);
		strncpy(cloud_path, path_buf, sizeof(cloud_path) - 1U);
		cloud_path[sizeof(cloud_path) - 1U] = '\0';
		k_mutex_unlock(&cloud_cfg_lock);
	}

	k_mutex_lock(&cloud_cfg_lock, K_FOREVER);
	(void)snprintk(response, sizeof(response),
		      "{\"ok\":true,\"host\":\"%s\",\"port\":%d,\"path\":\"%s\"}",
		      cloud_host, cloud_port, cloud_path);
	k_mutex_unlock(&cloud_cfg_lock);

	return send_json_response(request, RESPONSE_LINE_200, response);
}

static int send_index_response(struct http_req *request)
{
	int ret;
	char header[96];

	ret = snprintk(header, sizeof(header),
		       "%sContent-Type: text/html; charset=utf-8\r\n"
		       "Content-Length: %u\r\n\r\n",
		       RESPONSE_200, (unsigned int)INDEX_HTML_LEN);
	if ((ret < 0) || (ret >= sizeof(header))) {
		return -ENOBUFS;
	}

	ret = send_response_raw(request, header, ret);
	if (ret) {
		return ret;
	}

	return send_response_raw(request, (const char *)index_html, INDEX_HTML_LEN);
}

static int send_sensor_data_response(struct http_req *request)
{
	int ret;
	char header[176];
	uint8_t meta[SENSOR_BIN_HEADER_SIZE];
	size_t points = sensor_ringbuffer_size(&sensor_data);
	unsigned int payload_len;

	if (points > SENSOR_JSON_MAX_POINTS) {
		points = SENSOR_JSON_MAX_POINTS;
	}

	payload_len = SENSOR_BIN_HEADER_SIZE + ((unsigned int)points * SENSOR_BIN_SAMPLE_SIZE);

	ret = snprintk(header, sizeof(header),
		       "%sContent-Type: application/octet-stream\r\n"
		       "Cache-Control: no-store\r\n"
		       "Content-Length: %u\r\n\r\n",
		       RESPONSE_200, payload_len);
	if ((ret < 0) || (ret >= sizeof(header))) {
		return -ENOBUFS;
	}

	ret = send_response_raw(request, header, ret);
	if (ret) {
		return ret;
	}

	meta[0] = (uint8_t)(SENSOR_BIN_MAGIC & 0xFFU);
	meta[1] = (uint8_t)((SENSOR_BIN_MAGIC >> 8) & 0xFFU);
	meta[2] = (uint8_t)((SENSOR_BIN_MAGIC >> 16) & 0xFFU);
	meta[3] = (uint8_t)((SENSOR_BIN_MAGIC >> 24) & 0xFFU);
	meta[4] = (uint8_t)(points & 0xFFU);
	meta[5] = (uint8_t)((points >> 8) & 0xFFU);
	meta[6] = (uint8_t)(SENSOR_BIN_SAMPLE_SIZE & 0xFFU);
	meta[7] = (uint8_t)((SENSOR_BIN_SAMPLE_SIZE >> 8) & 0xFFU);

	ret = send_response_raw(request, (const char *)meta, sizeof(meta));
	if (ret) {
		return ret;
	}

	for (size_t i = points; i > 0; i--) {
		struct sensor_sample sample;
		float packed[4];

		if (!sensor_ringbuffer_get_latest(&sensor_data, i - 1U, &sample)) {
			return -ENOENT;
		}

		packed[0] = (float)sample.temperature;
		packed[1] = (float)sample.humidity;
		packed[2] = (float)sample.pressure;
		packed[3] = (float)sample.light;

		ret = send_response_raw(request, (const char *)packed, sizeof(packed));
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int send_prediction_response(struct http_req *request)
{
	int ret;
	char header[176];
	uint8_t meta[PRED_BIN_HEADER_SIZE];
	float packed[4];
	struct sensor_sample history_window[WEATHER_MODEL_HISTORY_STEPS];
	struct sensor_sample oldest_available;
	struct sensor_sample predicted;
	size_t history_points = sensor_ringbuffer_size(&sensor_data);
	size_t usable_history;
	unsigned int payload_len;

	if (history_points > SENSOR_JSON_MAX_POINTS) {
		history_points = SENSOR_JSON_MAX_POINTS;
	}

	if (history_points == 0U) {
		return -ENOENT;
	}

	usable_history = history_points;
	if (usable_history > WEATHER_MODEL_HISTORY_STEPS) {
		usable_history = WEATHER_MODEL_HISTORY_STEPS;
	}

	if (!sensor_ringbuffer_get_latest(&sensor_data, usable_history - 1U,
					  &oldest_available)) {
		return -ENOENT;
	}

	for (size_t i = 0U; i < WEATHER_MODEL_HISTORY_STEPS; i++) {
		history_window[i] = oldest_available;
	}

	for (size_t i = 0U; i < usable_history; i++) {
		size_t age = (usable_history - 1U) - i;
		size_t dst = (WEATHER_MODEL_HISTORY_STEPS - usable_history) + i;

		if (!sensor_ringbuffer_get_latest(&sensor_data, age,
						  &history_window[dst])) {
			return -ENOENT;
		}
	}

	payload_len = PRED_BIN_HEADER_SIZE +
		((unsigned int)(history_points + PREDICTION_STEPS_AHEAD) * PRED_BIN_SAMPLE_SIZE);

	ret = snprintk(header, sizeof(header),
		       "%sContent-Type: application/octet-stream\r\n"
		       "Cache-Control: no-store\r\n"
		       "Content-Length: %u\r\n\r\n",
		       RESPONSE_200, payload_len);
	if ((ret < 0) || (ret >= sizeof(header))) {
		return -ENOBUFS;
	}

	ret = send_response_raw(request, header, ret);
	if (ret) {
		return ret;
	}

	meta[0] = (uint8_t)(PRED_BIN_MAGIC & 0xFFU);
	meta[1] = (uint8_t)((PRED_BIN_MAGIC >> 8) & 0xFFU);
	meta[2] = (uint8_t)((PRED_BIN_MAGIC >> 16) & 0xFFU);
	meta[3] = (uint8_t)((PRED_BIN_MAGIC >> 24) & 0xFFU);
	meta[4] = (uint8_t)(history_points & 0xFFU);
	meta[5] = (uint8_t)((history_points >> 8) & 0xFFU);
	meta[6] = (uint8_t)(PREDICTION_STEPS_AHEAD & 0xFFU);
	meta[7] = (uint8_t)((PREDICTION_STEPS_AHEAD >> 8) & 0xFFU);
	meta[8] = (uint8_t)(PRED_BIN_SAMPLE_SIZE & 0xFFU);
	meta[9] = (uint8_t)((PRED_BIN_SAMPLE_SIZE >> 8) & 0xFFU);
	meta[10] = 0U;
	meta[11] = 0U;

	ret = send_response_raw(request, (const char *)meta, sizeof(meta));
	if (ret) {
		return ret;
	}

	for (size_t i = history_points; i > 0; i--) {
		struct sensor_sample sample;

		if (!sensor_ringbuffer_get_latest(&sensor_data, i - 1U, &sample)) {
			return -ENOENT;
		}

		packed[0] = (float)sample.temperature;
		packed[1] = (float)sample.humidity;
		packed[2] = (float)sample.pressure;
		packed[3] = (float)sample.light;

		ret = send_response_raw(request, (const char *)packed, sizeof(packed));
		if (ret) {
			return ret;
		}
	}

	for (size_t i = 0; i < PREDICTION_STEPS_AHEAD; i++) {
		if (!weather_predict_next_sample_from_history(
				history_window,
				WEATHER_MODEL_HISTORY_STEPS,
				&predicted)) {
			return -EINVAL;
		}

		packed[0] = (float)predicted.temperature;
		packed[1] = (float)predicted.humidity;
		packed[2] = (float)predicted.pressure;
		packed[3] = (float)predicted.light;

		ret = send_response_raw(request, (const char *)packed, sizeof(packed));
		if (ret) {
			return ret;
		}

		for (size_t j = 1U; j < WEATHER_MODEL_HISTORY_STEPS; j++) {
			history_window[j - 1U] = history_window[j];
		}
		history_window[WEATHER_MODEL_HISTORY_STEPS - 1U] = predicted;
	}

	return 0;
}

/* Handle HTTP request */
static void handle_http_request(struct http_req *request)
{
	int ret;
	char *resp_ptr = RESPONSE_200;
	char get_response_buffer[128] = { 0 };

	if (request->method == HTTP_GET &&
	    (url_matches(request, "/") || url_matches(request, "/index.html"))) {
		ret = send_index_response(request);
		if (ret) {
			LOG_ERR("send_index_response, error: %d", ret);
			FATAL_ERROR();
		}

		return;
	}

	if (request->method == HTTP_GET && url_matches(request, "/api/pair")) {
		ret = handle_pair_request(request);
		if (ret) {
			LOG_ERR("handle_pair_request, error: %d", ret);
		}

		return;
	}

	if (request->method == HTTP_GET && url_matches(request, "/api/cloud")) {
		ret = handle_cloud_config_request(request);
		if (ret) {
			LOG_ERR("handle_cloud_config_request, error: %d", ret);
		}

		return;
	}

	if (request->method == HTTP_GET &&
	    (url_matches(request, "/api/sensors") || url_matches(request, "/api/data") ||
	     url_matches(request, "/api/sensors.bin"))) {
		ret = send_sensor_data_response(request);
		if (ret) {
			LOG_ERR("send_sensor_data_response, error: %d", ret);
		}

		return;
	}

	if (request->method == HTTP_GET &&
	    (url_matches(request, "/api/predict.bin") ||
	     url_matches(request, "/api/predict") ||
	     url_matches(request, "/api/predict/next"))) {
		ret = send_prediction_response(request);
		if (ret) {
			LOG_ERR("send_prediction_response, error: %d", ret);
		}

		return;
	}

	/* Handle the request method */
	switch (request->method) {
	case HTTP_PUT:
		ret = handle_put(request, get_response_buffer, sizeof(get_response_buffer));
		if (ret) {
			LOG_WRN("Incoming HTTP GET request, error: %d", ret);
			resp_ptr = (ret == -EINVAL) ? RESPONSE_404 :
				   (ret == -EBADMSG) ? RESPONSE_400 : RESPONSE_500;
			break;
		}

		resp_ptr = get_response_buffer;
		break;
	case HTTP_GET:
		ret = handle_get(request, get_response_buffer, sizeof(get_response_buffer));
		if (ret) {
			LOG_WRN("Incoming HTTP GET request, error: %d", ret);
			resp_ptr = (ret == -EINVAL) ? RESPONSE_404 : RESPONSE_500;
			break;
		}

		resp_ptr = get_response_buffer;
		break;
	default:
		LOG_WRN("Unsupported request method");
		resp_ptr = RESPONSE_405;
		break;
	}

	/* Send response */
	ret = send_response(request, resp_ptr);
	if (ret) {
		LOG_ERR("send_response, error: %d", ret);
		return;
	}
}

/* HTTP parser callbacks */
static int on_body(struct http_parser *parser, const char *at, size_t length)
{
	struct http_req *req = CONTAINER_OF(parser, struct http_req, parser);

	req->body = at;
	req->body_len = length;

	LOG_DBG("on_body: %d", parser->method);
	LOG_DBG("> %.*s", length, at);

	return 0;
}

static int on_headers_complete(struct http_parser *parser)
{
	struct http_req *req = CONTAINER_OF(parser, struct http_req, parser);

	req->method = parser->method;

	LOG_DBG("on_headers_complete, method: %s", http_method_str(parser->method));

	return 0;
}

static int on_message_begin(struct http_parser *parser)
{
	struct http_req *req = CONTAINER_OF(parser, struct http_req, parser);

	req->received_all = false;

	LOG_DBG("on_message_begin, method: %d", parser->method);

	return 0;
}

static int on_message_complete(struct http_parser *parser)
{
	struct http_req *req = CONTAINER_OF(parser, struct http_req, parser);

	req->received_all = true;

	LOG_DBG("on_message_complete, method: %d", parser->method);

	return 0;
}

static int on_url(struct http_parser *parser, const char *at, size_t length)
{
	struct http_req *req = CONTAINER_OF(parser, struct http_req, parser);

	req->url = at;
	req->url_len = length;

	LOG_DBG("on_url, method: %d", parser->method);
	LOG_DBG("> %.*s", length, at);

	return 0;
}

/* Initialize HTTP parser. */
static void parser_init(void)
{
	http_parser_settings_init(&parser_settings);

	parser_settings.on_body = on_body;
	parser_settings.on_headers_complete = on_headers_complete;
	parser_settings.on_message_begin = on_message_begin;
	parser_settings.on_message_complete = on_message_complete;
	parser_settings.on_url = on_url;
}

static int setup_server(int *sock, struct sockaddr *bind_addr, socklen_t bind_addrlen)
{
	int ret;
	int enable = 1;

	if (IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS)) {
		/* Run the Zephyr Native TLS stack (MBed TLS) in the application core instead of
		 * using the TLS stack on the modem.
		 * This is needed because the modem does not support TLS in server mode.
		 *
		 * This is done by using (SOCK_STREAM | SOCK_NATIVE_TLS) as socket type when
		 * building for a nRF91 Series board.
		 */
		int type = IS_ENABLED(CONFIG_NRF_MODEM_LIB) ? SOCK_STREAM | SOCK_NATIVE_TLS :
							      SOCK_STREAM;

		*sock = socket(bind_addr->sa_family, type, IPPROTO_TLS_1_2);
	} else {
		*sock = socket(bind_addr->sa_family, SOCK_STREAM, IPPROTO_TCP);
	}

	if (*sock < 0) {
		LOG_ERR("Failed to create socket: %d", -errno);
		return -errno;
	}

	if (IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS)) {
		sec_tag_t sec_tag_list[] = {
			CONFIG_HTTP_SERVER_SAMPLE_SERVER_CERTIFICATE_SEC_TAG,
		};

		ret = setsockopt(*sock, SOL_TLS, TLS_SEC_TAG_LIST,
				 sec_tag_list, sizeof(sec_tag_list));
		if (ret < 0) {
			LOG_ERR("Failed to set security tag list %d", -errno);
			return -errno;
		}

		if (IS_ENABLED(CONFIG_HTTP_SERVER_SAMPLE_PEER_VERIFICATION_REQUIRE)) {
			int require = 2;

			ret = zsock_setsockopt(*sock, SOL_TLS, TLS_PEER_VERIFY,
					       &require,
					       sizeof(require));
			if (ret < 0) {
				LOG_ERR("Failed to set peer verification option %d", -errno);
				return -errno;
			}
		}
	}

	ret = setsockopt(*sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
	if (ret) {
		LOG_ERR("Failed to set SO_REUSEADDR %d", -errno);
		return -errno;
	}

	ret = bind(*sock, bind_addr, bind_addrlen);
	if (ret < 0) {
		LOG_ERR("Failed to bind socket %d", -errno);
		return -errno;
	}

	ret = listen(*sock, MAX_CLIENT_QUEUE);
	if (ret < 0) {
		LOG_ERR("Failed to listen on socket %d", -errno);
		return -errno;
	}

	return ret;
}

static void client_conn_handler(void *ptr1, void *ptr2, void *ptr3)
{
	ARG_UNUSED(ptr1);
	int *sock = ptr2;
	k_tid_t *in_use = ptr3;
	int received;
	int ret;
	char buf[CONFIG_HTTP_SERVER_SAMPLE_RECEIVE_BUFFER_SIZE];
	size_t offset = 0;
	size_t total_received = 0;
	struct http_req request = {
		.socket = *sock,
	};

	http_parser_init(&request.parser, HTTP_REQUEST);

	while (true) {
		received = recv(request.socket, buf + offset, sizeof(buf) - offset, 0);
		if (received == 0) {
			/* Connection closed */
			LOG_DBG("[%d] Connection closed by peer", request.socket);
			break;
		} else if (received < 0) {
			/* Socket error */
			ret = -errno;
			LOG_ERR("[%d] Connection error %d", request.socket, ret);
			break;
		}

		/* Parse the received data as HTTP request */
		(void)http_parser_execute(&request.parser,
					  &parser_settings, buf + offset, received);

		total_received += received;
		offset += received;

		if (offset >= sizeof(buf)) {
			offset = 0;
		}

		/* If the HTTP request has been completely received, stop receiving data and
		 * proceed to process the request.
		 */
		if (request.received_all) {
			handle_http_request(&request);
			break;
		}
	};

	(void)close(request.socket);

	*sock = -1;
	*in_use = NULL;
}

static int get_free_slot(int *accepted)
{
	for (int i = 0; i < MAX_CLIENT_QUEUE; i++) {
		if (accepted[i] < 0) {
			return i;
		}
	}

	return -1;
}

static void process_tcp(sa_family_t family, int *sock, int *accepted)
{
	int client;
	int slot;
	struct sockaddr_in6 client_addr;
	socklen_t client_addr_len = sizeof(client_addr);
	char addr_str[INET6_ADDRSTRLEN];

	client = accept(*sock, (struct sockaddr *)&client_addr,
			&client_addr_len);
	if (client < 0) {
		LOG_ERR("Error in accept %d, try again", -errno);
		return;
	}

	slot = get_free_slot(accepted);
	if (slot < 0 || slot >= MAX_CLIENT_QUEUE) {
		LOG_ERR("Cannot accept more connections");
		close(client);
		return;
	}

	accepted[slot] = client;

	if (family == AF_INET6) {
		tcp6_handler_tid[slot] = k_thread_create(
			&tcp6_handler_thread[slot],
			tcp6_handler_stack[slot],
			K_THREAD_STACK_SIZEOF(tcp6_handler_stack[slot]),
			(k_thread_entry_t)client_conn_handler,
			INT_TO_POINTER(slot),
			&accepted[slot],
			&tcp6_handler_tid[slot],
			THREAD_PRIORITY,
			0, K_NO_WAIT);
	} else if (family == AF_INET) {
		tcp4_handler_tid[slot] = k_thread_create(
			&tcp4_handler_thread[slot],
			tcp4_handler_stack[slot],
			K_THREAD_STACK_SIZEOF(tcp4_handler_stack[slot]),
			(k_thread_entry_t)client_conn_handler,
			INT_TO_POINTER(slot),
			&accepted[slot],
			&tcp4_handler_tid[slot],
			THREAD_PRIORITY,
			0, K_NO_WAIT);
	}

	net_addr_ntop(client_addr.sin6_family, &client_addr.sin6_addr, addr_str, sizeof(addr_str));

	LOG_INF("[%d] Connection from %s accepted", client, addr_str);
}

/* Processing incoming IPv4 clients */
static void process_tcp4(void)
{
	int ret;
	struct sockaddr_in addr4 = {
		.sin_family = AF_INET,
		.sin_port = htons(SERVER_PORT),
	};

	ret = setup_server(&tcp4_sock, (struct sockaddr *)&addr4, sizeof(addr4));
	if (ret < 0) {
		LOG_ERR("Failed to create IPv4 socket %d", ret);
		return;
	}

	LOG_INF("Waiting for IPv4 HTTP connections on port %d, sock %d", SERVER_PORT, tcp4_sock);

	while (true) {
		process_tcp(AF_INET, &tcp4_sock, tcp4_accepted);
	}
}

/* Processing incoming IPv6 clients */
static void process_tcp6(void)
{
	int ret;
	struct sockaddr_in6 addr6 = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(SERVER_PORT),
	};

	ret = setup_server(&tcp6_sock, (struct sockaddr *)&addr6, sizeof(addr6));

	k_sem_give(&ipv6_setup_sem);

	if (ret < 0) {
		LOG_ERR("Failed to create IPv6 socket %d", ret);
		return;
	}

	LOG_INF("Waiting for IPv6 HTTP connections on port %d, sock %d",
		SERVER_PORT, tcp6_sock);

	while (true) {
		process_tcp(AF_INET6, &tcp6_sock, tcp6_accepted);
	}
}

void start_listener(void)
{
	for (size_t i = 0; i < MAX_CLIENT_QUEUE; i++) {
		if (IS_ENABLED(CONFIG_NET_IPV4)) {
			tcp4_accepted[i] = -1;
			tcp4_sock = -1;
		}

		if (IS_ENABLED(CONFIG_NET_IPV6)) {
			tcp6_accepted[i] = -1;
			tcp6_sock = -1;
		}
	}

	if (IS_ENABLED(CONFIG_NET_IPV6)) {
		k_thread_start(tcp6_thread_id);

		/* Wait for the thread that sets up the IPv6 sockets to complete
		 * before starting the IPv4 thread.
		 * This is to avoid an error when secure sockets
		 * for both IP families are created at the same time in two different threads.
		 */
		k_sem_take(&ipv6_setup_sem, K_FOREVER);
	}

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		k_thread_start(tcp4_thread_id);
	}
}

int main(void)
{
	int ret;

	LOG_INF("HTTP Server sample started");

	if (IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS)) {
		ret = credentials_provision();
		if (ret) {
			LOG_ERR("credentials_provision, error: %d", ret);
			FATAL_ERROR();
			return ret;
		}
	}

	parser_init();

#if defined(CONFIG_DK_LIBRARY)
	ret = dk_leds_init();
	if (ret) {
		LOG_ERR("dk_leds_init, error: %d", ret);
		FATAL_ERROR();
		return ret;
	}
#endif /* defined(CONFIG_DK_LIBRARY) */

#if defined(CONFIG_HTTP_SERVER_SAMPLE_BLE_SCAN) && defined(CONFIG_BT)
	ret = start_ble_scan();
	if (ret) {
		FATAL_ERROR();
		return ret;
	}
#endif

	/* Setup handler for Zephyr NET Connection Manager events. */
	net_mgmt_init_event_callback(&l4_cb, l4_event_handler, L4_EVENT_MASK);
	net_mgmt_add_event_callback(&l4_cb);

	/* Setup handler for Zephyr NET Connection Manager Connectivity layer. */
	net_mgmt_init_event_callback(&conn_cb, connectivity_event_handler, CONN_LAYER_EVENT_MASK);
	net_mgmt_add_event_callback(&conn_cb);

	ret = conn_mgr_all_if_up(true);
	if (ret) {
		LOG_ERR("conn_mgr_all_if_up, error: %d", ret);
		FATAL_ERROR();
		return ret;
	}

	LOG_INF("Network interface brought up");

	ret = conn_mgr_all_if_connect(true);
	if (ret) {
		LOG_ERR("conn_mgr_all_if_connect, error: %d", ret);
		FATAL_ERROR();
		return ret;
	}

	/* Resend connection status if the sample is built for NATIVE_SIM.
	 * This is necessary because the network interface is automatically brought up
	 * at SYS_INIT() before main() is called.
	 * This means that NET_EVENT_L4_CONNECTED fires before the
	 * appropriate handler l4_event_handler() is registered.
	 */
	if (IS_ENABLED(CONFIG_BOARD_NATIVE_SIM)) {
		conn_mgr_mon_resend_status();
	}

	k_sem_take(&network_connected_sem, K_FOREVER);

	start_listener();

	return 0;
}
