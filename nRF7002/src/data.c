#include "data.h"

struct sensor_ringbuffer sensor_data = {
	.head = 0U,
	.count = 0U,
};
