#ifndef DATA_H_
#define DATA_H_

#include <stdbool.h>
#include <stddef.h>

#define SENSOR_RINGBUFFER_CAPACITY 1024U

struct sensor_sample {
	double temperature;
	double humidity;
	double pressure;
	double light;
};

struct sensor_ringbuffer {
	struct sensor_sample samples[SENSOR_RINGBUFFER_CAPACITY];
	size_t head;
	size_t count;
};

extern struct sensor_ringbuffer sensor_data;

static inline void sensor_ringbuffer_reset(struct sensor_ringbuffer *rb)
{
	rb->head = 0U;
	rb->count = 0U;
}

static inline size_t sensor_ringbuffer_size(const struct sensor_ringbuffer *rb)
{
	return rb->count;
}

static inline bool sensor_ringbuffer_is_empty(const struct sensor_ringbuffer *rb)
{
	return rb->count == 0U;
}

static inline bool sensor_ringbuffer_is_full(const struct sensor_ringbuffer *rb)
{
	return rb->count == SENSOR_RINGBUFFER_CAPACITY;
}

/* Store the newest value and overwrite the oldest when full. */
static inline void sensor_ringbuffer_push(struct sensor_ringbuffer *rb,
					      struct sensor_sample sample)
{
	rb->samples[rb->head] = sample;
	rb->head = (rb->head + 1U) % SENSOR_RINGBUFFER_CAPACITY;

	if (rb->count < SENSOR_RINGBUFFER_CAPACITY) {
		rb->count++;
	}
}

static inline void sensor_ringbuffer_push_values(struct sensor_ringbuffer *rb,
						 double temperature,
						 double humidity,
						 double pressure,
						 double light)
{
	struct sensor_sample sample = {
		.temperature = temperature,
		.humidity = humidity,
		.pressure = pressure,
		.light = light,
	};

	sensor_ringbuffer_push(rb, sample);
}

/* age == 0 returns the newest sample, age == count-1 the oldest. */
static inline bool sensor_ringbuffer_get_latest(const struct sensor_ringbuffer *rb,
						       size_t age,
						       struct sensor_sample *value_out)
{
	size_t index;

	if (age >= rb->count || value_out == NULL) {
		return false;
	}

	index = (rb->head + SENSOR_RINGBUFFER_CAPACITY - 1U - age) %
		SENSOR_RINGBUFFER_CAPACITY;
	*value_out = rb->samples[index];

	return true;
}

#endif /* DATA_H_ */