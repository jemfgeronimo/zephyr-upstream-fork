/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad7866

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER 1
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_ad7866, CONFIG_ADC_LOG_LEVEL);

#define AD7866_RESOLUTION   12
#define AD7866_NUM_CHANNELS 4
#define AD7866_NUM_ADCS     2
#define AD7866_NUM_PAIRS    2

/* Both ADC results are read on one data line: 2 x 16 SCLK cycles. */
#define AD7866_FRAME_BYTES 4

/* Each 16-bit word is: 1 leading zero, 3 STATUS bits, 12 data bits. */
#define AD7866_ZERO_MASK         BIT(15)
#define AD7866_STATUS_RANGE_MASK BIT(14)
#define AD7866_STATUS_A0_MASK    BIT(13)
#define AD7866_STATUS_AB_MASK    BIT(12)
#define AD7866_DATA_MASK         GENMASK(11, 0)

#define AD7866_SPI_MIN_HZ 10000
#define AD7866_SPI_MAX_HZ 20000000

/* Internal reference voltage, in mV. */
#define AD7866_VREF_INTERNAL_MV 2500

/* Power-up time from full power-down is about 4 ms. */
#define AD7866_POWER_UP_MS 5

/*
 * Channels are numbered VA1, VA2, VB1, VB2. ADC A has channels 0 and 1, ADC B
 * has channels 2 and 3, and the A0 pin selects the channel of each ADC that is
 * converted: bit 0 of the channel number.
 */
#define AD7866_CHANNEL_PAIR(ch) ((ch) & BIT(0))
#define AD7866_PAIR_MASK(pair)  (BIT(pair) | BIT((pair) + 2))

struct ad7866_config {
	struct spi_dt_spec bus;
	/* port is NULL when the pin is hardwired low */
	struct gpio_dt_spec gpio_a0;
	struct gpio_dt_spec gpio_range;
	uint16_t vref_mv;
	uint16_t avdd_mv;
};

struct ad7866_data {
	struct adc_context ctx;
	struct k_sem acquire_signal;
	uint16_t *buffer;
	uint16_t *buffer_ptr;
	/* Channels of the running sequence */
	uint8_t channels;
	/* Channels set up with the VREF +/- VREF range (gain 1/2) */
	uint8_t range_2x_mask;
	/*
	 * The A0 and RANGE pins are sampled on the CS falling edge and apply to
	 * the next conversion: pair and range are what the next frame converts,
	 * pair_sel and range_sel are the levels driven on the pins.
	 */
	uint8_t pair;
	uint8_t range;
	uint8_t pair_sel;
	uint8_t range_sel;
#if CONFIG_ADC_ASYNC
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_AD7866_ACQUISITION_THREAD_STACK_SIZE);
#endif /* CONFIG_ADC_ASYNC */
};

static int ad7866_transfer(const struct device *dev, uint16_t *words)
{
	const struct ad7866_config *config = dev->config;
	uint8_t buffer_rx[AD7866_FRAME_BYTES];
	const struct spi_buf rx_buf = {
		.buf = buffer_rx,
		.len = sizeof(buffer_rx),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};
	int ret;

	ret = spi_read_dt(&config->bus, &rx);
	if (ret != 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return ret;
	}

	words[0] = sys_get_be16(&buffer_rx[0]);
	words[1] = sys_get_be16(&buffer_rx[2]);

	return 0;
}

/*
 * The STATUS bits tell which ADC a word belongs to, so the result does not
 * depend on whether DOUTA or DOUTB is wired to the SPI MISO line. The A0 and
 * RANGE status bits are checked against what the pins were set to before the
 * previous conversion, which flags a wrong wiring or a missing device.
 *
 * Results of the VREF +/- VREF range are twos complement codes, they are
 * offset by half the full scale so that all samples are unsigned codes.
 */
static int ad7866_decode(const struct device *dev, const uint16_t *words, uint16_t *val_a,
			 uint16_t *val_b)
{
	struct ad7866_data *data = dev->data;
	uint16_t val[AD7866_NUM_ADCS];
	bool found[AD7866_NUM_ADCS] = {false, false};

	for (int i = 0; i < AD7866_NUM_ADCS; i++) {
		uint8_t adc;

		if (words[i] & AD7866_ZERO_MASK) {
			return -EIO;
		}

		adc = FIELD_GET(AD7866_STATUS_AB_MASK, words[i]);
		if (found[adc]) {
			return -EIO;
		}

		found[adc] = true;
		val[adc] = FIELD_GET(AD7866_DATA_MASK, words[i]);
		if (FIELD_GET(AD7866_STATUS_RANGE_MASK, words[i]) != 0) {
			val[adc] ^= BIT(AD7866_RESOLUTION - 1);
		}
	}

	if (FIELD_GET(AD7866_STATUS_A0_MASK, words[0]) != data->pair) {
		return -EIO;
	}

	if (FIELD_GET(AD7866_STATUS_RANGE_MASK, words[0]) != data->range) {
		return -EIO;
	}

	if (val_a != NULL) {
		*val_a = val[0];
	}

	if (val_b != NULL) {
		*val_b = val[1];
	}

	return 0;
}

/* Run one conversion frame with the pins already set. */
static int ad7866_frame(const struct device *dev, uint16_t *val_a, uint16_t *val_b)
{
	struct ad7866_data *data = dev->data;
	uint16_t words[AD7866_NUM_ADCS];
	int ret;

	ret = ad7866_transfer(dev, words);
	if (ret != 0) {
		return ret;
	}

	ret = ad7866_decode(dev, words, val_a, val_b);

	/* The pins were sampled on the CS falling edge of this frame. */
	data->pair = data->pair_sel;
	data->range = data->range_sel;

	return ret;
}

static int ad7866_set_pins(const struct device *dev, uint8_t pair, uint8_t range)
{
	const struct ad7866_config *config = dev->config;
	struct ad7866_data *data = dev->data;
	int ret;

	if (pair != data->pair_sel) {
		if (config->gpio_a0.port == NULL) {
			return -ENOTSUP;
		}

		ret = gpio_pin_set_dt(&config->gpio_a0, pair);
		if (ret != 0) {
			return ret;
		}

		data->pair_sel = pair;
	}

	if (range != data->range_sel) {
		if (config->gpio_range.port == NULL) {
			return -ENOTSUP;
		}

		ret = gpio_pin_set_dt(&config->gpio_range, range);
		if (ret != 0) {
			return ret;
		}

		data->range_sel = range;
	}

	return 0;
}

/*
 * Read one simultaneous conversion of ADC A and ADC B. The conversion is done
 * on the pair selected by the previous frame, and next_pair is selected for
 * the following one.
 */
static int ad7866_convert(const struct device *dev, uint8_t next_pair, uint16_t *val_a,
			  uint16_t *val_b)
{
	struct ad7866_data *data = dev->data;
	int ret;

	ret = ad7866_set_pins(dev, next_pair, data->range_sel);
	if (ret != 0) {
		return ret;
	}

	return ad7866_frame(dev, val_a, val_b);
}

/* Apply a new pair or range, with a conversion that is discarded. */
static int ad7866_select(const struct device *dev, uint8_t pair, uint8_t range)
{
	struct ad7866_data *data = dev->data;
	int ret;

	if (pair == data->pair && pair == data->pair_sel && range == data->range &&
	    range == data->range_sel) {
		return 0;
	}

	ret = ad7866_set_pins(dev, pair, range);
	if (ret != 0) {
		return ret;
	}

	return ad7866_frame(dev, NULL, NULL);
}

/*
 * Read one scan of the sequence channels, in ascending channel order. Both
 * pairs are only converted when they hold a selected channel, and each
 * conversion selects the pair of the next one so no conversion is wasted.
 */
static int ad7866_read_scan(const struct device *dev, uint16_t *samples)
{
	struct ad7866_data *data = dev->data;
	uint8_t pairs[AD7866_NUM_PAIRS];
	uint16_t vals[AD7866_NUM_CHANNELS];
	uint8_t nb_pairs = 0;
	uint8_t range;
	int ret;

	for (uint8_t pair = 0; pair < AD7866_NUM_PAIRS; pair++) {
		if ((data->channels & AD7866_PAIR_MASK(pair)) != 0) {
			pairs[nb_pairs++] = pair;
		}
	}

	range = (data->channels & data->range_2x_mask) != 0 ? 1 : 0;

	/* A previous read may have left another pair or range selected. */
	ret = ad7866_select(dev, pairs[0], range);
	if (ret != 0) {
		return ret;
	}

	for (uint8_t i = 0; i < nb_pairs; i++) {
		uint16_t val_a;
		uint16_t val_b;

		ret = ad7866_convert(dev, pairs[(i + 1) % nb_pairs], &val_a, &val_b);
		if (ret != 0) {
			return ret;
		}

		vals[pairs[i]] = val_a;
		vals[pairs[i] + 2] = val_b;
	}

	for (uint8_t ch = 0; ch < AD7866_NUM_CHANNELS; ch++) {
		if ((data->channels & BIT(ch)) != 0) {
			*samples++ = vals[ch];
		}
	}

	return 0;
}

static int ad7866_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
	const struct ad7866_config *config = dev->config;
	struct ad7866_data *data = dev->data;

	if (channel_cfg->channel_id >= AD7866_NUM_CHANNELS) {
		LOG_ERR("invalid channel selection %u", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (AD7866_CHANNEL_PAIR(channel_cfg->channel_id) != 0 && config->gpio_a0.port == NULL) {
		LOG_ERR("channel %u needs the A0 pin", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL &&
	    channel_cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("invalid reference %i", channel_cfg->reference);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("invalid acquisition time %u", channel_cfg->acquisition_time);
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("differential inputs are not supported");
		return -EINVAL;
	}

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		data->range_2x_mask &= ~BIT(channel_cfg->channel_id);
		break;
	case ADC_GAIN_1_2:
		/* The VREF +/- VREF range must not exceed the supply. */
		if (config->gpio_range.port == NULL) {
			LOG_ERR("gain 1/2 needs the RANGE pin");
			return -ENOTSUP;
		}

		if (2 * config->vref_mv > config->avdd_mv) {
			LOG_ERR("gain 1/2 needs a supply of at least 2 x VREF");
			return -EINVAL;
		}

		data->range_2x_mask |= BIT(channel_cfg->channel_id);
		break;
	default:
		LOG_ERR("invalid gain %i", channel_cfg->gain);
		return -EINVAL;
	}

	return 0;
}

static int ad7866_validate_sequence(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ad7866_data *data = dev->data;
	uint8_t channels = sequence->channels;
	size_t needed;

	if (sequence->resolution != AD7866_RESOLUTION) {
		LOG_ERR("invalid resolution");
		return -EINVAL;
	}

	if (sequence->oversampling != 0) {
		LOG_ERR("oversampling is not supported");
		return -EINVAL;
	}

	if (sequence->channels == 0 || (sequence->channels & ~BIT_MASK(AD7866_NUM_CHANNELS)) != 0) {
		LOG_ERR("invalid channel selection");
		return -EINVAL;
	}

	/* One RANGE pin is shared by all the channels. */
	if ((channels & data->range_2x_mask) != 0 && (channels & ~data->range_2x_mask) != 0) {
		LOG_ERR("channels with different gains in one sequence");
		return -EINVAL;
	}

	needed = POPCOUNT(channels) * sizeof(uint16_t);
	if (sequence->options != NULL) {
		needed *= 1 + sequence->options->extra_samplings;
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ad7866_data *data = CONTAINER_OF(ctx, struct ad7866_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->buffer_ptr;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ad7866_data *data = CONTAINER_OF(ctx, struct ad7866_data, ctx);

	data->channels = ctx->sequence.channels;
	data->buffer_ptr = data->buffer;
	k_sem_give(&data->acquire_signal);
}

static int ad7866_adc_start_read(const struct device *dev, const struct adc_sequence *sequence,
				 bool wait)
{
	struct ad7866_data *data = dev->data;
	int result;

	result = ad7866_validate_sequence(dev, sequence);
	if (result != 0) {
		LOG_ERR("sequence validation failed");
		return result;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	if (wait) {
		result = adc_context_wait_for_completion(&data->ctx);
	}

	return result;
}

static int ad7866_adc_perform_read(const struct device *dev)
{
	struct ad7866_data *data = dev->data;
	int result;

	k_sem_take(&data->acquire_signal, K_FOREVER);

	result = ad7866_read_scan(dev, data->buffer);
	if (result != 0) {
		LOG_ERR("reading sample failed: %d", result);
		adc_context_complete(&data->ctx, result);
		return result;
	}

	data->buffer += POPCOUNT(data->channels);

	adc_context_on_sampling_done(&data->ctx, dev);

	return result;
}

#if CONFIG_ADC_ASYNC
static int ad7866_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct ad7866_data *data = dev->data;
	int result;

	adc_context_lock(&data->ctx, true, async);
	result = ad7866_adc_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, result);

	return result;
}

static int ad7866_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ad7866_data *data = dev->data;
	int result;

	adc_context_lock(&data->ctx, false, NULL);
	result = ad7866_adc_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, result);

	return result;
}

static void ad7866_acquisition_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;

	while (true) {
		ad7866_adc_perform_read(dev);
	}
}
#else
static int ad7866_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ad7866_data *data = dev->data;
	int result;

	adc_context_lock(&data->ctx, false, NULL);
	result = ad7866_adc_start_read(dev, sequence, false);

	while (result == 0 && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
		result = ad7866_adc_perform_read(dev);
	}

	/* A failed sample completes the context, discard it for the next read. */
	if (result != 0) {
		k_sem_reset(&data->ctx.sync);
	}

	adc_context_release(&data->ctx, result);

	return result;
}
#endif /* CONFIG_ADC_ASYNC */

static int ad7866_init(const struct device *dev)
{
	const struct ad7866_config *config = dev->config;
	struct ad7866_data *data = dev->data;
	uint16_t words[AD7866_NUM_ADCS];
	int ret;

	adc_context_init(&data->ctx);

	k_sem_init(&data->acquire_signal, 0, 1);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI device is not ready");
		return -ENODEV;
	}

	if (config->bus.config.frequency < AD7866_SPI_MIN_HZ ||
	    config->bus.config.frequency > AD7866_SPI_MAX_HZ) {
		LOG_ERR("SPI frequency out of range");
		return -EINVAL;
	}

	if (config->gpio_a0.port != NULL) {
		if (!gpio_is_ready_dt(&config->gpio_a0)) {
			LOG_ERR("A0 GPIO is not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->gpio_a0, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("failed to configure the A0 GPIO");
			return ret;
		}
	}

	if (config->gpio_range.port != NULL) {
		if (!gpio_is_ready_dt(&config->gpio_range)) {
			LOG_ERR("RANGE GPIO is not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->gpio_range, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("failed to configure the RANGE GPIO");
			return ret;
		}
	}

	/*
	 * After power-up the part may be in any power mode. A first dummy
	 * conversion wakes it up and samples the pins, then the full power-down
	 * wake-up time is waited and a checked conversion confirms the device.
	 */
	ret = ad7866_transfer(dev, words);
	if (ret != 0) {
		return ret;
	}

	k_msleep(AD7866_POWER_UP_MS);

	ret = ad7866_frame(dev, NULL, NULL);
	if (ret != 0) {
		LOG_ERR("device did not respond as expected: %d", ret);
		return ret;
	}

#if CONFIG_ADC_ASYNC
	k_tid_t tid = k_thread_create(&data->thread, data->stack,
				      CONFIG_ADC_AD7866_ACQUISITION_THREAD_STACK_SIZE,
				      ad7866_acquisition_thread, (void *)dev, NULL, NULL,
				      CONFIG_ADC_AD7866_ACQUISITION_THREAD_INIT_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ad7866");
#endif

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(adc, ad7866_api) = {
	.channel_setup = ad7866_channel_setup,
	.read = ad7866_read,
	.ref_internal = AD7866_VREF_INTERNAL_MV,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ad7866_adc_read_async,
#endif
};

#define AD7866_INIT(inst)                                                                          \
	BUILD_ASSERT(DT_INST_PROP(inst, vref_mv) >= 2000 && DT_INST_PROP(inst, vref_mv) <= 3000,   \
		     "vref-mv must be between 2000 and 3000");                                     \
	BUILD_ASSERT(DT_INST_PROP(inst, avdd_mv) >= 2700 && DT_INST_PROP(inst, avdd_mv) <= 5250,   \
		     "avdd-mv must be between 2700 and 5250");                                     \
	static const struct ad7866_config ad7866_config_##inst = {                                 \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_CONTROLLER | SPI_MODE_CPOL |         \
							  SPI_WORD_SET(8) | SPI_TRANSFER_MSB),     \
		.gpio_a0 = GPIO_DT_SPEC_INST_GET_OR(inst, a0_gpios, {0}),                          \
		.gpio_range = GPIO_DT_SPEC_INST_GET_OR(inst, range_gpios, {0}),                    \
		.vref_mv = DT_INST_PROP(inst, vref_mv),                                            \
		.avdd_mv = DT_INST_PROP(inst, avdd_mv),                                            \
	};                                                                                         \
	static struct ad7866_data ad7866_data_##inst;                                              \
	DEVICE_DT_INST_DEFINE(inst, ad7866_init, NULL, &ad7866_data_##inst, &ad7866_config_##inst, \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &ad7866_api);

DT_INST_FOREACH_STATUS_OKAY(AD7866_INIT)
