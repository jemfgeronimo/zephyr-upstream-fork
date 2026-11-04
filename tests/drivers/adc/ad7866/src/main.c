/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad7866

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#define AD7866_NODE    DT_NODELABEL(ad7866)
#define A0_GPIO_DEV    DEVICE_DT_GET(DT_GPIO_CTLR(AD7866_NODE, a0_gpios))
#define A0_GPIO_PIN    DT_GPIO_PIN(AD7866_NODE, a0_gpios)
#define RANGE_GPIO_DEV DEVICE_DT_GET(DT_GPIO_CTLR(AD7866_NODE, range_gpios))
#define RANGE_GPIO_PIN DT_GPIO_PIN(AD7866_NODE, range_gpios)

#define VREF_MV 2500

/* Model of the AD7866: two ADCs with a 2-channel mux each. */
static struct {
	int vin_mv[4];
	bool wired_b;     /* MISO on DOUTB instead of DOUTA */
	bool bad_zero;    /* leading zero is not 0, as for a missing device */
	int pair_latched; /* pins sampled on the previous CS falling edge */
	int range_latched;
	int frames;
	bool spi_config_ok;
} model;

static void model_reset(void)
{
	memset(&model, 0, sizeof(model));
	model.vin_mv[0] = 500;
	model.vin_mv[1] = 1000;
	model.vin_mv[2] = 1500;
	model.vin_mv[3] = 2000;
	model.spi_config_ok = true;
	model.pair_latched = gpio_emul_output_get(A0_GPIO_DEV, A0_GPIO_PIN);
	model.range_latched = gpio_emul_output_get(RANGE_GPIO_DEV, RANGE_GPIO_PIN);
}

static uint16_t model_code(int mv, bool twos)
{
	int code;

	if (twos) {
		code = (mv - VREF_MV) * 4096 / (2 * VREF_MV);
		code = CLAMP(code, -2048, 2047);
		return code & 0xFFF;
	}

	code = mv * 4096 / VREF_MV;

	return CLAMP(code, 0, 4095);
}

static int ad7866_emul_io(const struct emul *target, const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	uint8_t *rx;

	ARG_UNUSED(target);
	ARG_UNUSED(tx_bufs);

	if (rx_bufs == NULL || rx_bufs->count != 1 || rx_bufs->buffers[0].len != 4) {
		return -EINVAL;
	}

	/* Mode 2 (CPOL = 1, CPHA = 0), 8 bit words, MSB first. */
	if ((config->operation & SPI_MODE_CPOL) == 0 || (config->operation & SPI_MODE_CPHA) != 0 ||
	    SPI_WORD_SIZE_GET(config->operation) != 8 ||
	    (config->operation & SPI_TRANSFER_LSB) != 0 || config->frequency < 10000 ||
	    config->frequency > 20000000) {
		model.spi_config_ok = false;
	}

	rx = rx_bufs->buffers[0].buf;

	for (int i = 0; i < 2; i++) {
		int adc = (model.wired_b ? 1 : 0) ^ i;
		uint16_t word =
			(model.bad_zero ? BIT(15) : 0) | (model.range_latched << 14) |
			(model.pair_latched << 13) | (adc << 12) |
			model_code(model.vin_mv[adc * 2 + model.pair_latched], model.range_latched);

		sys_put_be16(word, &rx[2 * i]);
	}

	/* The pins are sampled on the CS falling edge, for the next conversion. */
	model.pair_latched = gpio_emul_output_get(A0_GPIO_DEV, A0_GPIO_PIN);
	model.range_latched = gpio_emul_output_get(RANGE_GPIO_DEV, RANGE_GPIO_PIN);
	model.frames++;

	return 0;
}

static struct spi_emul_api ad7866_emul_api = {
	.io = ad7866_emul_io,
};

static int ad7866_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

EMUL_DT_INST_DEFINE(0, ad7866_emul_init, NULL, NULL, &ad7866_emul_api, NULL);

static const struct device *const adc = DEVICE_DT_GET(AD7866_NODE);

static struct adc_channel_cfg channel_cfg(uint8_t id, enum adc_gain gain)
{
	return (struct adc_channel_cfg){
		.gain = gain,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = id,
	};
}

static void setup_all(enum adc_gain gain)
{
	for (uint8_t ch = 0; ch < 4; ch++) {
		struct adc_channel_cfg cfg = channel_cfg(ch, gain);

		zassert_ok(adc_channel_setup(adc, &cfg), "channel %u setup", ch);
	}
}

static int do_read(uint32_t channels, uint16_t *buf, size_t size, uint8_t extra)
{
	struct adc_sequence_options options = {
		.extra_samplings = extra,
	};
	struct adc_sequence sequence = {
		.options = extra != 0 ? &options : NULL,
		.channels = channels,
		.buffer = buf,
		.buffer_size = size,
		.resolution = 12,
	};

	return adc_read(adc, &sequence);
}

static uint16_t expected_binary(int ch)
{
	return model_code(model.vin_mv[ch], false);
}

static uint16_t expected_offset_binary(int ch)
{
	int code = (model.vin_mv[ch] - VREF_MV) * 4096 / (2 * VREF_MV);

	return code + 2048;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_true(device_is_ready(adc), "AD7866 is not ready");

	/* Go back to the 0 V to VREF range and the first pair. */
	setup_all(ADC_GAIN_1);
	model_reset();
}

ZTEST(ad7866, test_channel_setup_validation)
{
	struct adc_channel_cfg cfg;

	cfg = channel_cfg(4, ADC_GAIN_1);
	zassert_equal(adc_channel_setup(adc, &cfg), -EINVAL, "channel 4");

	cfg = channel_cfg(0, ADC_GAIN_1);
	cfg.reference = ADC_REF_VDD_1;
	zassert_equal(adc_channel_setup(adc, &cfg), -EINVAL, "reference");

	cfg = channel_cfg(0, ADC_GAIN_1);
	cfg.reference = ADC_REF_EXTERNAL0;
	zassert_ok(adc_channel_setup(adc, &cfg), "external reference");

	cfg = channel_cfg(0, ADC_GAIN_1);
	cfg.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10);
	zassert_equal(adc_channel_setup(adc, &cfg), -EINVAL, "acquisition time");

	cfg = channel_cfg(0, ADC_GAIN_1);
	cfg.differential = 1;
	zassert_equal(adc_channel_setup(adc, &cfg), -EINVAL, "differential");

	cfg = channel_cfg(0, ADC_GAIN_2);
	zassert_equal(adc_channel_setup(adc, &cfg), -EINVAL, "gain 2");

	cfg = channel_cfg(0, ADC_GAIN_1_2);
	zassert_ok(adc_channel_setup(adc, &cfg), "gain 1/2");
}

ZTEST(ad7866, test_spi_configuration)
{
	uint16_t buf[1];

	zassert_ok(do_read(BIT(0), buf, sizeof(buf), 0));
	zassert_true(model.spi_config_ok, "SPI mode, word size or frequency");
}

ZTEST(ad7866, test_single_channels)
{
	uint16_t buf[1];

	for (int ch = 0; ch < 4; ch++) {
		zassert_ok(do_read(BIT(ch), buf, sizeof(buf), 0), "channel %d", ch);
		zassert_equal(buf[0], expected_binary(ch), "channel %d: %u", ch, buf[0]);
	}
}

ZTEST(ad7866, test_all_channels_with_extra_samplings)
{
	uint16_t buf[3 * 4];

	zassert_ok(do_read(0xF, buf, sizeof(buf), 2));

	for (int sample = 0; sample < 3; sample++) {
		for (int ch = 0; ch < 4; ch++) {
			zassert_equal(buf[sample * 4 + ch], expected_binary(ch),
				      "sample %d channel %d", sample, ch);
		}
	}
}

ZTEST(ad7866, test_channel_subsets_in_ascending_order)
{
	uint16_t buf[2];

	zassert_ok(do_read(BIT(3) | BIT(0), buf, sizeof(buf), 0));
	zassert_equal(buf[0], expected_binary(0));
	zassert_equal(buf[1], expected_binary(3));

	zassert_ok(do_read(BIT(2) | BIT(1), buf, sizeof(buf), 0));
	zassert_equal(buf[0], expected_binary(1));
	zassert_equal(buf[1], expected_binary(2));
}

ZTEST(ad7866, test_both_dout_wirings)
{
	uint16_t buf[4];

	model.wired_b = true;
	zassert_ok(do_read(0xF, buf, sizeof(buf), 0));

	for (int ch = 0; ch < 4; ch++) {
		zassert_equal(buf[ch], expected_binary(ch), "channel %d", ch);
	}
}

ZTEST(ad7866, test_gain_half_range)
{
	uint16_t buf[4];
	int32_t mv;

	setup_all(ADC_GAIN_1_2);
	model.vin_mv[1] = 4000;
	model.vin_mv[3] = 100;

	zassert_ok(do_read(0xF, buf, sizeof(buf), 0));

	for (int ch = 0; ch < 4; ch++) {
		zassert_equal(buf[ch], expected_offset_binary(ch), "channel %d: %u", ch, buf[ch]);

		mv = buf[ch];
		zassert_ok(adc_raw_to_millivolts(VREF_MV, ADC_GAIN_1_2, 12, &mv));
		zassert_within(mv, model.vin_mv[ch], 2, "channel %d: %d mV", ch, mv);
	}

	/* and back to the 0 V to VREF range */
	setup_all(ADC_GAIN_1);
	zassert_ok(do_read(0xF, buf, sizeof(buf), 0));

	for (int ch = 0; ch < 4; ch++) {
		zassert_equal(buf[ch], model_code(model.vin_mv[ch], false), "channel %d", ch);
	}
}

ZTEST(ad7866, test_mixed_gains_in_one_sequence)
{
	struct adc_channel_cfg cfg = channel_cfg(1, ADC_GAIN_1_2);
	uint16_t buf[2];

	zassert_ok(adc_channel_setup(adc, &cfg));

	zassert_equal(do_read(BIT(0) | BIT(1), buf, sizeof(buf), 0), -EINVAL);
	zassert_ok(do_read(BIT(0), buf, sizeof(buf), 0));
	zassert_ok(do_read(BIT(1), buf, sizeof(buf), 0));
	zassert_equal(buf[0], expected_offset_binary(1));
}

ZTEST(ad7866, test_sequence_validation)
{
	struct adc_sequence_options options = {.extra_samplings = 0};
	uint16_t buf[4];
	struct adc_sequence sequence = {
		.channels = BIT(0),
		.buffer = buf,
		.buffer_size = sizeof(buf),
		.resolution = 12,
	};

	sequence.resolution = 10;
	zassert_equal(adc_read(adc, &sequence), -EINVAL, "resolution");

	sequence.resolution = 12;
	sequence.oversampling = 1;
	zassert_equal(adc_read(adc, &sequence), -EINVAL, "oversampling");

	sequence.oversampling = 0;
	sequence.channels = 0;
	zassert_equal(adc_read(adc, &sequence), -EINVAL, "no channel");

	sequence.channels = BIT(4);
	zassert_equal(adc_read(adc, &sequence), -EINVAL, "channel 4");

	sequence.channels = 0xF;
	sequence.buffer_size = 3 * sizeof(uint16_t);
	zassert_equal(adc_read(adc, &sequence), -ENOMEM, "buffer too small");

	sequence.buffer_size = sizeof(buf);
	sequence.options = &options;
	options.extra_samplings = 1;
	zassert_equal(adc_read(adc, &sequence), -ENOMEM, "extra samplings do not fit");
}

ZTEST(ad7866, test_conversions_per_scan)
{
	uint16_t buf[4];
	int frames;

	/* one pair: one conversion per scan, in steady state */
	zassert_ok(do_read(BIT(0) | BIT(2), buf, sizeof(buf), 0));
	frames = model.frames;
	zassert_ok(do_read(BIT(0) | BIT(2), buf, sizeof(buf), 0));
	zassert_equal(model.frames - frames, 1, "single pair");

	/* two pairs: one conversion per pair, none wasted */
	zassert_ok(do_read(0xF, buf, sizeof(buf), 0));
	frames = model.frames;
	zassert_ok(do_read(0xF, buf, sizeof(buf), 0));
	zassert_equal(model.frames - frames, 2, "both pairs");
}

ZTEST(ad7866, test_missing_device_is_reported)
{
	uint16_t buf[1];

	model.bad_zero = true;
	zassert_equal(do_read(BIT(0), buf, sizeof(buf), 0), -EIO);

	/* it recovers once the device answers again */
	model.bad_zero = false;
	zassert_ok(do_read(BIT(0), buf, sizeof(buf), 0));
	zassert_equal(buf[0], expected_binary(0), "got %u expected %u", buf[0], expected_binary(0));
}

ZTEST_SUITE(ad7866, NULL, NULL, before, NULL, NULL);
