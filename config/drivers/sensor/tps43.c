/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_tps43

#include "tps43.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tps43, CONFIG_SENSOR_LOG_LEVEL);

/* Input event codes (from Linux input event codes) */
#ifndef INPUT_REL_X
#define INPUT_REL_X 0x00
#endif
#ifndef INPUT_REL_Y
#define INPUT_REL_Y 0x01
#endif

/* Forward declarations */
static int iqs5xx_device_init(const struct device *dev);
static int iqs5xx_verify_product_id(const struct device *dev);
static int iqs5xx_configure_device(const struct device *dev);
static int iqs5xx_close_comms_window(const struct device *dev);
static int iqs5xx_ack_event(const struct device *dev);

struct tps43_data {
    struct k_work_delayable work;
    struct k_mutex lock;
    const struct device *dev;

    /* Touch data */
    int16_t x;
    int16_t y;
    uint8_t touch_state;
    uint8_t touch_strength;

    /* Position tracking for delta calculation */
    int16_t last_x;
    int16_t last_y;
    bool was_touching;

    /* Device state */
    bool device_ready;
    bool initialized;
    uint8_t error_count;

    /* Callbacks */
    sensor_trigger_handler_t trigger_handler;
    const struct sensor_trigger *trigger;
};

struct tps43_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec rst_gpio;
    uint16_t resolution_x;
    uint16_t resolution_y;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
};

/* ============================================================================
 * I2C Helper Functions for 16-bit Big-Endian Register Addressing
 * ============================================================================ */

/**
 * Read from IQS5xx register (16-bit address, big-endian)
 */
static int iqs5xx_read_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len)
{
    const struct tps43_config *config = dev->config;
    uint8_t reg_addr[2];
    int ret;

    /* Convert register address to big-endian */
    reg_addr[0] = (reg >> 8) & 0xFF;  /* MSB */
    reg_addr[1] = reg & 0xFF;          /* LSB */

    LOG_DBG("I2C READ: addr=0x%02X reg=0x%04X len=%d", config->i2c.addr, reg, len);

    /* IQS5xx uses write-then-read for register access */
    ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, data, len);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%04X: %d", reg, ret);
        return ret;
    }

    if (len > 0) {
        LOG_DBG("I2C READ SUCCESS: reg=0x%04X data[0]=0x%02X", reg, data[0]);
    }

    return 0;
}

/**
 * Write to IQS5xx register (16-bit address, big-endian)
 */
static int iqs5xx_write_reg(const struct device *dev, uint16_t reg, const uint8_t *data, size_t len)
{
    const struct tps43_config *config = dev->config;
    uint8_t buffer[len + 2];
    int ret;

    /* Build packet: [reg_msb][reg_lsb][data...] */
    buffer[0] = (reg >> 8) & 0xFF;  /* MSB */
    buffer[1] = reg & 0xFF;          /* LSB */
    memcpy(&buffer[2], data, len);

    LOG_DBG("I2C WRITE: addr=0x%02X reg=0x%04X len=%d", config->i2c.addr, reg, len);

    ret = i2c_write_dt(&config->i2c, buffer, len + 2);
    if (ret < 0) {
        LOG_ERR("Failed to write register 0x%04X: %d", reg, ret);
        return ret;
    }

    LOG_DBG("I2C WRITE SUCCESS: reg=0x%04X", reg);

    return 0;
}

/**
 * Close communication window by writing to END_COMM register
 */
static int iqs5xx_close_comms_window(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    uint8_t reg_addr[2];
    int ret;

    /* END_COMM is special - just write the register address itself */
    reg_addr[0] = (IQS5XX_END_COMM >> 8) & 0xFF;
    reg_addr[1] = IQS5XX_END_COMM & 0xFF;

    LOG_DBG("Closing communication window (0xEEEE)");

    ret = i2c_write_dt(&config->i2c, reg_addr, 2);
    if (ret < 0) {
        LOG_ERR("Failed to close comms window: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * Acknowledge event by writing to SYS_CTRL0
 */
static int iqs5xx_ack_event(const struct device *dev)
{
    uint8_t ack = IQS5XX_ACK_RESET;
    int ret;

    LOG_DBG("Acknowledging event (SYS_CTRL0)");

    ret = iqs5xx_write_reg(dev, IQS5XX_SYS_CTRL0, &ack, 1);
    if (ret < 0) {
        LOG_ERR("Failed to acknowledge event: %d", ret);
        return ret;
    }

    return 0;
}

/* ============================================================================
 * Device Initialization
 * ============================================================================ */

/**
 * Verify product number (should be 58 for IQS572)
 */
static int iqs5xx_verify_product_id(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    uint8_t prod_num_data[2];
    uint16_t product_num;
    int ret;

    LOG_INF("Reading product number from 0x%02X...", config->i2c.addr);

    ret = iqs5xx_read_reg(dev, IQS5XX_PROD_NUM, prod_num_data, 2);
    if (ret < 0) {
        LOG_ERR("✗ Failed to read product number (error: %d)", ret);
        LOG_ERR("  Possible causes:");
        LOG_ERR("    - Device not powered");
        LOG_ERR("    - Wrong I2C address");
        LOG_ERR("    - Wiring issue");
        LOG_ERR("    - Pull-up resistors missing");
        return ret;
    }

    /* Product number is stored as big-endian */
    product_num = sys_get_be16(prod_num_data);

    LOG_INF("✓ Product Number: 0x%04X (decimal: %u)", product_num, product_num);

    /* Verify it's an IQS572 */
    if (product_num != IQS5XX_PROD_NUM_IQS572) {
        LOG_WRN("⚠ Unexpected product number: %u (expected: %u for IQS572)",
                product_num, IQS5XX_PROD_NUM_IQS572);
        LOG_WRN("  Device may be IQS550 (%u) or IQS525 (%u)",
                IQS5XX_PROD_NUM_IQS550, IQS5XX_PROD_NUM_IQS525);
        /* Don't fail - other IQS5xx variants should work too */
    } else {
        LOG_INF("✓ Confirmed IQS572 trackpad controller");
    }

    return 0;
}

/**
 * Configure device settings
 */
static int iqs5xx_configure_device(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    uint8_t sys_cfg[2];
    uint8_t sys_info[2];
    int ret;

    LOG_INF("Configuring IQS5xx device...");

    /* Read SYS_INFO to check for reset flag */
    ret = iqs5xx_read_reg(dev, IQS5XX_SYS_INFO0, sys_info, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read system info");
        return ret;
    }

    LOG_DBG("SYS_INFO: [0x%02X 0x%02X]", sys_info[0], sys_info[1]);

    /* Check if device shows reset flag (bit 7 of SYS_INFO0) */
    if (sys_info[0] & BIT(7)) {
        LOG_INF("Device shows reset flag - acknowledging...");
        ret = iqs5xx_ack_event(dev);
        if (ret < 0) {
            LOG_ERR("Failed to acknowledge reset");
            return ret;
        }
        ret = iqs5xx_close_comms_window(dev);
        if (ret < 0) {
            return ret;
        }
        k_msleep(10);  /* Brief delay after ACK */
    }

    /* Read current system configuration */
    ret = iqs5xx_read_reg(dev, IQS5XX_SYS_CFG0, sys_cfg, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read system config");
        return ret;
    }

    LOG_DBG("Current config: SYS_CFG0=0x%02X SYS_CFG1=0x%02X", sys_cfg[0], sys_cfg[1]);

    /* Configure SYS_CFG0: Setup complete, enable WDT, enable both re-ATI modes */
    sys_cfg[0] = IQS5XX_SETUP_COMPLETE | IQS5XX_WDT | IQS5XX_ALP_REATI | IQS5XX_REATI;

    /* Configure SYS_CFG1: Coordinate transformations
     * NOTE: We DON'T set EVENT_MODE because we're polling without INT pin.
     * EVENT_MODE makes the device only update registers when INT is asserted,
     * but we need continuous updates for polling mode.
     */
    sys_cfg[1] = IQS5XX_TP_EVENT;  /* Enable touch events, but NOT event mode */

    if (config->invert_x) {
        sys_cfg[1] |= IQS5XX_FLIP_X;
        LOG_DBG("  Enabling X flip");
    }
    if (config->invert_y) {
        sys_cfg[1] |= IQS5XX_FLIP_Y;
        LOG_DBG("  Enabling Y flip");
    }
    if (config->swap_xy) {
        sys_cfg[1] |= IQS5XX_SWITCH_XY;
        LOG_DBG("  Enabling XY swap");
    }

    LOG_DBG("Writing config: SYS_CFG0=0x%02X SYS_CFG1=0x%02X", sys_cfg[0], sys_cfg[1]);

    ret = iqs5xx_write_reg(dev, IQS5XX_SYS_CFG0, sys_cfg, 2);
    if (ret < 0) {
        LOG_ERR("Failed to write system config");
        return ret;
    }

    /* Close communication window after config */
    ret = iqs5xx_close_comms_window(dev);
    if (ret < 0) {
        return ret;
    }

    /* Wait for Automatic Tuning Implementation (ATI) to complete */
    LOG_INF("Waiting %dms for ATI to complete...", IQS5XX_ATI_WAIT_MS);
    k_msleep(IQS5XX_ATI_WAIT_MS);

    /* Read back configuration to verify */
    uint8_t readback_cfg[2];
    ret = iqs5xx_read_reg(dev, IQS5XX_SYS_CFG0, readback_cfg, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read back system config");
        return ret;
    }

    LOG_INF("Readback config: SYS_CFG0=0x%02X SYS_CFG1=0x%02X", readback_cfg[0], readback_cfg[1]);

    if (readback_cfg[0] != sys_cfg[0] || readback_cfg[1] != sys_cfg[1]) {
        LOG_WRN("⚠ Configuration mismatch!");
        LOG_WRN("  Expected: SYS_CFG0=0x%02X SYS_CFG1=0x%02X", sys_cfg[0], sys_cfg[1]);
        LOG_WRN("  Got:      SYS_CFG0=0x%02X SYS_CFG1=0x%02X", readback_cfg[0], readback_cfg[1]);
    } else {
        LOG_INF("✓ Configuration verified successfully");
    }

    /* Close comms window after readback */
    ret = iqs5xx_close_comms_window(dev);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("✓ Device configured successfully (resolution: %dx%d)",
            config->resolution_x, config->resolution_y);

    return 0;
}

/**
 * Initialize the IQS5xx device
 */
static int iqs5xx_device_init(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    int ret;

    LOG_INF("=== Starting IQS5xx device initialization ===");

    /* Verify product ID */
    ret = iqs5xx_verify_product_id(dev);
    if (ret < 0) {
        LOG_ERR("✗ Device verification failed (error: %d)", ret);
        return ret;
    }

    /* Configure device */
    ret = iqs5xx_configure_device(dev);
    if (ret < 0) {
        LOG_ERR("✗ Device configuration failed (error: %d)", ret);
        return ret;
    }

    data->device_ready = true;
    data->initialized = true;

    LOG_INF("=== ✓ IQS5xx device initialized successfully ===");
    return 0;
}

/* ============================================================================
 * Touch Data Reading
 * ============================================================================ */

/**
 * Read touch data from device
 */
static int iqs5xx_read_touch_data(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    /* Status structure: 2 (sys_info) + 1 (num_active) + 4 (rel_x/y) + 35 (5 fingers * 7 bytes) = 42 bytes */
    uint8_t status[42];
    int ret;

    if (!data->device_ready) {
        return -ENODEV;
    }

    /* Read from SYS_INFO0: complete status structure */
    ret = iqs5xx_read_reg(dev, IQS5XX_SYS_INFO0, status, sizeof(status));
    if (ret < 0) {
        LOG_ERR("Failed to read touch data");
        data->error_count++;
        if (data->error_count > IQS5XX_MAX_ERROR_COUNT) {
            LOG_WRN("Too many errors, reinitializing device");
            data->device_ready = false;
            k_work_reschedule(&data->work, K_MSEC(100));
        }
        return ret;
    }

    /* Reset error count on successful read */
    data->error_count = 0;

    /* Parse status structure (per Linux kernel driver):
     * Bytes 0-1: sys_info[2]
     * Byte 2: num_active
     * Bytes 3-4: rel_x (relative X, big-endian)
     * Bytes 5-6: rel_y (relative Y, big-endian)
     * Bytes 7+: touch_data array (5 fingers * 7 bytes each)
     *   Per finger (7 bytes):
     *     Bytes 0-1: abs_x (big-endian)
     *     Bytes 2-3: abs_y (big-endian)
     *     Bytes 4-5: strength (big-endian)
     *     Byte 6: area
     */
    uint8_t num_active = status[2];

    /* Debug: Show raw data and decode SYS_INFO */
    LOG_INF("Status: SYS_INFO=[0x%02X 0x%02X] num_active=%d rel=[0x%02X%02X 0x%02X%02X] finger0=[0x%02X 0x%02X 0x%02X 0x%02X]",
            status[0], status[1], num_active,
            status[3], status[4], status[5], status[6],
            status[7], status[8], status[9], status[10]);

    /* Decode SYS_INFO flags (per Linux kernel driver) */
    uint8_t sys_info0 = status[0];
    uint8_t sys_info1 = status[1];
    LOG_DBG("SYS_INFO0 bits: SHOW_RESET=%d SETUP_COMPLETE=%d",
            (sys_info0 & BIT(7)) ? 1 : 0,  /* Show reset */
            (sys_info0 & BIT(6)) ? 1 : 0); /* Setup complete */

    if (num_active > 0 && num_active <= IQS5XX_MAX_TOUCHES) {
        data->touch_state = 1;

        /* First finger data starts at byte 7 */
        const uint8_t *finger0 = &status[7];

        data->x = sys_get_be16(&finger0[0]);  /* abs_x at bytes 0-1 */
        data->y = sys_get_be16(&finger0[2]);  /* abs_y at bytes 2-3 */
        uint16_t strength = sys_get_be16(&finger0[4]);  /* strength at bytes 4-5 */
        data->touch_strength = strength & 0xFF;

        LOG_INF("✓ TOUCH DETECTED: fingers=%d x=%d y=%d strength=%d",
                num_active, data->x, data->y, data->touch_strength);

        /* Calculate deltas for cursor movement (only if finger was already down) */
        if (data->was_touching) {
            int16_t delta_x = data->x - data->last_x;
            int16_t delta_y = data->y - data->last_y;

            /* Report relative movement to ZMK if there's actual movement */
            if (delta_x != 0 || delta_y != 0) {
                LOG_DBG("Reporting movement: dx=%d dy=%d", delta_x, delta_y);
                input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
                input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_NO_WAIT);  /* sync=true on last */
            }
        } else {
            LOG_INF("Initial touch - not reporting movement yet");
        }

        /* Update last position for next delta calculation */
        data->last_x = data->x;
        data->last_y = data->y;
        data->was_touching = true;
    } else {
        /* Finger lifted - reset position tracking */
        if (data->was_touching) {
            LOG_INF("Touch released");
        }
        data->touch_state = 0;
        data->was_touching = false;
        data->last_x = 0;
        data->last_y = 0;
        LOG_DBG("No touch detected (num_active=%d)", num_active);
    }

    /* Acknowledge the event */
    ret = iqs5xx_ack_event(dev);
    if (ret < 0) {
        LOG_WRN("Failed to ACK event: %d", ret);
        /* Don't fail - continue anyway */
    }

    /* Close communication window */
    ret = iqs5xx_close_comms_window(dev);
    if (ret < 0) {
        LOG_WRN("Failed to close comms window: %d", ret);
        /* Don't fail - continue anyway */
    }

    return 0;
}

/* ============================================================================
 * Work Queue Handler
 * ============================================================================ */

static void tps43_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
    struct tps43_data *data = CONTAINER_OF(delayable_work, struct tps43_data, work);
    const struct device *dev = data->dev;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);

    /* Reinitialize device if needed */
    if (!data->device_ready) {
        LOG_INF("Reinitializing device");
        ret = iqs5xx_device_init(dev);
        if (ret < 0) {
            LOG_ERR("Device reinitialization failed");
            k_mutex_unlock(&data->lock);
            /* Retry after longer delay */
            k_work_reschedule(&data->work, K_MSEC(1000));
            return;
        }
    }

    /* Read touch data */
    ret = iqs5xx_read_touch_data(dev);
    if (ret < 0) {
        k_mutex_unlock(&data->lock);
        /* Retry after delay even on error */
        k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));
        return;
    }

    /* Trigger callback if configured */
    if (data->trigger_handler && data->trigger) {
        data->trigger_handler(dev, data->trigger);
    }

    k_mutex_unlock(&data->lock);

    /* Schedule next poll using configured poll rate */
    k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));
}

/* ============================================================================
 * Sensor API Implementation
 * ============================================================================ */

static int tps43_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct tps43_data *data = dev->data;
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY) {
        return -ENOTSUP;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = iqs5xx_read_touch_data(dev);
    k_mutex_unlock(&data->lock);

    return ret;
}

static int tps43_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct tps43_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->x;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->y;
        val->val2 = 0;
        break;
    default:
        k_mutex_unlock(&data->lock);
        return -ENOTSUP;
    }

    k_mutex_unlock(&data->lock);
    return 0;
}

static int tps43_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler)
{
    struct tps43_data *data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret = 0;

    k_mutex_lock(&data->lock, K_FOREVER);

    if (trig->type != SENSOR_TRIG_DATA_READY) {
        ret = -ENOTSUP;
        goto unlock;
    }

    data->trigger_handler = handler;
    data->trigger = trig;

    if (handler) {
        /* Enable interrupt */
        ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_FALLING);
        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt");
            goto unlock;
        }
    } else {
        /* Disable interrupt */
        ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
    }

unlock:
    k_mutex_unlock(&data->lock);
    return ret;
}

static const struct sensor_driver_api tps43_driver_api = {
    .sample_fetch = tps43_sample_fetch,
    .channel_get = tps43_channel_get,
    .trigger_set = tps43_trigger_set,
};

/* ============================================================================
 * Driver Initialization
 * ============================================================================ */

static int tps43_init(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    LOG_INF("IQS5xx driver initializing...");
    LOG_INF("I2C bus: %s", config->i2c.bus->name);
    LOG_INF("Target I2C address: 0x%02X", config->i2c.addr);

    data->dev = dev;

    /* Initialize mutex */
    k_mutex_init(&data->lock);

    /* Initialize work queue */
    k_work_init_delayable(&data->work, tps43_work_handler);

    /* Check I2C bus readiness */
    LOG_INF("Checking I2C bus...");
    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("✗ I2C bus NOT ready!");
        return -ENODEV;
    }
    LOG_INF("✓ I2C bus is ready!");

    /* Read GPIO pin states for debugging */
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (device_is_ready(gpio0)) {
        int sda_state = gpio_pin_get_raw(gpio0, 17);  /* P0.17 = SDA */
        int scl_state = gpio_pin_get_raw(gpio0, 20);  /* P0.20 = SCL */
        LOG_INF("GPIO Pin States:");
        LOG_INF("  SDA (P0.17): %s (%d)", sda_state ? "HIGH" : "LOW", sda_state);
        LOG_INF("  SCL (P0.20): %s (%d)", scl_state ? "HIGH" : "LOW", scl_state);
    }

    /* Scan I2C bus for devices */
    int found_devices = 0;
    LOG_INF("Scanning I2C bus for devices...");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy_buf;
        ret = i2c_write_read(config->i2c.bus, addr, NULL, 0, &dummy_buf, 0);
        if (ret == 0) {
            LOG_INF("  0x%02X: ACK (device found!)", addr);
            found_devices++;
        }
    }
    LOG_INF("I2C scan complete: Found %d device(s)", found_devices);

    /* Initialize device */
    ret = iqs5xx_device_init(dev);
    if (ret < 0) {
        LOG_ERR("Device initialization failed: %d", ret);
        return ret;
    }

    /* Start periodic polling (no INT pin connected) */
    LOG_INF("Starting periodic polling mode (%dms interval)", CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS);
    k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));

    LOG_INF("IQS5xx driver initialized successfully");
    return 0;
}

#define TPS43_DEFINE(inst)                                                    \
    static struct tps43_data tps43_data_##inst;                             \
                                                                             \
    static const struct tps43_config tps43_config_##inst = {                \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
        .int_gpio = {0},                                                    \
        .rst_gpio = {0},                                                    \
        .resolution_x = DT_INST_PROP_OR(inst, resolution_x, 2048),         \
        .resolution_y = DT_INST_PROP_OR(inst, resolution_y, 1792),         \
        .invert_x = DT_INST_PROP_OR(inst, invert_x, false),                \
        .invert_y = DT_INST_PROP_OR(inst, invert_y, false),                \
        .swap_xy = DT_INST_PROP_OR(inst, swap_xy, false),                  \
    };                                                                       \
                                                                             \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL,                   \
                                &tps43_data_##inst, &tps43_config_##inst,   \
                                POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
                                &tps43_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TPS43_DEFINE)
