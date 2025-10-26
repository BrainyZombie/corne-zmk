/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_tps43

/**
 * @file tps43.c
 * @brief TPS43 Trackpad Feature Handler (Mouse Movement)
 *
 * This driver implements the mouse movement feature for the TPS43 trackpad.
 * It uses the low-level library (tps43_ll) for all hardware communication
 * and focuses on feature implementation.
 *
 * Future features (tap detection, gestures, scroll) will be added here.
 */

#include "tps43.h"
#include "tps43_ll.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tps43, CONFIG_SENSOR_LOG_LEVEL);

/* Input event codes */
#ifndef INPUT_REL_X
#define INPUT_REL_X 0x00
#endif
#ifndef INPUT_REL_Y
#define INPUT_REL_Y 0x01
#endif

/* ============================================================================
 * Driver Data Structures
 * ============================================================================ */

/**
 * @brief TPS43 driver runtime data
 */
struct tps43_data {
    struct k_work_delayable work;        /**< Periodic polling work queue */
    struct k_mutex lock;                 /**< Thread safety lock */
    const struct device *dev;            /**< Device pointer */

    /* Hardware state (managed by low-level library) */
    struct iqs5xx_device_state hw_state; /**< Device state */

    /* Feature handlers */
    struct tps43_mouse_data mouse;       /**< Mouse movement state */
    struct tps43_features features;      /**< Feature configuration */

    /* Callbacks */
    sensor_trigger_handler_t trigger_handler;
    const struct sensor_trigger *trigger;
};

/* Use iqs5xx_config from low-level library */
typedef struct iqs5xx_config tps43_config;

/* ============================================================================
 * Mouse Movement Feature Handler
 * ============================================================================ */

int tps43_handle_mouse_movement(const struct device *dev,
                                const struct iqs5xx_touch_data *touch,
                                struct tps43_mouse_data *mouse)
{
    /* Check if there's an active touch */
    if (touch->num_fingers > 0) {
        /* Get first finger position */
        int16_t x = touch->fingers[0].x;
        int16_t y = touch->fingers[0].y;

        /* Calculate deltas (only if finger was already down) */
        if (mouse->was_touching) {
            int16_t delta_x = x - mouse->last_x;
            int16_t delta_y = y - mouse->last_y;

            /* Report relative movement if there's actual movement */
            if (delta_x != 0 || delta_y != 0) {
                LOG_DBG("Mouse movement: dx=%d dy=%d", delta_x, delta_y);
                input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
                input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_NO_WAIT);  /* sync=true */
            }
        } else {
            LOG_INF("Initial touch - not reporting movement yet");
        }

        /* Update last position for next delta calculation */
        mouse->last_x = x;
        mouse->last_y = y;
        mouse->was_touching = true;

    } else {
        /* Finger lifted - reset tracking */
        if (mouse->was_touching) {
            LOG_INF("Touch released");
        }
        mouse->was_touching = false;
        mouse->last_x = 0;
        mouse->last_y = 0;
    }

    return 0;
}

/* ============================================================================
 * Work Queue Handler (Polling)
 * ============================================================================ */

static void tps43_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
    struct tps43_data *data = CONTAINER_OF(delayable_work, struct tps43_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_touch_data touch;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);

    /* Reinitialize device if needed */
    if (!data->hw_state.ready) {
        LOG_INF("Reinitializing device");
        ret = iqs5xx_ll_init(dev, config, &data->hw_state);
        if (ret < 0) {
            LOG_ERR("Device reinitialization failed");
            k_mutex_unlock(&data->lock);
            /* Retry after longer delay */
            k_work_reschedule(&data->work, K_MSEC(1000));
            return;
        }
    }

    /* Read touch data from hardware */
    ret = iqs5xx_ll_read_touch(dev, &touch);
    if (ret < 0) {
        LOG_ERR("Failed to read touch data");
        data->hw_state.error_count++;

        /* Reinitialize if too many errors */
        if (data->hw_state.error_count > IQS5XX_MAX_ERROR_COUNT) {
            LOG_WRN("Too many errors, marking device as not ready");
            data->hw_state.ready = false;
        }

        k_mutex_unlock(&data->lock);
        k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));
        return;
    }

    /* Reset error count on successful read */
    data->hw_state.error_count = 0;

    /* Process features */
    if (data->features.mouse_enabled) {
        tps43_handle_mouse_movement(dev, &touch, &data->mouse);
    }

    /* Future feature handlers will go here:
     * if (data->features.tap_enabled) {
     *     tps43_handle_tap_detection(dev, &touch, &data->tap);
     * }
     * if (data->features.gesture_enabled) {
     *     tps43_handle_gesture_recognition(dev, &touch, &data->gesture);
     * }
     */

    /* Trigger callback if configured */
    if (data->trigger_handler && data->trigger) {
        data->trigger_handler(dev, data->trigger);
    }

    k_mutex_unlock(&data->lock);

    /* Schedule next poll */
    k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));
}

/* ============================================================================
 * Zephyr Sensor API Implementation
 * ============================================================================ */

static int tps43_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct tps43_data *data = dev->data;
    struct iqs5xx_touch_data touch;
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY) {
        return -ENOTSUP;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = iqs5xx_ll_read_touch(dev, &touch);
    k_mutex_unlock(&data->lock);

    return ret;
}

static int tps43_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct tps43_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->mouse.last_x;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->mouse.last_y;
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
    const struct iqs5xx_config *config = dev->config;
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
    const struct iqs5xx_config *config = dev->config;
    int ret;

    LOG_INF("TPS43 driver initializing...");
    LOG_INF("I2C bus: %s", config->i2c.bus->name);
    LOG_INF("Target I2C address: 0x%02X", config->i2c.addr);

    data->dev = dev;

    /* Initialize mutex */
    k_mutex_init(&data->lock);

    /* Initialize work queue */
    k_work_init_delayable(&data->work, tps43_work_handler);

    /* Enable features */
    data->features.mouse_enabled = true;  /* Mouse movement enabled by default */

    /* Check I2C bus readiness */
    LOG_INF("Checking I2C bus...");
    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("✗ I2C bus NOT ready!");
        return -ENODEV;
    }
    LOG_INF("✓ I2C bus is ready!");

    /* Debug: Read GPIO pin states */
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

    /* Initialize hardware using low-level library */
    ret = iqs5xx_ll_init(dev, config, &data->hw_state);
    if (ret < 0) {
        LOG_ERR("Device initialization failed: %d", ret);
        return ret;
    }

    /* Start periodic polling */
    LOG_INF("Starting periodic polling mode (%dms interval)", CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS);
    k_work_reschedule(&data->work, K_MSEC(CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS));

    LOG_INF("TPS43 driver initialized successfully");
    return 0;
}

/* ============================================================================
 * Device Tree Instantiation
 * ============================================================================ */

#define TPS43_DEFINE(inst)                                                    \
    static struct tps43_data tps43_data_##inst;                             \
                                                                             \
    static const struct iqs5xx_config tps43_config_##inst = {                \
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
