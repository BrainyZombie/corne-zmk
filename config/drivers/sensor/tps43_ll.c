/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file tps43_ll.c
 * @brief Low-level hardware library implementation for IQS5xx trackpad controllers
 *
 * This file contains all hardware-specific communication and device management code.
 * It is independent of feature implementations (mouse, gestures, etc.) and can be
 * reused across different feature handlers.
 */

#include "tps43_ll.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tps43_ll, CONFIG_SENSOR_LOG_LEVEL);

/* ============================================================================
 * I2C Communication Primitives
 * ============================================================================ */

int iqs5xx_ll_read_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len)
{
    const struct iqs5xx_config *config = dev->config;
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

int iqs5xx_ll_write_reg(const struct device *dev, uint16_t reg, const uint8_t *data, size_t len)
{
    const struct iqs5xx_config *config = dev->config;
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

int iqs5xx_ll_close_comms(const struct device *dev)
{
    const struct iqs5xx_config *config = dev->config;
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

int iqs5xx_ll_ack_event(const struct device *dev)
{
    uint8_t ack = IQS5XX_ACK_RESET;
    int ret;

    LOG_DBG("Acknowledging event (SYS_CTRL0)");

    ret = iqs5xx_ll_write_reg(dev, IQS5XX_SYS_CTRL0, &ack, 1);
    if (ret < 0) {
        LOG_ERR("Failed to acknowledge event: %d", ret);
        return ret;
    }

    return 0;
}

/* ============================================================================
 * Device Identification and Configuration
 * ============================================================================ */

int iqs5xx_ll_verify_product(const struct device *dev, uint16_t *product_num)
{
    const struct iqs5xx_config *config = dev->config;
    uint8_t prod_num_data[2];
    int ret;

    LOG_INF("Reading product number from 0x%02X...", config->i2c.addr);

    ret = iqs5xx_ll_read_reg(dev, IQS5XX_PROD_NUM, prod_num_data, 2);
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
    *product_num = sys_get_be16(prod_num_data);

    LOG_INF("✓ Product Number: 0x%04X (decimal: %u)", *product_num, *product_num);

    /* Verify it's an IQS5xx family device */
    if (*product_num != IQS5XX_PROD_NUM_IQS572) {
        LOG_WRN("⚠ Unexpected product number: %u (expected: %u for IQS572)",
                *product_num, IQS5XX_PROD_NUM_IQS572);
        LOG_WRN("  Device may be IQS550 (%u) or IQS525 (%u)",
                IQS5XX_PROD_NUM_IQS550, IQS5XX_PROD_NUM_IQS525);
        /* Don't fail - other IQS5xx variants should work too */
    } else {
        LOG_INF("✓ Confirmed IQS572 trackpad controller");
    }

    return 0;
}

int iqs5xx_ll_configure(const struct device *dev, const struct iqs5xx_config *config)
{
    uint8_t sys_cfg[2];
    uint8_t sys_info[2];
    int ret;

    LOG_INF("Configuring IQS5xx device...");

    /* Read SYS_INFO to check for reset flag */
    ret = iqs5xx_ll_read_reg(dev, IQS5XX_SYS_INFO0, sys_info, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read system info");
        return ret;
    }

    LOG_DBG("SYS_INFO: [0x%02X 0x%02X]", sys_info[0], sys_info[1]);

    /* Check if device shows reset flag (bit 7 of SYS_INFO0) */
    if (sys_info[0] & IQS5XX_SHOW_RESET) {
        LOG_INF("Device shows reset flag - acknowledging...");
        ret = iqs5xx_ll_ack_event(dev);
        if (ret < 0) {
            LOG_ERR("Failed to acknowledge reset");
            return ret;
        }
        ret = iqs5xx_ll_close_comms(dev);
        if (ret < 0) {
            return ret;
        }
        k_msleep(10);  /* Brief delay after ACK */
    }

    /* Read current system configuration */
    ret = iqs5xx_ll_read_reg(dev, IQS5XX_SYS_CFG0, sys_cfg, 2);
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

    ret = iqs5xx_ll_write_reg(dev, IQS5XX_SYS_CFG0, sys_cfg, 2);
    if (ret < 0) {
        LOG_ERR("Failed to write system config");
        return ret;
    }

    /* Close communication window after config */
    ret = iqs5xx_ll_close_comms(dev);
    if (ret < 0) {
        return ret;
    }

    /* Wait for Automatic Tuning Implementation (ATI) to complete */
    LOG_INF("Waiting %dms for ATI to complete...", IQS5XX_ATI_WAIT_MS);
    k_msleep(IQS5XX_ATI_WAIT_MS);

    /* Read back configuration to verify */
    uint8_t readback_cfg[2];
    ret = iqs5xx_ll_read_reg(dev, IQS5XX_SYS_CFG0, readback_cfg, 2);
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
    ret = iqs5xx_ll_close_comms(dev);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("✓ Device configured successfully (resolution: %dx%d)",
            config->resolution_x, config->resolution_y);

    return 0;
}

/* ============================================================================
 * Touch Data Reading
 * ============================================================================ */

int iqs5xx_ll_read_touch(const struct device *dev, struct iqs5xx_touch_data *touch_data)
{
    /* Status structure: 2 (sys_info) + 1 (num_active) + 4 (rel_x/y) + 35 (5 fingers * 7 bytes) */
    uint8_t status[IQS5XX_STATUS_BUFFER_SIZE];
    int ret;

    /* Read complete status structure from SYS_INFO0 */
    ret = iqs5xx_ll_read_reg(dev, IQS5XX_SYS_INFO0, status, sizeof(status));
    if (ret < 0) {
        LOG_ERR("Failed to read touch data");
        return ret;
    }

    /* Parse status structure (per Linux kernel driver):
     * Bytes 0-1: sys_info[2]
     * Byte 2: num_active
     * Bytes 3-4: rel_x (relative X, big-endian)
     * Bytes 5-6: rel_y (relative Y, big-endian)
     * Bytes 7+: touch_data array (5 fingers * 7 bytes each)
     */

    /* Extract system info */
    touch_data->sys_info[0] = status[0];
    touch_data->sys_info[1] = status[1];

    /* Extract number of active fingers */
    touch_data->num_fingers = status[2];

    /* Extract relative movement (signed 16-bit) */
    touch_data->rel_x = (int16_t)sys_get_be16(&status[3]);
    touch_data->rel_y = (int16_t)sys_get_be16(&status[5]);

    /* Parse finger data (if any) */
    if (touch_data->num_fingers > 0 && touch_data->num_fingers <= IQS5XX_MAX_TOUCHES) {
        for (int i = 0; i < touch_data->num_fingers; i++) {
            const uint8_t *finger = &status[IQS5XX_FINGER_DATA_OFFSET + (i * IQS5XX_FINGER_DATA_SIZE)];

            touch_data->fingers[i].x = sys_get_be16(&finger[0]);           /* abs_x */
            touch_data->fingers[i].y = sys_get_be16(&finger[2]);           /* abs_y */
            touch_data->fingers[i].strength = sys_get_be16(&finger[4]);    /* strength */
            touch_data->fingers[i].area = finger[6];                        /* area */
        }

        LOG_INF("✓ Touch: fingers=%d x=%d y=%d strength=%d",
                touch_data->num_fingers,
                touch_data->fingers[0].x,
                touch_data->fingers[0].y,
                touch_data->fingers[0].strength);
    } else {
        LOG_DBG("No touch (num_fingers=%d)", touch_data->num_fingers);
    }

    /* Acknowledge the event */
    ret = iqs5xx_ll_ack_event(dev);
    if (ret < 0) {
        LOG_WRN("Failed to ACK event: %d", ret);
        /* Don't fail - continue anyway */
    }

    /* Close communication window */
    ret = iqs5xx_ll_close_comms(dev);
    if (ret < 0) {
        LOG_WRN("Failed to close comms window: %d", ret);
        /* Don't fail - continue anyway */
    }

    return 0;
}

/* ============================================================================
 * Complete Initialization
 * ============================================================================ */

int iqs5xx_ll_init(const struct device *dev,
                   const struct iqs5xx_config *config,
                   struct iqs5xx_device_state *state)
{
    int ret;

    LOG_INF("=== Starting IQS5xx device initialization ===");

    /* Verify product ID */
    ret = iqs5xx_ll_verify_product(dev, &state->product_num);
    if (ret < 0) {
        LOG_ERR("✗ Device verification failed (error: %d)", ret);
        state->ready = false;
        return ret;
    }

    /* Configure device */
    ret = iqs5xx_ll_configure(dev, config);
    if (ret < 0) {
        LOG_ERR("✗ Device configuration failed (error: %d)", ret);
        state->ready = false;
        return ret;
    }

    state->initialized = true;
    state->ready = true;
    state->error_count = 0;

    LOG_INF("=== ✓ IQS5xx device initialized successfully ===");
    return 0;
}
