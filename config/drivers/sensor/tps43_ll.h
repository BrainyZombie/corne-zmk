/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS43_LL_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS43_LL_H_

/**
 * @file tps43_ll.h
 * @brief Low-level hardware abstraction library for Azoteq TPS43 trackpad (IQS572 controller)
 *
 * This library provides hardware communication primitives and device management
 * for the IQS5xx family of trackpad controllers. It is designed to be independent
 * of any specific feature implementation (mouse, gestures, etc.).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Register Definitions (16-bit addresses, big-endian)
 * ============================================================================ */

/* Device information registers */
#define IQS5XX_PROD_NUM             0x0000  /* Product number (2 bytes) */
#define IQS5XX_VERSION_NUM          0x0002  /* Firmware version (2 bytes) */

/* Touch data registers */
#define IQS5XX_SYS_INFO0            0x000F  /* System info 0 - touch data starts here */
#define IQS5XX_SYS_INFO1            0x0010  /* System info 1 */
#define IQS5XX_NUM_FINGERS          0x0011  /* Number of active fingers */
#define IQS5XX_REL_X                0x0012  /* Relative X (2 bytes) */
#define IQS5XX_REL_Y                0x0014  /* Relative Y (2 bytes) */

/* Finger data starts at offset from SYS_INFO0 (7 bytes per finger) */
#define IQS5XX_FINGER_DATA_OFFSET   7       /* Offset from SYS_INFO0 */

/* System control registers */
#define IQS5XX_SYS_CTRL0            0x0431  /* System control 0 - ACK/reset */
#define IQS5XX_SYS_CTRL1            0x0432  /* System control 1 */

/* System configuration registers */
#define IQS5XX_SYS_CFG0             0x058E  /* System config 0 */
#define IQS5XX_SYS_CFG1             0x058F  /* System config 1 */

/* Resolution configuration */
#define IQS5XX_X_RES                0x066E  /* X resolution (2 bytes) */
#define IQS5XX_Y_RES                0x0670  /* Y resolution (2 bytes) */

/* Special commands */
#define IQS5XX_END_COMM             0xEEEE  /* End communication window */

/* ============================================================================
 * Product Identification
 * ============================================================================ */

#define IQS5XX_PROD_NUM_IQS525      52      /* IQS525 */
#define IQS5XX_PROD_NUM_IQS550      40      /* IQS550 */
#define IQS5XX_PROD_NUM_IQS572      58      /* IQS572 (TPS43) */

/* ============================================================================
 * Register Bit Definitions
 * ============================================================================ */

/* System Control 0 (SYS_CTRL0) bits */
#define IQS5XX_ACK_RESET            BIT(0)  /* Acknowledge event/reset */

/* System Config 0 (SYS_CFG0) bits */
#define IQS5XX_SETUP_COMPLETE       BIT(7)  /* Setup mode complete */
#define IQS5XX_SW_RESET             BIT(6)  /* Software reset trigger */
#define IQS5XX_WDT                  BIT(5)  /* Watchdog timer enable */
#define IQS5XX_ALP_REATI            BIT(4)  /* ALP re-ATI enable */
#define IQS5XX_REATI                BIT(3)  /* Re-ATI enable */
#define IQS5XX_MANUAL_CONTROL       BIT(2)  /* Manual control mode */
#define IQS5XX_SUSPEND              BIT(1)  /* Low power suspend mode */

/* System Config 1 (SYS_CFG1) bits */
#define IQS5XX_FLIP_X               BIT(7)  /* Flip X coordinates */
#define IQS5XX_FLIP_Y               BIT(6)  /* Flip Y coordinates */
#define IQS5XX_SWITCH_XY            BIT(5)  /* Swap X and Y axes */
#define IQS5XX_TP_EVENT             BIT(2)  /* Touch/proximity events */
#define IQS5XX_GESTURE_EVENT        BIT(1)  /* Gesture events */
#define IQS5XX_EVENT_MODE           BIT(0)  /* Event mode (INT-driven) */

/* System Info 0 (SYS_INFO0) bits */
#define IQS5XX_SHOW_RESET           BIT(7)  /* Show reset indicator */
#define IQS5XX_SETUP_COMPLETE_FLAG  BIT(6)  /* Setup complete flag */

/* ============================================================================
 * Constants
 * ============================================================================ */

#define IQS5XX_MAX_TOUCHES          5       /* Maximum simultaneous touches */
#define IQS5XX_FINGER_DATA_SIZE     7       /* Bytes per finger (x, y, strength, area) */
#define IQS5XX_STATUS_BUFFER_SIZE   42      /* Full status read size */
#define IQS5XX_ATI_WAIT_MS          250     /* Wait after config for ATI */
#define IQS5XX_RESET_DELAY_MS       20      /* Delay after reset */
#define IQS5XX_MAX_ERROR_COUNT      5       /* Max consecutive errors before reinit */

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Configuration for IQS5xx device
 */
struct iqs5xx_config {
    struct i2c_dt_spec i2c;         /**< I2C bus configuration */
    struct gpio_dt_spec int_gpio;   /**< Interrupt GPIO (optional) */
    struct gpio_dt_spec rst_gpio;   /**< Reset GPIO (optional) */
    uint16_t resolution_x;           /**< X resolution in counts */
    uint16_t resolution_y;           /**< Y resolution in counts */
    bool invert_x;                   /**< Invert X coordinates */
    bool invert_y;                   /**< Invert Y coordinates */
    bool swap_xy;                    /**< Swap X and Y axes */
};

/**
 * @brief Single finger touch data
 */
struct iqs5xx_finger_data {
    uint16_t x;                      /**< Absolute X position */
    uint16_t y;                      /**< Absolute Y position */
    uint16_t strength;               /**< Touch strength */
    uint8_t area;                    /**< Touch area */
};

/**
 * @brief Complete touch report from device
 */
struct iqs5xx_touch_data {
    uint8_t sys_info[2];             /**< System info bytes */
    uint8_t num_fingers;             /**< Number of active touches (0-5) */
    int16_t rel_x;                   /**< Relative X movement */
    int16_t rel_y;                   /**< Relative Y movement */
    struct iqs5xx_finger_data fingers[IQS5XX_MAX_TOUCHES];  /**< Finger data array */
};

/**
 * @brief Device state
 */
struct iqs5xx_device_state {
    bool initialized;                /**< Device has been initialized */
    bool ready;                      /**< Device is ready for operations */
    uint8_t error_count;             /**< Consecutive error counter */
    uint16_t product_num;            /**< Product number (58 for IQS572) */
    uint16_t firmware_version;       /**< Firmware version */
};

/* ============================================================================
 * Low-Level API Functions
 * ============================================================================ */

/**
 * @brief Read from IQS5xx register (16-bit address, big-endian)
 *
 * @param dev Device structure
 * @param reg 16-bit register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_read_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len);

/**
 * @brief Write to IQS5xx register (16-bit address, big-endian)
 *
 * @param dev Device structure
 * @param reg 16-bit register address
 * @param data Data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_write_reg(const struct device *dev, uint16_t reg, const uint8_t *data, size_t len);

/**
 * @brief Close communication window
 *
 * Must be called after any read/write operation to allow device to process
 *
 * @param dev Device structure
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_close_comms(const struct device *dev);

/**
 * @brief Acknowledge event/reset flag
 *
 * @param dev Device structure
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_ack_event(const struct device *dev);

/**
 * @brief Read and verify product ID
 *
 * @param dev Device structure
 * @param product_num Pointer to store product number
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_verify_product(const struct device *dev, uint16_t *product_num);

/**
 * @brief Initialize and configure device
 *
 * @param dev Device structure
 * @param config Configuration parameters
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_configure(const struct device *dev, const struct iqs5xx_config *config);

/**
 * @brief Read complete touch data from device
 *
 * @param dev Device structure
 * @param touch_data Pointer to structure to fill with touch data
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_read_touch(const struct device *dev, struct iqs5xx_touch_data *touch_data);

/**
 * @brief Perform full device initialization
 *
 * This combines product verification and configuration
 *
 * @param dev Device structure
 * @param config Configuration parameters
 * @param state Pointer to device state structure to update
 * @return 0 on success, negative errno on failure
 */
int iqs5xx_ll_init(const struct device *dev,
                   const struct iqs5xx_config *config,
                   struct iqs5xx_device_state *state);

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS43_LL_H_ */
