/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS43_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS43_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/* IQS5xx-B000 Register Definitions (16-bit addresses, big-endian) */
#define IQS5XX_PROD_NUM             0x0000  /* Product number (2 bytes) */
#define IQS5XX_SYS_INFO0            0x000F  /* System info 0 - touch data starts here */
#define IQS5XX_SYS_INFO1            0x0010  /* System info 1 */
#define IQS5XX_NUM_FINGERS          0x0011  /* Number of fingers */
#define IQS5XX_REL_X                0x0012  /* Relative X */
#define IQS5XX_REL_Y                0x0013  /* Relative Y */
#define IQS5XX_ABS_X                0x0014  /* Absolute X (first finger) */
#define IQS5XX_ABS_Y                0x0016  /* Absolute Y (first finger) */
#define IQS5XX_TOUCH_STRENGTH       0x0018  /* Touch strength */
#define IQS5XX_AREA                 0x001A  /* Touch area */

#define IQS5XX_SYS_CTRL0            0x0431  /* System control 0 - ACK/reset */
#define IQS5XX_SYS_CTRL1            0x0432  /* System control 1 */
#define IQS5XX_SYS_CFG0             0x058E  /* System config 0 */
#define IQS5XX_SYS_CFG1             0x058F  /* System config 1 */
#define IQS5XX_X_RES                0x066E  /* X resolution (2 bytes) */
#define IQS5XX_Y_RES                0x0670  /* Y resolution (2 bytes) */
#define IQS5XX_END_COMM             0xEEEE  /* End communication window */

/* Product Numbers */
#define IQS5XX_PROD_NUM_IQS550      40
#define IQS5XX_PROD_NUM_IQS572      58      /* 0x003A */
#define IQS5XX_PROD_NUM_IQS525      52

/* System Control 0 bits */
#define IQS5XX_ACK_RESET            BIT(0)  /* Acknowledge event */

/* System Config 0 bits */
#define IQS5XX_SETUP_COMPLETE       BIT(7)  /* Setup mode complete */
#define IQS5XX_SW_RESET             BIT(6)  /* Software reset */
#define IQS5XX_WDT                  BIT(5)  /* Watchdog enable */
#define IQS5XX_ALP_REATI            BIT(4)  /* ALP re-ATI */
#define IQS5XX_REATI                BIT(3)  /* Re-ATI */
#define IQS5XX_MANUAL_CONTROL       BIT(2)  /* Manual control */
#define IQS5XX_SUSPEND              BIT(1)  /* Suspend */

/* System Config 1 bits */
#define IQS5XX_FLIP_X               BIT(7)  /* Flip X coordinates */
#define IQS5XX_FLIP_Y               BIT(6)  /* Flip Y coordinates */
#define IQS5XX_SWITCH_XY            BIT(5)  /* Swap X and Y */
#define IQS5XX_TP_EVENT             BIT(2)  /* Touch/prox events */
#define IQS5XX_GESTURE_EVENT        BIT(1)  /* Gesture events */
#define IQS5XX_EVENT_MODE           BIT(0)  /* Event mode enable */

/* Touch data constants */
#define IQS5XX_MAX_TOUCHES          5       /* Maximum simultaneous touches */
#define IQS5XX_FINGER_SIZE          7       /* Bytes per finger data */

/* Timing constants (from kernel driver) */
#define IQS5XX_ATI_WAIT_MS          250     /* Wait after config for ATI */
#define IQS5XX_RESET_DELAY_MS       20      /* Delay after reset */

/* Error recovery */
#define IQS5XX_MAX_ERROR_COUNT      5

void tps43_ready_callback(const struct device *gpio_dev,
			  struct gpio_callback *cb, uint32_t pins);

/* Forward declaration for GPIO callback structure */
struct gpio_callback;

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS43_H_ */
