/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS43_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS43_H_

/**
 * @file tps43.h
 * @brief TPS43 Feature Handler API
 *
 * This file contains declarations for feature-specific functionality
 * (mouse movement, tap detection, gestures, etc.) that uses the
 * low-level hardware library (tps43_ll).
 */

#include "tps43_ll.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/**
 * @brief Mouse movement feature data
 *
 * Tracks state for delta calculation and cursor movement
 */
struct tps43_mouse_data {
    int16_t last_x;          /**< Last X position for delta calculation */
    int16_t last_y;          /**< Last Y position for delta calculation */
    bool was_touching;       /**< Was finger down in previous read */
};

/**
 * @brief Feature handler configuration
 *
 * This will be extended as more features are added
 * (tap detection, gestures, etc.)
 */
struct tps43_features {
    bool mouse_enabled;      /**< Enable mouse movement feature */
    /* Future features:
     * bool tap_enabled;
     * bool gesture_enabled;
     * bool scroll_enabled;
     */
};

/* ============================================================================
 * Feature Handler Functions
 * ============================================================================ */

/**
 * @brief Process touch data for mouse movement
 *
 * Calculates deltas and reports input events
 *
 * @param dev Device structure
 * @param touch Touch data from low-level library
 * @param mouse Mouse state data
 * @return 0 on success, negative errno on failure
 */
int tps43_handle_mouse_movement(const struct device *dev,
                                const struct iqs5xx_touch_data *touch,
                                struct tps43_mouse_data *mouse);

/* Future feature handlers:
 * int tps43_handle_tap_detection(const struct device *dev, ...);
 * int tps43_handle_gesture_recognition(const struct device *dev, ...);
 * int tps43_handle_scroll(const struct device *dev, ...);
 */

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS43_H_ */
