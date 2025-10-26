# TPS43 Trackpad Implementation Status

## Summary

Successfully integrated an Azoteq TPS43 (IQS572 controller) trackpad with ZMK firmware for a Corne keyboard. Basic 1-finger cursor movement is now working via mouse emulation.

## Hardware Setup

- **Trackpad**: Azoteq TPS43 with IQS572 controller (43mm variant)
- **Resolution**: 2048 x 1792
- **I2C Address**: 0x74
- **Board**: nice_nano_v2 (nRF52840)
- **Connections**: VCC, GND, SDA (P0.17), SCL (P0.20) only
  - No INT pin connected (using polling mode)
  - RST connected to MCU's RST (not controllable)

## Current Status ✓

### Working Features
- ✅ I2C communication with IQS572
- ✅ Device initialization and configuration
- ✅ Touch detection
- ✅ **Basic cursor movement (1 finger drag)**
- ✅ Polling mode at 10Hz (100ms intervals)
- ✅ ZMK input integration

### Key Technical Solutions

1. **I2C Pull-ups Issue**
   - Problem: nRF52840 upstream doesn't enable `bias-pull-up` for I2C
   - Solution: Created `nice_nano_v2.overlay` with explicit `bias-pull-up` in pinctrl
   - File: `config/boards/shields/corne_tps43/boards/nice_nano_v2.overlay`

2. **Event Mode vs Polling Mode**
   - Problem: Device was in EVENT_MODE, only updating registers on INT assertion
   - Solution: Disabled `IQS5XX_EVENT_MODE` bit in SYS_CFG1 for continuous updates
   - Config: `SYS_CFG1 = IQS5XX_TP_EVENT` (without EVENT_MODE bit)

3. **Reset Acknowledgement**
   - Problem: Device showed SHOW_RESET flag (0x80 in SYS_INFO0)
   - Solution: Check for reset flag during init and acknowledge before configuring
   - Added: `IQS5XX_ALP_REATI` flag that was missing from initial config

4. **16-bit Big-Endian Register Addressing**
   - IQS5xx uses 16-bit big-endian register addresses
   - All register reads/writes use MSB-first addressing
   - Communication window protocol: write 0xEEEE to close window

## File Structure

```
config/
├── drivers/sensor/
│   ├── CMakeLists.txt          # Builds tps43.c
│   ├── Kconfig                 # CONFIG_ZMK_SENSOR_TPS43
│   ├── tps43.c                 # Main driver implementation
│   └── tps43.h                 # IQS5xx register definitions
├── dts/bindings/sensor/
│   └── azoteq,tps43.yaml       # Device tree bindings
└── boards/shields/corne_tps43/
    ├── boards/
    │   └── nice_nano_v2.overlay    # I2C pinctrl with pull-ups
    ├── corne_tps43.dtsi            # Common definitions
    ├── corne_tps43_left.overlay    # Left side + input listener
    ├── corne_tps43_left.conf       # Left config + ZMK_POINTING
    ├── corne_tps43_right.overlay   # Right side + input listener
    ├── corne_tps43_right.conf      # Right config + ZMK_POINTING
    ├── corne_tps43.keymap          # Keymap
    └── Kconfig.*                   # Shield definitions
```

## Key Register Configuration

### Product ID
- **Expected**: 0x003A (58 decimal) for IQS572
- **Register**: 0x0000 (IQS5XX_PROD_NUM)

### System Configuration
- **SYS_CFG0** (0x058E): `0xB8`
  - `IQS5XX_SETUP_COMPLETE` (bit 7)
  - `IQS5XX_WDT` (bit 5) - Watchdog
  - `IQS5XX_ALP_REATI` (bit 4) - ALP re-ATI
  - `IQS5XX_REATI` (bit 3) - Re-ATI

- **SYS_CFG1** (0x058F): `0x04`
  - `IQS5XX_TP_EVENT` (bit 2) - Touch/prox events
  - **NOT** `IQS5XX_EVENT_MODE` - Disabled for polling mode

### Touch Data Structure (42 bytes from 0x000F)
```
Bytes 0-1:   sys_info[2]
Byte 2:      num_active (number of fingers)
Bytes 3-4:   rel_x (relative X, big-endian)
Bytes 5-6:   rel_y (relative Y, big-endian)
Bytes 7+:    touch_data (5 fingers * 7 bytes each)
  Per finger:
    Bytes 0-1: abs_x (big-endian)
    Bytes 2-3: abs_y (big-endian)
    Bytes 4-5: strength (big-endian)
    Byte 6:    area
```

## ZMK Integration

### Input Reporting
Driver reports relative movement using Zephyr input API:
```c
input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_NO_WAIT);
```

### Input Listener
Device tree creates listener to capture events:
```dts
tps43_listener: tps43_listener {
    compatible = "zmk,input-listener";
    device = <&tps43>;
};
```

### Configuration
```
CONFIG_ZMK_POINTING=y
```

### How It Works
1. Driver polls trackpad every 100ms
2. Calculates delta (change since last position)
3. Reports delta via `input_report_rel()`
4. ZMK input listener captures events
5. ZMK sends HID mouse reports to host
6. Computer sees cursor movement

## Gesture Support (Hardware Capabilities)

The IQS572 chip supports hardware gesture recognition (not yet implemented):

### 1-Finger Gestures (GESTURE_EVENTS_0)
- Single tap
- Press and hold
- Swipe X+ / X-
- Swipe Y+ / Y-

### 2-Finger Gestures (GESTURE_EVENTS_1)
- 2 simultaneous taps
- Scroll
- Zoom

### Planned Mappings
- **1 finger moving** → cursor movement ✅ (DONE)
- **1 finger tap** → left click (TODO)
- **1 finger press & hold** → drag mode (TODO)
- **2 finger tap** → right click (TODO)
- **2 finger scroll** → scroll wheel vertical/horizontal (TODO)
- **2 finger zoom** → Ctrl+scroll for zoom (TODO)
- **Swipes** → not mapped (user requested nothing for swipes)

## Next Steps

### To Enable Gesture Support

1. **Read datasheet for:**
   - GESTURE_EVENTS_0 register address
   - GESTURE_EVENTS_1 register address
   - Bit mapping for each gesture
   - How to enable gesture detection (already have IQS5XX_GESTURE_EVENT bit)
   - Whether gesture bits are sticky or auto-clear

2. **Implementation approach:**
   - Enable `IQS5XX_GESTURE_EVENT` bit (bit 1) in SYS_CFG1
   - Read gesture registers alongside touch data
   - Map gestures to ZMK events:
     - Taps → mouse button clicks
     - Scroll → `INPUT_REL_WHEEL` / `INPUT_REL_HWHEEL`
     - Zoom → Modifiers + scroll
     - Press & hold → Enter drag mode

3. **Driver changes needed:**
   - Add gesture register reads to `iqs5xx_read_touch_data()`
   - Implement gesture detection logic
   - Send appropriate input events (buttons, scroll, modifiers)
   - Add state machine for drag mode

## Build Process

Using GitHub Actions (defined in `build.yaml`):
```yaml
board: nice_nano_v2
shield: corne_tps43_right  # or corne_tps43_left
```

No local build needed - push to trigger Actions build.

## Important Notes

- **After enabling CONFIG_ZMK_POINTING**: Must unpair and re-pair BLE devices (HID descriptor changes)
- **Polling rate**: Currently 10Hz (100ms). Can be adjusted if needed.
- **No external pull-ups**: Using nRF52840 internal pull-ups (~13kΩ)
- **Split keyboard**: Trackpad on right side only (can mirror to left if needed)

## Debug Info

### Logs to Watch
- Device initialization: Product ID 0x003A confirmation
- Configuration readback: Verify SYS_CFG0/1 written correctly
- Touch detection: `num_active` changing from 0 to 1+
- Movement reporting: `dx` and `dy` values
- SYS_INFO flags: Check for unexpected resets (bit 7)

### Common Issues
- All addresses respond on I2C → Missing pull-ups
- Touch detected but `num_active=0` → EVENT_MODE enabled (should be disabled)
- Device shows 0x80 in SYS_INFO → Reset flag, need to acknowledge
- No touch detected → Check ATI completed (250ms wait)

## References

- Linux kernel driver: `drivers/input/touchscreen/iqs5xx.c`
- IQS5xx datasheet: Available from Azoteq
- ZMK pointing docs: https://zmk.dev/docs/development/hardware-integration/pointing
- Zephyr input API: https://docs.zephyrproject.org/apidoc/latest/group__input__interface.html

## Session History

1. Fixed I2C pull-up issues
2. Debugged I2C scanning code
3. Found correct device address (0x74)
4. Rewrote driver based on Linux kernel driver
5. Fixed 16-bit big-endian register addressing
6. Added communication window protocol
7. Fixed touch data parsing structure
8. Added reset acknowledgement
9. Disabled EVENT_MODE for polling
10. **Implemented basic cursor movement with ZMK integration**

---

*Last updated: 2025-10-25*
*Status: Basic cursor movement working, gestures pending datasheet review*
