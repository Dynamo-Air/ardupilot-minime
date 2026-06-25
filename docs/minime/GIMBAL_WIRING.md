# SIYI A8 Mini Gimbal Wiring Guide

## Connection Method Decision

**Selected: UART Serial to TELEM2 Port**

Per Software Scope Section 9.1, UART Serial was selected for the following reasons:
- Simple wiring with proven reliability
- No additional hardware required (Cube Orange+ lacks native Ethernet)
- Sufficient bandwidth for gimbal control commands
- Video feed routes separately via HDMI to Herelink air unit

## TELEM2 Port Wiring

### Cube Orange+ TELEM2 Pinout (JST GH 6 pin)

| Pin | Signal | Connect To | Notes |
|-----|--------|------------|-------|
| 1 | VCC (5V) | Not connected | Gimbal has dedicated power supply |
| 2 | TX (out) | SIYI RX | Cube transmits to gimbal |
| 3 | RX (in) | SIYI TX | Cube receives from gimbal |
| 4 | CTS | Not connected | Hardware flow control not used |
| 5 | RTS | Not connected | Hardware flow control not used |
| 6 | GND | SIYI GND | Common ground reference |

### SIYI A8 Mini Serial Connector

| Wire Color | Signal | Connect To |
|------------|--------|------------|
| Red | VCC | Not connected (use gimbal power supply) |
| Black | GND | TELEM2 Pin 6 (GND) |
| Yellow | TX | TELEM2 Pin 3 (RX) |
| White | RX | TELEM2 Pin 2 (TX) |

Note: TX and RX are crossed (Cube TX to SIYI RX, Cube RX to SIYI TX).

## ArduPilot Parameters

The following parameters are configured in `params/minime_gimbal.param`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| MNT1_TYPE | 8 | SIYI mount type (reboot required) |
| SERIAL2_PROTOCOL | 8 | SToRM32 Gimbal Serial protocol |
| SERIAL2_BAUD | 115 | 115200 bps baud rate |
| MNT1_PITCH_MIN | -90 | Minimum pitch angle (degrees) |
| MNT1_PITCH_MAX | 25 | Maximum pitch angle (degrees) |
| MNT1_YAW_MIN | -135 | Minimum yaw angle (degrees) |
| MNT1_YAW_MAX | 135 | Maximum yaw angle (degrees) |
| MNT1_RC_RATE | 90 | RC control rate (degrees per second) |
| CAM1_TYPE | 4 | Mount/Siyi camera type |

## Video Feed Routing

Video from the SIYI A8 Mini is routed separately from gimbal control:

| Method | Connection | Notes |
|--------|------------|-------|
| HDMI | SIYI HDMI out to Herelink air unit HDMI in | Primary method, 1080p |
| Ethernet (alternative) | RTSP stream on port 37260 | Requires Ethernet adapter |

Latency target: Sub 200 ms glass to glass per Software Scope Section 9.2.

## Alternative Configuration: Ethernet

If Ethernet is required in the future (higher bandwidth, unified video/control):

| Parameter | Value |
|-----------|-------|
| IP Address | 192.168.144.25 |
| Port | 37260 |
| Protocol | RTSP for video |

Note: Ethernet requires additional hardware since Cube Orange+ does not have a native Ethernet port. An Ethernet adapter would need to be integrated.

## Commissioning Checklist

Before first use:

1. Verify physical connections (TX/RX crossover, GND)
2. Load gimbal parameters from `params/minime_gimbal.param`
3. Reboot flight controller (required after MNT1_TYPE change)
4. Verify gimbal responds to RC control (pitch and yaw)
5. Verify gimbal limits are enforced correctly
6. Test video feed via HDMI to Herelink

## Reference Documents

- Software Scope Section 9: Gimbal and Camera Integration Scope
- SPECS.md Section 30: SIYI A8 Mini Gimbal Parameters
- ArduPilot SIYI Gimbal Documentation
