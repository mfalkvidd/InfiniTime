# Screenshot Service

## Introduction

The Screenshot Service triggers a 240x240 display capture and writes it to LittleFS as raw RGB565.

The trigger characteristic requires an encrypted bonded connection. The BLE callback only validates the request and posts it to the system/display task; the display task performs the capture and file write.

## Service

The service UUID is **00060000-78fc-48fe-8e23-433b3a1942d0**.

## Characteristics

### Trigger (UUID 00060001-78fc-48fe-8e23-433b3a1942d0)

Write any value to this characteristic to request a screenshot. The characteristic is write-only.

### Status (UUID 00060002-78fc-48fe-8e23-433b3a1942d0)

The status characteristic supports read and notify.

Possible values:

- `idle`
- `busy`
- `ok:/screenshots/shot-YYYYMMDD-HHMMSS.rgb565`
- `err`

Screenshot files are written to `/screenshots/` and are 115200 bytes: 240 pixels wide, 240 pixels high, 2 bytes per pixel. Pixels are stored as RGB565, most significant byte first.

## Triggering with bluetoothctl

Pair, trust, and connect to the watch first. The trigger requires the encrypted bonded connection created by pairing.

```sh
bluetoothctl
```

```text
[bluetooth]# scan on
[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[InfiniTime]# menu gatt
[InfiniTime]# list-attributes
[InfiniTime]# select-attribute 00060002-78fc-48fe-8e23-433b3a1942d0
[InfiniTime:/service.../char...]# notify on
[InfiniTime:/service.../char...]# select-attribute 00060001-78fc-48fe-8e23-433b3a1942d0
[InfiniTime:/service.../char...]# write 0x01
```

The status notification first reports `busy`, then `ok:/screenshots/...rgb565` when the file is ready. Use the existing BLE FS service to read the returned path from LittleFS.

## Converting to GIF

After reading the file from the watch:

```sh
convert -size 240x240 RGB565:shot-YYYYMMDD-HHMMSS.rgb565 shot-YYYYMMDD-HHMMSS.gif
```

With ImageMagick 7, the equivalent command is:

```sh
magick -size 240x240 RGB565:shot-YYYYMMDD-HHMMSS.rgb565 shot-YYYYMMDD-HHMMSS.gif
```
