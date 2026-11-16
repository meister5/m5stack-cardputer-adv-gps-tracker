# Cardputer ADV GPS Tracker

GPS trail recorder with trail drawing and a backtrack arrow for the M5Stack Cardputer ADV with the Cap LoRa868.

## Features

- Records a trail as you move (a point every 2 m or more, up to 2000 points)
- Live trail map, auto-scaled, with point count, distance, satellites and speed
- Backtrack screen: trail with start and end markers, a heading arrow and the distance to the target trail point
- GPS info screen: satellites, HDOP, altitude, speed, date and time
- GPS debug screen: raw character stream, sentence count and fix state
- Saves the trail to the SD card as CSV

## Hardware

- M5Stack Cardputer ADV
- Cap LoRa868 (its GNSS receiver is what the sketch reads)
- Optional microSD card for saving trails

The sketch reads GPS on `G15` (RX) and `G13` (TX) at 115200 baud.

## Usage

The sketch starts on the GPS debug screen. Press `Enter` to reach the menu.

| Key | Action |
|---|---|
| `1` | Start recording |
| `2` | Save trail to `/trk_<seconds>.csv` (`time,lat,lon,alt`) |
| `3` | Backtrack map |
| `4` | GPS info |
| `5` | Clear trail |
| `9` | GPS debug |
| `Enter` | Leave the current screen (stops recording) |

## Build

The repo is a single Arduino sketch, `gpstrackerv2.ino`. Open it in the Arduino IDE, or build it with `arduino-cli` for the `m5stack:esp32:m5stack_cardputer` board. It needs the `M5Cardputer` and `TinyGPSPlus` libraries.
