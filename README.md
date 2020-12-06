# r2_dome_position

This project implements a customized controller for my model R2-D2. It provides a servo-like function in that the specific position of the dome can be set and this project will ensure it gets there.

## Hardware
* [Pololu SMC 18v7](https://www.pololu.com/product/1373)
* [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
* [CALT 12-bit Absolute Magnetic Rotary Encoder](https://www.aliexpress.com/item/32798941702.html?spm=a2g0s.9042311.0.0.40694c4dXe7zuw)

## Connections
The rotary encoder is connected to the Teensy via the SPI port. The following table shows the connections

| Rotary Encoder Pin | Wire Colour | Teensy |
| VCC (+3.3V or +5V) | Red         | +3.3V  |
| GND                | Black       | GND    |
| CS                 | Yellow      | 10     |
| CLK                | Blue        |        |
| DO                 | Green       |        |
| Shield             | Bold Black  | GND    |

The Teensy is also connected to the motor controller via UART.

## Protocol
