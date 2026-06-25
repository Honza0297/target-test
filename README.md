# esp-usb-bridge target test (echo slave)

Companion firmware for testing the **SPI** and **I2C** bridging features of
[esp-usb-bridge](../repos/lab/esp-usb-bridge). It runs on an **ESP32-C6** acting
as the *target* chip and **echoes** every payload the bridge sends it, using the
bridge's framing protocol:

```
Frame = [LEN_H][LEN_L][PAYLOAD...]      (LEN = big-endian uint16, 0 = "no data")
```

End-to-end data path being tested:

```
host terminal  --USB CDC-->  bridge  --SPI/I2C-->  C6 (this) --echo-->  bridge  --USB CDC-->  host terminal
```

Type into the bridge's serial port and you should see your text echoed back.

## 1. Configure & flash the bridge

In the esp-usb-bridge project (`idf.py menuconfig`):

- **Bridge Configuration → Communication interface between the bridge and the target** → `SPI` or `I2C`.
- For SPI, set **Serial Handler Configuration → SPI clock frequency** to **1 MHz** for first bring-up
  (the C6 slave uses the GPIO matrix; start slow, then raise it).
- Note the pin numbers (SPI MOSI/MISO/SCLK/CS or I2C SDA/SCL + address `0x28`).

Then `idf.py build flash`.

## 2. Configure & flash this target

```bash
idf.py set-target esp32c6
idf.py menuconfig    # Target Test Configuration → pick the SAME bus as the bridge
idf.py build flash monitor
```

The defaults already match a 7-bit address of `0x28` for I2C.

## 3. Wiring (bridge ↔ C6)

Defaults shown; change either side in menuconfig to taste. **Always connect GND.**

### SPI (bridge = master, C6 = slave)

| Signal | Bridge pin (default) | C6 pin (default) |
|--------|----------------------|------------------|
| MOSI   | 11                   | 7                |
| MISO   | 13                   | 2                |
| SCLK   | 12                   | 6                |
| CS     | 10                   | 10               |
| GND    | GND                  | GND              |

### I2C (bridge = master, C6 = slave @ 0x28)

| Signal | Bridge pin (default) | C6 pin (default) |
|--------|----------------------|------------------|
| SDA    | 8                    | 5                |
| SCL    | 9                    | 4                |
| GND    | GND                  | GND              |

### UART (symmetric — note the cross-over)

| Bridge pin (default)     | C6 pin (default) |
|--------------------------|------------------|
| TXD (5)                  | RX (4)           |
| RXD (6)                  | TX (5)           |
| GND                      | GND              |

For UART the host terminal baud rate matters: the bridge forwards the host's
CDC line-coding baud to its UART, so set your terminal to the same baud as the
target (`TARGET_TEST_UART_BAUD`, default 115200).

Both sides enable internal pull-ups for I2C, but at higher clocks add external
pull-ups (e.g. 2.2k–4.7k to 3V3).

> **Do not** wire the bridge's BOOT/RST lines to the C6. The bridge toggles them
> from the USB CDC DTR/RTS line state and would reset the target. Only the bus
> lines and GND are needed. Power the C6 from its own USB or share 3V3 + GND.

## 4. Test

1. Open the bridge's USB serial port on the host (`idf.py -p <bridge_port> monitor`,
   `minicom`, `screen`, PuTTY, …) at any baud rate.
2. In a second terminal, watch this target: `idf.py -p <c6_port> monitor`.
3. Type text into the bridge terminal. You should see:
   - on the C6: `RX N bytes from bridge: <text>` then `TX N bytes to bridge (poll served)`,
   - back in the bridge terminal: your text echoed.

## Notes & limitations

- **Echo only.** The target buffers the last frame and returns it on the next
  poll (the bridge polls ~every 10 ms). It is a connectivity/throughput test,
  not a real protocol endpoint.
- **Flashing is not exercised** — UF2/esptool flashing over the bridge is
  UART-only by design; this target only covers data bridging.
- **SPI clock:** the C6 slave routes through the GPIO matrix, so very high clocks
  may be unreliable. Start at 1 MHz.
- The receive staging buffer is single-entry; rapid back-to-back bridge writes
  faster than the target task drains them may be coalesced. Fine for interactive
  testing.
