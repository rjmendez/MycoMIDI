# ADC module (dry zone)

One module = one ADS131M08 (8ch, simultaneous, 24-bit, SPI, ~$5).
Lives away from humidity, connected to the pin board by cable.

## Per-module parts
- 1x ADS131M08 (TQFP-32, hand-solderable)
- 1.2V reference is internal -- no external ref needed
- Decoupling: 100nF + 1uF per supply pin, per datasheet
- Passive input protection per channel: series 1k-10k resistor +
  small (100pF-1nF) cap to ground for RF/ESD, before each AINxP/AINxN
- Screw terminal or header block matching the pin-board cable

## Shared bus (scales 8 -> 16 -> 24 -> 32ch)
- CLKIN: common to all modules (from one oscillator or MCU-generated
  clock), required for sample-synchronous capture across modules
- SCLK / MOSI / MISO: shared SPI bus across all modules
- CS: one unique line per module (CS1, CS2, CS3, CS4 ...)
- SYNC/RESET: shared, strobed once at startup to align all modules'
  sample periods
- DRDY: only one module's DRDY needs to be monitored -- all modules
  convert on the same clock edge once synced

## Scaling table
| Channels | Modules | New wires per step |
|----------|---------|---------------------|
| 8        | 1       | -                   |
| 16       | 2       | +1 CS               |
| 24       | 3       | +1 CS               |
| 32       | 4       | +1 CS               |

## Per-channel calibration (do this in software, not analog)
- Record a shorted/disconnected baseline per channel at startup
- Use ADS131M08's built-in offset/gain calibration registers
- Log gain=1 or gain=2 initially -- raise only if signal is too
  small to resolve above ADC noise floor at your data rate
