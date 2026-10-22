# Clock

A weight-driven, 3D-printed pendulum clock with a deadbeat escapement. The pendulum period is 1 s, so there are 2 beats per second.

## Pendulum timer (`firmware/`)

ESP32 firmware written in Rust, with no Arduino and no ESP-IDF. It uses `esp-hal`, `esp-radio` and `embassy`. An IR beam on **GPIO22** sits at roughly the bottom of the swing. A paper flag about ¼″ (6.35 mm) wide on the pendulum breaks the beam on every pass.

The live page at `http://pendulum.local` shows:

- **Clock vs real time**: how far the clock is ahead or behind, in seconds. It's calculated as `beats × 0.5 s − real elapsed`, counted from the first beat after boot or after **Re-zero**. It also shows real time and pendulum time side by side.
- **Rate** in s/day, from full periods (tick + tock). A beam that isn't exactly at dead centre doesn't bias it.
- **Period** against the 1000 ms target, **beat error** (tick/tock asymmetry) and **swing amplitude**. Amplitude is estimated from how long the flag blocks the beam: `A = w / (2·sin(π·t_blocked / T))`.

Each pass is timestamped at the **middle** of its blocked pulse. That way, a change in amplitude (which changes the pulse width) doesn't shift the timing. Real time comes from the ESP32's crystal, which is typically within 1–2 s/day.

Flag width and pivot-to-flag distance are constants at the top of `firmware/pendulum-timer/src/timing.rs`. They only affect the amplitude estimate.

### Layout

| Path | What |
|---|---|
| `firmware/pendulum-timer/src/timing.rs` | All the timing maths (pure Rust, no hardware) |
| `firmware/pendulum-timer/src/bin/main.rs` | Beam interrupt, Wi-Fi, setup hotspot, HTTP server, mDNS |
| `firmware/pendulum-timer/src/page.html` | The dashboard |
| `firmware/pendulum-timer/src/creds.rs` | Saved Wi-Fi credentials (in the `nvs` flash partition) |
| `firmware/pendulum-timer/src/wifitxt.rs` | Parser for a plain-text `wifi.txt` flashed to that partition |
| `firmware/pendulum-timer/src/mdns.rs` | Tiny `pendulum.local` responder |
| `firmware/host-tests/` | Runs the timing and mDNS tests on a PC against a simulated pendulum |
| `archive/arduino-pendulum-timer/` | The original Arduino sketch (v1), kept for reference |

### Build and flash (Windows)

One-time setup:

```
winget install Rustlang.Rustup
cargo install espup espflash --locked
espup install --targets esp32
```

Then, from `firmware/pendulum-timer`:

```
cargo run --release        # builds, flashes and opens the serial log (espflash)
```

If you don't want to install a toolchain, flash a prebuilt merged image `pendulum-timer-esp32.bin` at offset **0x0** with the browser flasher at https://espressif.github.io/esptool-js/ (Chrome or Edge).

### Tests

```
cd firmware/host-tests
cargo test
```

### First boot / Wi-Fi

With no network saved, the board opens an open hotspot named **Pendulum-Setup**. Join it, open `http://192.168.4.1`, pick your network and enter the password. The board then reboots onto your network.

**Or skip the hotspot with a text file.** Copy `firmware/pendulum-timer/wifi.example.txt` to `wifi.txt` (it's gitignored) and fill it in:

```
ssid=YourNetwork
pass=YourPassword
```

Then flash it to the start of the `nvs` partition:

```
python -m esptool --chip esp32 write_flash 0x9000 wifi.txt
```

`cargo run` / `espflash flash` leave it alone. The merged `pendulum-timer-esp32.bin` does **not**: it's padded with 0xFF over 0x9000, so after flashing that, flash `wifi.txt` again (two separate commands, merged image first). Whichever was written last wins: the text file or a network saved from the setup page. `/forget` erases both. Leave out `pass` for an open network; lines starting with `#` are ignored.

If the saved network can't be reached (router down, for example), it falls back to the hotspot and retries the saved network after about 5 minutes. To erase the saved network, open `/forget`.
