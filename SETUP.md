# Goodix 27c6:5125 on the Huawei MateBook D (BOM-WXX9)

Setup guide, PAM configuration and per-device requirements:
https://github.com/uSp8ce/matebook-d-bom-wxx9-linux/tree/main/fingerprint

## Driver changes in this branch

- Firmware 12508: 5120-pixel frame layout with CRC check, the 12-argument `identifyImage` engine call, and factory ChicagoHS calibration/preprocessing.
- Each touch reads up to 3 frames about 80 ms apart and stops at the first match (`GOODIX5125_BURST` overrides).
- Enrollment takes 20 touches. The engine holds up to 32 images per template, and a print can hold several templates (format version 4).
- libfprint's heat model is off for this driver. The default disabled the reader after about 3 minutes of waiting.
- A finger already on the sensor while the driver arms is matched straight away instead of becoming the empty baseline.
- The TLS handshake waits 20 ms after each server flight instead of 200 ms.
- If the sensor completes start-up but sends no image, the driver soft-resets the sensor MCU (nothing is written) and fails that attempt. The next attempt works. Check with `journalctl -u fprintd | grep 'restarting its firmware'`.
- The pairing key, calibration and baseline default to `/var/lib/fprint/goodix5125-materials/{psk.bin,calibration.dat,baseline.raw}`. The `GOODIX5125_PSK_FILE`, `GOODIX5125_CALIBRATION_FILE` and `GOODIX5125_BASELINE_FILE` environment variables override them.

## Rebuilding the Arch package after a change

Commit here, then for each new driver commit run `git format-patch -1 <commit> --start-number <n> -o packaging/arch`. Add the patch to `source`, update `sha256sums` and `pkgver`, then run `makepkg -f` in `packaging/arch`. `check()` runs the goodix unit tests with the engine.

## Tuning tools

`tools/capture-run.sh <set> <touches> <print-file>` (as root) stops fprintd and saves 3 frames per touch under `/root/g5125-capture/<set>`. `tools/g5125-eval.c` and `tools/g5125-mkprint.c` evaluate thresholds and build prints offline. `tools/build-dev.sh` makes a dev build into `./build`.
