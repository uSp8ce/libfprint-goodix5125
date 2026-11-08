#!/usr/bin/env bash
# Dev build of libfprint (goodixtls5125 only) + the capture tool, into ./build
set -euo pipefail
cd "$(dirname "$0")/.."
# Optional: point PATH/PKG_CONFIG_PATH at newer meson/deps if your distro is too old.
if [[ ! -d build ]]; then
  meson setup build -Ddrivers=goodixtls5125 -Dintrospection=false -Ddoc=false \
    -Dinstalled-tests=false -Dudev_rules=disabled -Dudev_hwdb=disabled \
    -Dprefix=/usr -Dlibdir=lib -Dgtk-examples=false
fi
ninja -C build
gcc -O2 -Wall -o build/g5125-capture tools/g5125-capture.c -Ilibfprint -Ibuild/libfprint \
  $(pkg-config --cflags glib-2.0 gio-2.0 gusb) -Lbuild/libfprint -lfprint-2 \
  $(pkg-config --libs glib-2.0 gobject-2.0 gio-2.0)
