#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

VERSION="1.0.0"
ARCH="$(dpkg --print-architecture)"
PKG_NAME="pardus_${VERSION}_${ARCH}"
BUILD_DIR="$ROOT_DIR/build_deb"
PKG_DIR="$BUILD_DIR/$PKG_NAME"
DEB_OUT="$BUILD_DIR/${PKG_NAME}.deb"

rm -rf "$BUILD_DIR"
mkdir -p "$PKG_DIR/DEBIAN" \
         "$PKG_DIR/usr/bin" \
         "$PKG_DIR/usr/share/pardus" \
         "$PKG_DIR/usr/share/applications" \
         "$PKG_DIR/usr/share/mime/packages" \
         "$PKG_DIR/usr/share/icons/hicolor/48x48/apps" \
         "$PKG_DIR/usr/share/icons/hicolor/96x96/mimetypes" \
         "$PKG_DIR/etc/pardus"

echo "==> 1. Compiling PARDUS binaries for Linux ($ARCH)..."
GTK_CFLAGS=$(pkg-config --cflags gtk4)
GTK_LIBS=$(pkg-config --libs gtk4)

gcc -O2 pardserver.c -Wall -o "$PKG_DIR/usr/bin/pardserver" -lzmq -lsodium -lm

gcc -O2 $GTK_CFLAGS -Wall -Wno-format-truncation \
    resources.c pardclient.c -o "$PKG_DIR/usr/bin/pardclient" \
    $GTK_LIBS -lm -lzmq -lpng -ljpeg -lfontconfig -lpulse-simple -lpulse -lX11

gcc -O2 -w $GTK_CFLAGS \
    resources.c pcs-cli.c -o "$PKG_DIR/usr/bin/pcs-cli" \
    $GTK_LIBS -lm -lzmq -lpng -ljpeg -lfontconfig -lpulse-simple -lpulse -lX11

gcc -O2 $GTK_CFLAGS -Wall \
    pardcap.c -o "$PKG_DIR/usr/bin/pardcap" \
    $GTK_LIBS -lm -lpng -ljpeg -lpulse-simple -lpulse -lfontconfig

chmod 755 "$PKG_DIR/usr/bin/"*

echo "==> 2. Installing Manuals, AI Guides, PCS Samples, and Configs..."
[ -d "AI" ]           && cp -R "AI" "$PKG_DIR/usr/share/pardus/"
[ -d "PCS" ]          && cp -R "PCS" "$PKG_DIR/usr/share/pardus/"
[ -d "User_Manuals" ] && cp -R "User_Manuals" "$PKG_DIR/usr/share/pardus/"

for cfg in pardserver.config pardusers.txt server_identity.key pcscli.cfg; do
    if [ -f "$cfg" ]; then
        cp "$cfg" "$PKG_DIR/etc/pardus/$cfg"
        chmod 644 "$PKG_DIR/etc/pardus/$cfg"
    fi
done
[ -f "$PKG_DIR/etc/pardus/server_identity.key" ] && chmod 600 "$PKG_DIR/etc/pardus/server_identity.key"

echo "==> 3. Installing Icons, Desktop Entry, and .dove MIME Association..."
[ -f "pardclient.png" ] && cp "pardclient.png" "$PKG_DIR/usr/share/icons/hicolor/48x48/apps/pardus.png"
[ -f "dove.png" ]       && cp "dove.png"       "$PKG_DIR/usr/share/icons/hicolor/96x96/mimetypes/application-x-pardus-dove.png"

printf '%s\n' \
  '[Desktop Entry]' \
  'Type=Application' \
  'Name=Pardus Client' \
  'Comment=Remote Robotics and Vision Automation Client' \
  'Exec=/usr/bin/pardclient' \
  'Icon=pardus' \
  'Terminal=false' \
  'Categories=Science;Engineering;Development;' \
  > "$PKG_DIR/usr/share/applications/pardus.desktop"

printf '%s\n' \
  '[Desktop Entry]' \
  'Type=Application' \
  'Name=Pardus PCS Runner' \
  'Comment=Run compiled PARDUS .dove program' \
  'Exec=/usr/bin/pcs-cli %f' \
  'Icon=application-x-pardus-dove' \
  'Terminal=true' \
  'MimeType=application/x-pardus-dove;' \
  'NoDisplay=true' \
  > "$PKG_DIR/usr/share/applications/pardus-dove.desktop"

printf '%s\n' \
  '<?xml version="1.0" encoding="UTF-8"?>' \
  '<mime-info xmlns="http://freedesktop.org">' \
  '  <mime-type type="application/x-pardus-dove">' \
  '    <comment>Pardus Compiled PCS Program</comment>' \
  '    <glob pattern="*.dove"/>' \
  '    <icon name="application-x-pardus-dove"/>' \
  '  </mime-type>' \
  '</mime-info>' \
  > "$PKG_DIR/usr/share/mime/packages/application-x-pardus-dove.xml"

echo "==> 4. Creating DEBIAN metadata and conffiles..."
INSTALLED_KB=$(du -sk "$PKG_DIR" | cut -f1)

printf '%s\n' \
  "Package: pardus" \
  "Version: ${VERSION}" \
  "Section: science" \
  "Priority: optional" \
  "Architecture: ${ARCH}" \
  "Installed-Size: ${INSTALLED_KB}" \
  "Maintainer: TadPath <https://codeberg.org>" \
  "Depends: libgtk-4-1, libzmq5, libsodium23, libpng16-16 | libpng16-16t64, libjpeg8 | libjpeg62-turbo, libfontconfig1, libpulse0, libx11-6" \
  "Description: PARDUS Remote Robotics and Vision Automation Suite" \
  " Includes pardclient (GTK 4 GUI), pcs-cli (command-line runner)," \
  " pardserver (network server), pardcap (camera capture), manuals," \
  " AI coding references, and sample PCS/DOVE scripts." \
  > "$PKG_DIR/DEBIAN/control"

printf '%s\n' \
  '/etc/pardus/pardserver.config' \
  '/etc/pardus/pardusers.txt' \
  '/etc/pardus/server_identity.key' \
  '/etc/pardus/pcscli.cfg' \
  > "$PKG_DIR/DEBIAN/conffiles"

printf '%s\n' \
  '#!/bin/sh' \
  'set -e' \
  'update-mime-database /usr/share/mime >/dev/null 2>&1 || true' \
  'update-desktop-database /usr/share/applications >/dev/null 2>&1 || true' \
  'gtk-update-icon-cache -q -t -f /usr/share/icons/hicolor >/dev/null 2>&1 || true' \
  > "$PKG_DIR/DEBIAN/postinst"
chmod 755 "$PKG_DIR/DEBIAN/postinst"

printf '%s\n' \
  '#!/bin/sh' \
  'set -e' \
  'update-mime-database /usr/share/mime >/dev/null 2>&1 || true' \
  'update-desktop-database /usr/share/applications >/dev/null 2>&1 || true' \
  'gtk-update-icon-cache -q -t -f /usr/share/icons/hicolor >/dev/null 2>&1 || true' \
  > "$PKG_DIR/DEBIAN/postrm"
chmod 755 "$PKG_DIR/DEBIAN/postrm"

echo "==> 5. Building .deb package..."
dpkg-deb --root-owner-group --build "$PKG_DIR" "$DEB_OUT"

echo "==> Done! Debian package ready at: $DEB_OUT"
