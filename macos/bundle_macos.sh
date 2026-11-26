#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

BUILD_DIR="$ROOT_DIR/build_macos"
APP_DIR="$BUILD_DIR/PARDUS.app"
CONTENTS="$APP_DIR/Contents"
MACOS_BIN="$CONTENTS/MacOS"
FRAMEWORKS="$CONTENTS/Frameworks"
RESOURCES="$CONTENTS/Resources"
DMG_STAGING="$BUILD_DIR/dmg_staging"
DMG_OUT="$BUILD_DIR/PARDUS-macOS.dmg"

rm -rf "$BUILD_DIR"
mkdir -p "$MACOS_BIN" "$FRAMEWORKS" "$RESOURCES" "$DMG_STAGING"

echo "==> 1. Compiling PARDUS binaries for macOS..."
GTK_CFLAGS=$(pkg-config --cflags gtk4)
GTK_LIBS=$(pkg-config --libs gtk4)
MEDIA_CFLAGS=$(pkg-config --cflags gtk4 libpng libjpeg fontconfig)
MEDIA_LIBS=$(pkg-config --libs gtk4 libpng libjpeg fontconfig)
ZMQ_CFLAGS=$(pkg-config --cflags libzmq libsodium)
ZMQ_LIBS=$(pkg-config --libs libzmq libsodium)

GNU_GCC=$(ls "$(brew --prefix)/bin/gcc-"[0-9]* | head -n 1)

"$GNU_GCC" -O2 $GTK_CFLAGS $ZMQ_CFLAGS -Wall -Wno-format-truncation \
   resources.c pardclient.c -o "$MACOS_BIN/pardclient" \
   $GTK_LIBS $ZMQ_LIBS -lm -lpng -ljpeg -lfontconfig \
   -framework AudioToolbox -framework CoreFoundation

"$GNU_GCC" -O2 -w $GTK_CFLAGS $ZMQ_CFLAGS \
   resources.c pcs-cli.c -o "$MACOS_BIN/pcs-cli" \
   $GTK_LIBS $ZMQ_LIBS -lm -lpng -ljpeg -lfontconfig \
   -framework AudioToolbox -framework CoreFoundation


clang -std=gnu2x -O2 -Wall $ZMQ_CFLAGS \
   pardserver.c -o "$MACOS_BIN/pardserver" \
   $ZMQ_LIBS -lm

clang -std=gnu2x -x objective-c -fobjc-arc -O2 -Wall $MEDIA_CFLAGS \
   pardcap.c -o "$MACOS_BIN/pardcap" \
   $MEDIA_LIBS -lm \
   -framework Foundation -framework AVFoundation -framework CoreMedia \
   -framework CoreVideo -framework IOKit -framework CoreFoundation -framework AudioToolbox


echo "==> 2. Generating pardus.icns and dove.icns..."
make_icns() {
    local src_png="$1"
    local out_icns="$2"
    local iconset="$BUILD_DIR/tmp.iconset"
    rm -rf "$iconset"
    mkdir -p "$iconset"
    for sz in 16 32 128 256 512; do
        sips -z $sz $sz "$src_png" --out "$iconset/icon_${sz}x${sz}.png" >/dev/null
        dbl=$((sz * 2))
        sips -z $dbl $dbl "$src_png" --out "$iconset/icon_${sz}x${sz}@2x.png" >/dev/null
    done
    iconutil -c icns "$iconset" -o "$out_icns"
    rm -rf "$iconset"
}

[ -f "pardclient.png" ] && make_icns "pardclient.png" "$RESOURCES/pardus.icns"
[ -f "dove.png" ]       && make_icns "dove.png"       "$RESOURCES/dove.icns"

echo "==> 3. Bundling Homebrew shared libraries (.dylib) into Contents/Frameworks..."
for bin in pardclient pcs-cli pardserver pardcap; do
    dylibbundler -od -b \
        -x "$MACOS_BIN/$bin" \
        -d "$FRAMEWORKS" \
        -p "@executable_path/../Frameworks"
done

echo "==> 4. Bundling GTK 4 GSettings schemas and GdkPixbuf loaders..."
GLIB_PREFIX=$(pkg-config --variable=prefix glib-2.0)
PIXBUF_VER=$(pkg-config --variable=gdk_pixbuf_binary_version gdk-pixbuf-2.0)
PIXBUF_PREFIX=$(pkg-config --variable=prefix gdk-pixbuf-2.0)

mkdir -p "$RESOURCES/share/glib-2.0/schemas"
cp -R "$GLIB_PREFIX/share/glib-2.0/schemas/"* "$RESOURCES/share/glib-2.0/schemas/"
glib-compile-schemas "$RESOURCES/share/glib-2.0/schemas"

mkdir -p "$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders"
cp "$PIXBUF_PREFIX/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders/"*.so "$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders/"
for loader in "$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders/"*.so; do
    dylibbundler -of -b \
        -x "$loader" \
        -d "$FRAMEWORKS" \
        -p "@executable_path/../Frameworks" >/dev/null 2>&1 || true
done
GDK_PIXBUF_MODULEDIR="$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders" \
    gdk-pixbuf-query-loaders > "$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders.cache"
# Rewrite absolute loader paths inside loaders.cache to relative bundle paths
sed -i '' "s|$RESOURCES/|@executable_path/../Resources/|g" \
    "$RESOURCES/lib/gdk-pixbuf-2.0/$PIXBUF_VER/loaders.cache"

echo "==> 5. Copying Manuals, AI Guides, PCS Samples, and Default Configs..."
cp "$ROOT_DIR/macos/Info.plist" "$CONTENTS/Info.plist"
printf "APPL????" > "$CONTENTS/PkgInfo"

[ -d "AI" ]           && cp -R "AI" "$RESOURCES/"
[ -d "PCS" ]          && cp -R "PCS" "$RESOURCES/"
[ -d "User_Manuals" ] && cp -R "User_Manuals" "$RESOURCES/"
for cfg in pardserver.config pardusers.txt server_identity.key pcscli.cfg; do
    [ -f "$cfg" ] && cp "$cfg" "$RESOURCES/"
done

echo "==> 6. Creating PARDUSLauncher wrapper (GUI + .dove Finder double-click)..."
cat > "$MACOS_BIN/PARDUSLauncher"  7. Ad-hoc signing PARDUS.app and building PARDUS-macOS.dmg..."
codesign --force --deep --sign - "$APP_DIR"

cp -R "$APP_DIR" "$DMG_STAGING/"
ln -s /Applications "$DMG_STAGING/Applications"
hdiutil create -volname "PARDUS" \
    -srcfolder "$DMG_STAGING" \
    -ov -format UDZO \
    "$DMG_OUT"

echo "==> Done! Bundle ready at: $DMG_OUT"
