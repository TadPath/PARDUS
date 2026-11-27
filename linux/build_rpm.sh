#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

VERSION="1.0.0"
RELEASE="1"
ARCH="$(uname -m)"
BUILD_DIR="$ROOT_DIR/build_rpm"
RPMTOP="$BUILD_DIR/rpmbuild"

rm -rf "$BUILD_DIR"
mkdir -p "$RPMTOP"/{BUILD,RPMS,SOURCES,SPECS,SRPMS}

tar -czf "$RPMTOP/SOURCES/pardus-${VERSION}.tar.gz" \
    --transform "s,^\.,pardus-${VERSION}," \
    --exclude='.git' --exclude='build_*' .

printf '%s\n' \
  'Name:           pardus' \
  "Version:        ${VERSION}" \
  "Release:        ${RELEASE}" \
  'Summary:        Remote (network) control robotics and vision automation suite' \
  'License:        GPL-3.0-or-later' \
  'URL:            https://codeberg.org' \
  'Source0:        %{name}-%{version}.tar.gz' \
  '' \
  'BuildRequires:  gcc, pkgconf-pkg-config, gtk4-devel, zeromq-devel, libsodium-devel, libpng-devel, libjpeg-turbo-devel, fontconfig-devel, pulseaudio-libs-devel, libX11-devel' \
  'Requires:       gtk4, zeromq, libsodium, libpng, libjpeg-turbo, fontconfig, pulseaudio-libs, libX11' \
  '' \
  '%description' \
  'PARDUS Remote Robotics and Vision Automation Suite including pardclient' \
  '(GTK 4 GUI), pcs-cli (command-line runner), pardserver (network server),' \
  'pardcap (camera capture), manuals, AI references, and sample scripts.' \
  '' \
  '%prep' \
  '%autosetup' \
  '' \
  '%build' \
  'GTK_CFLAGS=$(pkg-config --cflags gtk4)' \
  'GTK_LIBS=$(pkg-config --libs gtk4)' \
  '' \
  'gcc -O2 pardserver.c -Wall -o pardserver -lzmq -lsodium -lm' \
  '' \
  'gcc -O2 $GTK_CFLAGS -Wall -Wno-format-truncation \' \
  '    resources.c pardclient.c -o pardclient \' \
  '    $GTK_LIBS -lm -lzmq -lpng -ljpeg -lfontconfig -lpulse-simple -lpulse -lX11' \
  '' \
  'gcc -O2 -w $GTK_CFLAGS \' \
  '    resources.c pcs-cli.c -o pcs-cli \' \
  '    $GTK_LIBS -lm -lzmq -lpng -ljpeg -lfontconfig -lpulse-simple -lpulse -lX11' \
  '' \
  'gcc -O2 $GTK_CFLAGS -Wall \' \
  '    pardcap.c -o pardcap \' \
  '    $GTK_LIBS -lm -lpng -ljpeg -lpulse-simple -lpulse -lfontconfig' \
  '' \
  '%install' \
  'install -Dm755 pardserver %{buildroot}%{_bindir}/pardserver' \
  'install -Dm755 pardclient %{buildroot}%{_bindir}/pardclient' \
  'install -Dm755 pcs-cli    %{buildroot}%{_bindir}/pcs-cli' \
  'install -Dm755 pardcap    %{buildroot}%{_bindir}/pardcap' \
  '' \
  'install -dm755 %{buildroot}%{_datadir}/pardus' \
  '[ -d "AI" ]           && cp -R "AI"           %{buildroot}%{_datadir}/pardus/' \
  '[ -d "PCS" ]          && cp -R "PCS"          %{buildroot}%{_datadir}/pardus/' \
  '[ -d "User_Manuals" ] && cp -R "User_Manuals" %{buildroot}%{_datadir}/pardus/' \
  '' \
  'install -dm755 %{buildroot}%{_sysconfdir}/pardus' \
  'for cfg in pardserver.config pardusers.txt pcscli.cfg; do' \
  '    [ -f "$cfg" ] && install -Dm644 "$cfg" %{buildroot}%{_sysconfdir}/pardus/$cfg' \
  'done' \
  '[ -f "server_identity.key" ] && install -Dm600 "server_identity.key" %{buildroot}%{_sysconfdir}/pardus/server_identity.key' \
  '' \
  '[ -f "pardclient.png" ] && install -Dm644 "pardclient.png" %{buildroot}%{_datadir}/icons/hicolor/48x48/apps/pardus.png' \
  '[ -f "dove.png" ]       && install -Dm644 "dove.png"       %{buildroot}%{_datadir}/icons/hicolor/96x96/mimetypes/application-x-pardus-dove.png' \
  '' \
  'install -dm755 %{buildroot}%{_datadir}/applications %{buildroot}%{_datadir}/mime/packages' \
  '' \
  'printf "%s\n" "[Desktop Entry]" "Type=Application" "Name=PardClient" "Comment=Pard GTK4 Client" "Exec=/usr/bin/pardclient" "Icon=pardus" "Terminal=false" "StartupWMClass=pardclient" "Categories=Science;Engineering;Network;Utility;" > %{buildroot}%{_datadir}/applications/pardus.desktop' \
  '' \
  'printf "%s\n" "[Desktop Entry]" "Type=Application" "Name=Pardus PCS Runner" "Comment=Run compiled PARDUS .dove program" "Exec=/usr/bin/pcs-cli %%f" "Icon=application-x-pardus-dove" "Terminal=true" "MimeType=application/x-pardus-dove;" "NoDisplay=true" > %{buildroot}%{_datadir}/applications/pardus-dove.desktop' \
  '' \
  'printf "%s\n" "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" "<mime-info xmlns=\"http://freedesktop.org\">" "  <mime-type type=\"application/x-pardus-dove\">" "    <comment>Pardus Compiled PCS Program</comment>" "    <glob pattern=\"*.dove\"/>" "    <icon name=\"application-x-pardus-dove\"/>" "  </mime-type>" "</mime-info>" > %{buildroot}%{_datadir}/mime/packages/application-x-pardus-dove.xml' \
  '' \
  '%files' \
  '%{_bindir}/pardserver' \
  '%{_bindir}/pardclient' \
  '%{_bindir}/pcs-cli' \
  '%{_bindir}/pardcap' \
  '%{_datadir}/pardus' \
  '%{_datadir}/applications/pardus.desktop' \
  '%{_datadir}/applications/pardus-dove.desktop' \
  '%{_datadir}/mime/packages/application-x-pardus-dove.xml' \
  '%{_datadir}/icons/hicolor/48x48/apps/pardus.png' \
  '%{_datadir}/icons/hicolor/96x96/mimetypes/application-x-pardus-dove.png' \
  '%config(noreplace) %{_sysconfdir}/pardus/pardserver.config' \
  '%config(noreplace) %{_sysconfdir}/pardus/pardusers.txt' \
  '%config(noreplace) %{_sysconfdir}/pardus/pcscli.cfg' \
  '%config(noreplace) %attr(0600,root,root) %{_sysconfdir}/pardus/server_identity.key' \
  > "$RPMTOP/SPECS/pardus.spec"

rpmbuild --define "_topdir $RPMTOP" -bb "$RPMTOP/SPECS/pardus.spec"
cp "$RPMTOP/RPMS/${ARCH}/"*.rpm "$BUILD_DIR/"
echo "==> Done! Fedora RPM ready at: $BUILD_DIR/"
