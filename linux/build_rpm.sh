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

# Create source tarball from current repo
tar -czf "$RPMTOP/SOURCES/pardus-${VERSION}.tar.gz" \
    --transform "s,^\.,pardus-${VERSION}," \
    --exclude='.git' --exclude='build_*' .

cat > "$RPMTOP/SPECS/pardus.spec"  %{buildroot}%{_datadir}/applications/pardus.desktop

printf '%s\n' \\
  '[Desktop Entry]' \\
  'Type=Application' \\
  'Name=Pardus PCS Runner' \\
  'Comment=Run compiled PARDUS .dove program' \\
  'Exec=/usr/bin/pcs-cli %%f' \\
  'Icon=application-x-pardus-dove' \\
  'Terminal=true' \\
  'MimeType=application/x-pardus-dove;' \\
  'NoDisplay=true' \\
  > %{buildroot}%{_datadir}/applications/pardus-dove.desktop

printf '%s\n' \\
  '<?xml version="1.0" encoding="UTF-8"?>' \\
  '<mime-info xmlns="http://freedesktop.org">' \\
  '  <mime-type type="application/x-pardus-dove">' \\
  '    <comment>Pardus Compiled PCS Program</comment>' \\
  '    <glob pattern="*.dove"/>' \\
  '    <icon name="application-x-pardus-dove"/>' \\
  '  </mime-type>' \\
  '</mime-info>' \\
  > %{buildroot}%{_datadir}/mime/packages/application-x-pardus-dove.xml

%files
%{_bindir}/pardserver
%{_bindir}/pardclient
%{_bindir}/pcs-cli
%{_bindir}/pardcap
%{_datadir}/pardus
%{_datadir}/applications/pardus.desktop
%{_datadir}/applications/pardus-dove.desktop
%{_datadir}/mime/packages/application-x-pardus-dove.xml
%{_datadir}/icons/hicolor/48x48/apps/pardus.png
%{_datadir}/icons/hicolor/96x96/mimetypes/application-x-pardus-dove.png
%config(noreplace) %{_sysconfdir}/pardus/pardserver.config
%config(noreplace) %{_sysconfdir}/pardus/pardusers.txt
%config(noreplace) %{_sysconfdir}/pardus/pcscli.cfg
%config(noreplace) %attr(0600,root,root) %{_sysconfdir}/pardus/server_identity.key
EOF

rpmbuild --define "_topdir $RPMTOP" -bb "$RPMTOP/SPECS/pardus.spec"
cp "$RPMTOP/RPMS/${ARCH}/"*.rpm "$BUILD_DIR/"
echo "==> Done! Fedora RPM ready at: $BUILD_DIR/"
