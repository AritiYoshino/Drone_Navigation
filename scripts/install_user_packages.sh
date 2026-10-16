#!/usr/bin/env bash
# Install missing optional native/Python 2 tools into this project's sysroot.
# No dpkg database, system configuration or root-owned directory is modified.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
PACKAGE_CACHE="$PROJECT_ROOT/runtime/downloads/deb"
SYSROOT="$PROJECT_ROOT/runtime/sysroot"
GEOID_PARENT="$PROJECT_ROOT/runtime/local/share/GeographicLib"
mode=${1:---all}
case "$mode" in
  --all|--python-only|--geoid-only) ;;
  *) echo "Usage: $0 [--all|--python-only|--geoid-only]" >&2; exit 2 ;;
esac
mkdir -p "$PACKAGE_CACHE" "$SYSROOT" "$GEOID_PARENT/geoids"

if [[ "$mode" != --geoid-only ]]; then
  packages=(python-scipy python-decorator python-pyx python-opencv)
  if [[ "$mode" == --all ]]; then packages+=(texlive-latex-base); fi
  # apt resolves the packages missing from the real host; extraction alone
  # deliberately skips root-only maintainer scripts and system registration.
  simulation=$(LC_ALL=C apt-get -s --no-install-recommends install "${packages[@]}")
  mapfile -t resolved < <(printf '%s\n' "$simulation" |
    awk '$1 == "Inst" { print $2 "=" $3 }' | sed 's/=(/=/' )
  if ((${#resolved[@]})); then
    (cd -- "$PACKAGE_CACHE" && apt-get download "${resolved[@]}")
  fi
  while IFS= read -r -d '' package; do
    package_name=$(dpkg-deb -f "$package" Package)
    for requested in "${packages[@]}" "${resolved[@]}"; do
      if [[ "$package_name" == "${requested%%=*}" ]]; then
        dpkg-deb -x "$package" "$SYSROOT"
        break
      fi
    done
  done < <(find "$PACKAGE_CACHE" -maxdepth 1 -type f -name '*.deb' -print0)
  /usr/bin/python3 - "$PACKAGE_CACHE" "$SYSROOT/package_manifest.json" <<'PY'
import glob, hashlib, json, os, subprocess, sys
manifest = []
for filename in sorted(glob.glob(os.path.join(sys.argv[1], '*.deb'))):
    fields = subprocess.check_output(['dpkg-deb', '-f', filename,
        'Package', 'Version', 'Architecture']).decode().strip()
    with open(filename, 'rb') as stream:
        digest = hashlib.sha256(stream.read()).hexdigest()
    manifest.append({'archive': os.path.basename(filename), 'fields': fields,
                     'sha256': digest})
with open(sys.argv[2], 'w') as stream:
    json.dump(manifest, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
PY
  # Debian's TeX packages normally configure these paths in maintainer scripts.
  # Keep all TeX configuration, font indexes and generated formats local too.
  if [[ -f "$SYSROOT/usr/share/texlive/texmf-dist/web2c/texmf.cnf" ]]; then
    /usr/bin/python3 - "$SYSROOT" <<'PY'
import os, shutil, sys
prefix = sys.argv[1]
dist = prefix + '/usr/share/texlive/texmf-dist'
config = prefix + '/etc/texmf/web2c'
os.makedirs(config, exist_ok=True)
with open(dist + '/web2c/texmf.cnf') as stream:
    text = stream.read()
for old in ['/usr/share/texlive', '/usr/share/texmf', '/usr/local/share/texmf',
            '/var/lib/texmf', '/etc/texmf', '/var/cache/fonts']:
    text = text.replace(old, prefix + old)
text = text.replace('~/.texlive2017/texmf-var', prefix + '/var/lib/texmf')
text = text.replace('~/.texlive2017/texmf-config', prefix + '/etc/texmf')
text = text.replace('~/texmf', prefix + '/usr/local/share/texmf')
with open(config + '/texmf.cnf', 'w') as stream:
    stream.write(text)
language_dir = prefix + '/var/lib/texmf/tex/generic/config'
os.makedirs(language_dir, exist_ok=True)
for source, target in [('language.us', 'language.dat'),
                       ('language.us.def', 'language.def'),
                       ('language.us.lua', 'language.dat.lua')]:
    shutil.copyfile(dist + '/tex/generic/config/' + source, language_dir + '/' + target)
for tree in [dist, prefix + '/usr/share/texmf']:
    for directory, subdirs, files in os.walk(tree):
        for name in subdirs + files:
            path = os.path.join(directory, name)
            if not os.path.islink(path):
                continue
            target = os.readlink(path)
            if target.startswith(('/var/lib/texmf/', '/etc/texmf/')):
                new_target = prefix + target
                os.makedirs(os.path.dirname(new_target), exist_ok=True)
                os.unlink(path)
                os.symlink(os.path.relpath(new_target, directory), path)
PY
    export PATH="$SYSROOT/usr/bin:$PATH"
    export LD_LIBRARY_PATH="$SYSROOT/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export TEXMFCNF="$SYSROOT/etc/texmf/web2c"
    mkdir -p "$SYSROOT/var/lib/texmf/web2c/pdftex" "$SYSROOT/usr/local/share/texmf"
    mktexlsr "$SYSROOT/usr/share/texlive/texmf-dist" "$SYSROOT/usr/share/texmf" "$SYSROOT/var/lib/texmf"
    if [[ ! -s "$SYSROOT/var/lib/texmf/web2c/pdftex/interactive-format-ready" ]]; then
      (cd -- "$SYSROOT/var/lib/texmf/web2c/pdftex" &&
        pdftex -ini -etex -interaction=errorstopmode -jobname=latex -progname=latex latex.ini &&
        pdftex -ini -etex -interaction=errorstopmode -jobname=pdflatex -progname=pdflatex pdflatex.ini)
      mktexlsr "$SYSROOT/var/lib/texmf"
      touch "$SYSROOT/var/lib/texmf/web2c/pdftex/interactive-format-ready"
    fi
    if [[ ! -s "$SYSROOT/var/lib/texmf/web2c/tex/interactive-format-ready" ]]; then
      mkdir -p "$SYSROOT/var/lib/texmf/web2c/tex"
      (cd -- "$SYSROOT/var/lib/texmf/web2c/tex" &&
        tex -ini -interaction=errorstopmode -jobname=tex plain '\dump')
      mktexlsr "$SYSROOT/var/lib/texmf"
      touch "$SYSROOT/var/lib/texmf/web2c/tex/interactive-format-ready"
    fi
  fi
  PATH="$SYSROOT/usr/bin:$PATH" \
    LD_LIBRARY_PATH="$SYSROOT/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    PYTHONPATH="$SYSROOT/usr/lib/python2.7/dist-packages" /usr/bin/python2 -B - <<'PY'
import scipy, decorator, pyx
print('Python 2 local packages: scipy %s, decorator %s, pyx %s' %
      (scipy.__version__, decorator.__version__, pyx.__version__))
PY
fi

if [[ "$mode" != --python-only ]]; then
  geoid_archive="$PROJECT_ROOT/runtime/downloads/egm96-5.tar.bz2"
  if [[ ! -s "$GEOID_PARENT/geoids/egm96-5.pgm" ]]; then
    bundled_geoid="$PROJECT_ROOT/system_overrides/usr/share/GeographicLib/geoids/egm96-5.pgm"
    if [[ -s "$bundled_geoid" ]]; then
      cp -- "$bundled_geoid" "$GEOID_PARENT/geoids/egm96-5.pgm"
    else
    mkdir -p "$(dirname -- "$geoid_archive")"
    curl --fail --location --retry 2 --connect-timeout 15 \
      --output "$geoid_archive" \
      'https://downloads.sourceforge.net/project/geographiclib/geoids-distrib/egm96-5.tar.bz2'
    # GeographicLib's official archive contains the geoids/ subdirectory.
    tar -xjf "$geoid_archive" -C "$GEOID_PARENT" --no-same-owner
    fi
  fi
  GEOGRAPHICLIB_GEOID_PATH="$GEOID_PARENT/geoids" GeoidEval -n egm96-5 -l <<< '0 0'
  sha256sum "$GEOID_PARENT/geoids/egm96-5.pgm" > "$GEOID_PARENT/geoids/egm96-5.pgm.sha256"
fi

echo "Local dependencies: $SYSROOT"
echo "GeographicLib data: $GEOID_PARENT/geoids"
