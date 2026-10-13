#!/usr/bin/env python3
"""Offline content/permission/link verification; does not run project code."""
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

root = Path(__file__).resolve().parent
manifest_path = root / 'metadata' / 'INTEGRITY.json'
if not manifest_path.is_file():
    sys.exit('Backup is not finalized: metadata/INTEGRITY.json is missing.')
manifest = json.loads(manifest_path.read_text())
expected = {'metadata/INTEGRITY.json'}
errors = []

for rec in manifest['files']:
    name = rec['path']
    expected.add(name)
    path = root / name
    if path.is_symlink() or not path.is_file():
        errors.append('Missing regular file: ' + name)
        continue
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    if h.hexdigest() != rec['sha256'] or path.stat().st_size != rec['size']:
        errors.append('Content mismatch: ' + name)
    if stat.S_IMODE(path.stat().st_mode) != rec['mode']:
        errors.append('Permissions mismatch: ' + name)

for rec in manifest['symlinks']:
    name = rec['path']
    expected.add(name)
    path = root / name
    if not path.is_symlink() or os.readlink(str(path)) != rec['target']:
        errors.append('Symlink mismatch: ' + name)
        continue
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError:
        errors.append('External symlink: ' + name)
    if not resolved.exists() and not rec.get('preexisting_missing_target', False):
        errors.append('Missing symlink target: ' + name)

actual = set()
for folder, dirs, files in os.walk(str(root), followlinks=False):
    # The project root's own Git database is intentionally outside the backup inventory.
    if Path(folder) == root:
        dirs[:] = [name for name in dirs if name != '.git']
    for name in files:
        actual.add(str((Path(folder) / name).relative_to(root)))
    for name in dirs:
        path = Path(folder) / name
        if path.is_symlink():
            actual.add(str(path.relative_to(root)))
for name in sorted(actual - expected):
    errors.append('Unexpected file: ' + name)
for name in sorted(expected - actual):
    errors.append('Missing manifest entry file: ' + name)

if errors:
    for error in errors:
        print(error)
    sys.exit('FAILED: {} error(s).'.format(len(errors)))
print('PASS: {} regular files, {} symlinks; contents, permissions and inventory verified.'.format(
    len(manifest['files']), len(manifest['symlinks'])))
