#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$HOME/.local/bin" "$ROOT/runtime/backups"
/usr/bin/python3 - "$ROOT" <<'PY'
import datetime, re, shutil, sys, shlex
from pathlib import Path
root = Path(sys.argv[1])
path = Path.home() / '.bashrc'
text = path.read_text()
start, end = '# >>> Drone Navigation environment >>>', '# <<< Drone Navigation environment <<<'
text = re.sub(re.escape(start) + r'.*?' + re.escape(end) + r'\n?', '', text, flags=re.S)
for old in ('source ~/yolov8_ws/devel/setup.bash', 'source ~/vins_ws/devel/setup.bash'):
    text = text.replace('\n' + old + '\n', '\n# Replaced by the Drone Navigation environment: ' + old + '\n')
text = text.rstrip() + '\n\n' + start + '\n' + '''case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *) export PATH="$HOME/.local/bin:$PATH" ;;
esac
''' + 'source ' + shlex.quote(str(root / 'setup.bash')) + '\n' + end + '\n'
if text != path.read_text():
    backup = root / 'runtime/backups' / ('bashrc-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    shutil.copy2(str(path), str(backup))
    path.write_text(text)
    print('Shell backup:', backup)
for name, target in (('drone-nav', root / 'scripts/drone_nav.sh'),
                     ('drone-start', root / 'scripts/home/start_drone.sh'),
                     ('drone-check', root / 'scripts/doctor.sh')):
    command = Path.home() / '.local/bin' / name
    wrapper = '#!/usr/bin/env bash\nexec bash ' + shlex.quote(str(target)) + ' "$@"\n'
    if command.is_file() and not command.is_symlink() and command.read_text() == wrapper:
        continue
    if command.is_symlink() and command.resolve() == target:
        command.unlink()
    if command.exists() or command.is_symlink():
        raise SystemExit('Existing unrelated command: ' + str(command))
    command.write_text(wrapper)
    command.chmod(0o755)
print('Installed commands: drone-nav, drone-start, drone-check')
PY
