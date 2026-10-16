#!/usr/bin/env python3
"""Restore executable scripts in the disposable runtime workspace copies."""
import argparse
import json
import os
from pathlib import Path
import stat


WORKSPACES = Path(__file__).resolve().parent.parent / "runtime/workspaces"
SKIP_DIRECTORIES = {"build", "devel", "install", ".git", ".catkin_tools", "CMakeFiles", "__pycache__"}


def prepare(workspace):
    changes = []
    checked = 0
    for directory, subdirectories, filenames in os.walk(str(workspace), followlinks=False):
        subdirectories[:] = sorted(name for name in subdirectories
            if name not in SKIP_DIRECTORIES and not (Path(directory) / name).is_symlink())
        for name in sorted(filenames):
            path = Path(directory) / name
            if path.suffix not in (".py", ".sh") or path.is_symlink() or not path.is_file():
                continue
            contents = path.read_bytes()
            if not contents.startswith(b"#!"):
                continue
            checked += 1
            normalized = contents.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            mode = stat.S_IMODE(path.stat().st_mode)
            executable_mode = mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
            if normalized != contents:
                path.write_bytes(normalized)
            if executable_mode != mode:
                path.chmod(executable_mode)
            if normalized != contents or executable_mode != mode:
                changes.append({"script": str(path.relative_to(WORKSPACES)),
                    "normalized_line_endings": normalized != contents,
                    "restored_executable_bits": executable_mode != mode})
    return {"workspace": str(workspace), "scripts_checked": checked, "changes": changes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspaces", nargs="*", help="workspace names inside runtime/workspaces (default: all)")
    args = parser.parse_args()
    if not WORKSPACES.is_dir():
        parser.error("runtime/workspaces has not been prepared")
    roots = [WORKSPACES / name for name in args.workspaces] if args.workspaces else sorted(WORKSPACES.iterdir())
    reports = []
    for workspace in roots:
        if workspace.is_symlink():
            continue
        if WORKSPACES.resolve() not in workspace.resolve().parents or not workspace.is_dir():
            parser.error("workspace must be a directory inside runtime/workspaces: %s" % workspace)
        reports.append(prepare(workspace))
    print(json.dumps(reports, indent=2))


if __name__ == "__main__":
    main()
