#!/usr/bin/env python3
"""Create a relocatable run config without changing the archived calibration."""

import argparse
import json
import os
import re
import tempfile


def prepare(source, runtime):
    source = os.path.abspath(source)
    runtime = os.path.abspath(runtime)
    with open(source) as stream:
        content = stream.read()
    values = {
        "output_path": os.path.join(runtime, "vins_output"),
        "pose_graph_save_path": os.path.join(runtime, "pose_graph") + os.sep,
    }
    for key in ("cam0_calib", "cam1_calib"):
        match = re.search(r"^" + key + r":\s*[\"']?([^\"'\n#]+)", content, re.M)
        if not match:
            raise ValueError("Missing calibration entry: " + key)
        calibration = os.path.join(os.path.dirname(source), match.group(1).strip())
        if not os.path.isfile(calibration):
            raise ValueError("Missing camera calibration: " + calibration)
        # VINS always joins these values with the config's directory.
        values[key] = os.path.relpath(calibration, runtime)
    os.makedirs(runtime, exist_ok=True)
    for key in ("output_path", "pose_graph_save_path"):
        os.makedirs(values[key], exist_ok=True)
    for key, value in values.items():
        content, count = re.subn(r"^" + key + r":.*$", key + ": " + json.dumps(value),
                                 content, flags=re.M)
        if count != 1:
            raise ValueError("Expected one configuration entry: " + key)
    target = os.path.join(runtime, "vins_runtime.yaml")
    handle, temporary = tempfile.mkstemp(prefix=".vins-", suffix=".yaml", dir=runtime)
    try:
        with os.fdopen(handle, "w") as stream:
            stream.write(content)
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return target


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("runtime")
    arguments = parser.parse_args()
    print(prepare(arguments.source, arguments.runtime))
