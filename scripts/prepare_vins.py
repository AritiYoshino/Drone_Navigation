#!/usr/bin/python3
"""Apply migration changes to the disposable build copy only."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
workspace = root / 'runtime/workspaces/vinsf_gpu_ws'
for path in workspace.glob('src/**/CMakeLists.txt'):
    text = path.read_text()
    updated = re.sub(r'^set\(OpenCV_DIR ".*?"\)\n', '', text, flags=re.M)
    updated = re.sub(r'^set\(Ceres_DIR ".*?"\)\n', '', updated, flags=re.M)
    updated = re.sub(r'^set\(CMAKE_INSTALL_RPATH ".*?"\)\n', '', updated, flags=re.M)
    if re.search(r'find_package\(OpenCV\b', updated) and 'DRONE_OPENCV_INCLUDE_ORDER' not in updated:
        updated = re.sub(r'find_package\(OpenCV\b[^)]*\)', lambda match: match.group(0) +
            '\n# DRONE_OPENCV_INCLUDE_ORDER: keep /usr/local/include from selecting another OpenCV.\n'
            'set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON)\n'
            'include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})', updated, count=1)
    updated = updated.replace(' -gencode arch=compute_72,code=sm_72', ' -gencode arch=compute_${DRONE_CUDA_ARCH},code=sm_${DRONE_CUDA_ARCH}')
    if 'project(vins)' in updated and 'set(DRONE_CUDA_ARCH' not in updated:
        updated = updated.replace('project(vins)', 'project(vins)\nset(DRONE_CUDA_ARCH "72" CACHE STRING "Jetson CUDA compute architecture")')
    if updated != text:
        path.write_text(updated)
print('Prepared VINS build copy:', workspace)
