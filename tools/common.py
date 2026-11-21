"""
common.py  (PC only)

Shared setup for every offline tool in tools/. Importing it puts on sys.path:
  - qcar/              the car code (lane_camera, lane_fusion, vehicle_control, utils)
  - tools/<folder>/    every tools subfolder, so tools can import each other by name
  - ../libraries/python  the Quanser libraries (hal, pal)
and gives the shared paths (ROOT, QCAR, LOGS, TOOLS) and helpers (load_vc, pop_opt).

Each tool starts with:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    import common
"""
import ast
import os
import sys

import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
QCAR = os.path.join(ROOT, 'qcar')
LOGS = os.path.join(ROOT, 'logs')
LIBS = os.path.join(ROOT, '..', 'libraries', 'python')

for _p in [LIBS] + [os.path.join(TOOLS, d) for d in sorted(os.listdir(TOOLS))
                    if os.path.isdir(os.path.join(TOOLS, d)) and not d.startswith(('_', '.'))] + [QCAR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)          # qcar/ inserted last, so it is searched first


def tool(folder, name):
    """Path of another tool script, e.g. tool('checks', 'shift_meas.py')."""
    return os.path.join(TOOLS, folder, name)


def load_vc():
    """RoutePath, remove_backward_points and the settings from qcar/vehicle_control.py."""
    src = open(os.path.join(QCAR, 'vehicle_control.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    keep = []
    names = {'RoutePath', 'remove_backward_points'}
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names:
            keep.append(node)
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            n = node.targets[0].id
            if n.startswith(('cs_', 'enc_scale', 'gps_front_ahead', 'profile_s', 'nodeSequence',
                             'startDelay')):
                keep.append(node)
    ns = {'np': np}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'vehicle_control.py', 'exec'), ns)
    return ns


def pop_opt(name, default, choices):
    """--name value from sys.argv (removed), or the default."""
    if name in sys.argv:
        k = sys.argv.index(name)
        val = sys.argv[k + 1]
        del sys.argv[k:k + 2]
        assert val in choices, f'{name} must be one of {choices}'
        return val
    return default
