#!/usr/bin/env python3
"""Moved to tools/hcc_v2v/scripts/hcc_monitor.py (it is useful for in-car
V2V testing, not just HIL). This stub forwards for old docs/scripts."""
import os
import runpy
import sys

_target = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "..", "hcc_v2v", "scripts", "hcc_monitor.py")

if __name__ == "__main__":
  sys.argv[0] = _target
  runpy.run_path(_target, run_name="__main__")
