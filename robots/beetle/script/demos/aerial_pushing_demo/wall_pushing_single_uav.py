#!/usr/bin/env python3
"""Single-UAV entry point for the shared wall-pushing state machine."""

import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from wall_pushing_formation import main


if __name__ == '__main__':
    main(single_uav_mode=True)
