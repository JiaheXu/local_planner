"""Test the package runtime using only its source tree and installed ROS APIs."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
