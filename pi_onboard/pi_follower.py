"""Compatibility entry point. Run from the repo root; see README.md."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_robot.server import main  # noqa: E402

if __name__ == "__main__":
    main()
