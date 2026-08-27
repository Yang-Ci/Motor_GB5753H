#!/usr/bin/env python3
"""GB5753 / DaMiao CAN test tool entry point."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from gb5753_tool.app import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
