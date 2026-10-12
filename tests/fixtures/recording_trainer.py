"""Test-only trainer stub: capture commands without loading data or training."""
import json
import os
from pathlib import Path
import sys


def main():
    args = sys.argv[1:]
    label = args[args.index("--label-column") + 1]
    with Path(os.environ["STUB_CAPTURE_PATH"]).open("a", encoding="utf-8") as target:
        target.write(json.dumps({"args": args, "cwd": os.getcwd()}, ensure_ascii=True) + "\n")
    if label == os.environ.get("STUB_FAIL_LABEL"):
        return 23
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
