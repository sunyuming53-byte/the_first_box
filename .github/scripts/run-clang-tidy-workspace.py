#!/usr/bin/env python3
"""Run clang-tidy on workspace source files only.

Filters each compile_commands.json to entries under /ws/src/,
excluding external dependencies (gtest, system headers, etc.).
"""
import json
import os
import subprocess
import sys
from glob import glob


def main() -> int:
    failed = False

    for db_path in sorted(glob("/ws/build/*/compile_commands.json")):
        with open(db_path) as f:
            entries = json.load(f)

        # Keep only workspace source files
        src_files = [e["file"] for e in entries if "/ws/src/" in e["file"]]
        if not src_files:
            continue

        build_dir = os.path.dirname(db_path)
        pkg = os.path.basename(build_dir)

        print(f"Checking {pkg} ({len(src_files)} files)...")

        rc = subprocess.run(
            [
                "run-clang-tidy-19",
                "-p", build_dir,
                "-header-filter=/ws/src/.*",
                "-quiet",
            ] + src_files,
            check=False,
        ).returncode

        if rc != 0:
            print(f"FAILED: {pkg}")
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
