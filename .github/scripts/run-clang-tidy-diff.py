#!/usr/bin/env python3
"""Run clang-tidy on diff lines only.

Like clang-format's diff-only check, but for clang-tidy.  Uses
clang-tidy-diff to check only changed lines against the compile
database, keeping runtime proportional to diff size, not codebase size.
"""
import os
import subprocess
import sys
from glob import glob


def main() -> int:
    # Find first compile_commands.json (any package — clang-tidy-diff
    # resolves paths relative to it)
    dbs = sorted(glob("/ws/build/*/compile_commands.json"))
    if not dbs:
        print("No compile_commands.json found — build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON first")
        return 1

    build_dir = os.path.dirname(dbs[0])

    # Get changed C++ files via git diff.  In CI, the base ref is available;
    # locally, diff against HEAD~1 or just check all workspace files.
    # Docker mounts can trigger "dubious ownership" — fix if needed
    subprocess.run(
        ["git", "config", "--global", "--add", "safe.directory", "/ws"],
        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    base = os.environ.get("GIT_BASE_REF", "HEAD~1")
    try:
        changed = subprocess.check_output(
            ["git", "diff", "--name-only", f"{base}...HEAD",
             "--", "**.cpp", "**.hpp", "**.h"],
            text=True, stderr=subprocess.PIPE,
        ).strip()
    except subprocess.CalledProcessError as e:
        print(f"git diff failed: {e.stderr.decode().strip()}")
        return 1

    if not changed:
        print("No C++ files changed")
        return 0

    # Filter to workspace src/, skip deleted files and third_party
    src_files = [f for f in changed.splitlines()
                 if f.startswith("src/") and "third_party/" not in f
                 and os.path.exists(f"/ws/{f}")]
    if not src_files:
        print("No workspace C++ files changed")
        return 0

    print(f"Checking {len(src_files)} changed file(s): {', '.join(src_files)}")

    # Run clang-tidy on those files only
    rc = subprocess.run(
        [
            "clang-tidy-19",
            "-p", build_dir,
            "--header-filter=/ws/src/.*",
            "--quiet",
        ] + [f"/ws/{f}" for f in src_files],
        check=False,
    ).returncode

    if rc != 0:
        print("clang-tidy found issues in changed lines")
    return rc


if __name__ == "__main__":
    sys.exit(main())
