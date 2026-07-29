"""Test runner. BLENDER_ADDON_PLAN.md Sec 12.

Two ways to run, both of which must pass:

    blender --background --python so100_builder/tests/run.py     # in Blender
    python3 so100_builder/tests/run.py                           # bare CPython

The bare-CPython path matters because ``core/`` and ``kinematics/`` are
deliberately free of ``bpy``: if a ``bpy`` import ever leaks into them this
run fails immediately rather than at addon-install time on a user's Windows
machine. Blender is not installed on the development machine, so bare
CPython is the routine path.

Exits non-zero on failure so it can be wired straight into CI.
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ADDON_DIR = os.path.dirname(HERE)
REPO_DIR = os.path.dirname(ADDON_DIR)

if REPO_DIR not in sys.path:
    sys.path.insert(0, REPO_DIR)


def main():
    running_in_blender = "bpy" in sys.modules
    try:
        import bpy  # noqa: F401
        running_in_blender = True
    except ImportError:
        pass

    print("=" * 70)
    print("so100_builder tests -- %s (Python %s)"
          % ("inside Blender" if running_in_blender else "bare CPython",
             ".".join(str(v) for v in sys.version_info[:3])))
    print("=" * 70)

    suite = unittest.defaultTestLoader.discover(
        start_dir=HERE, pattern="test_*.py", top_level_dir=REPO_DIR
    )
    result = unittest.TextTestRunner(verbosity=2).run(suite)

    code = 0 if result.wasSuccessful() else 1
    if running_in_blender:
        # `blender --background --python` ignores a plain return, so the exit
        # code has to be forced or CI will always see success.
        sys.exit(code)
    return code


if __name__ == "__main__":
    sys.exit(main())
