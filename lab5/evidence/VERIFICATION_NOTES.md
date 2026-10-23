# Packaging verification — 2026-10-23

006 clean build passed with the exact README CMake options, starting with
only the existing environment prefix (not the original course workspace).
rospack resolved 006's clean workspace; ldd found no missing dependencies.
The launch command supplied the required output_dir and produced 101 good
matches / 79 inliers. The negative launch-argument check intentionally
confirmed omission of output_dir is invalid; this is not a test failure.

SIFT and LK each processed 12 original images / 11 pairs with exact input
stamps, rejected one duplicate and one invalid encoding, then recovered.
No full-bag re-run is claimed for 006: the included complete-bag records
are the existing 005 records. No algorithm .cpp files changed in 006.
The old report is preserved verbatim; packaging changes do not change results.

Reproduce the clean build using feature_tracking/README.md. The exact smoke
harness is ../../validation/check_runtime.py (outside this small submission
collection, alongside its ws and SOURCE_BEFORE.json).
To repeat the same harness, copy validation into a NEW sibling validation
directory, copy 005's SOURCE_BEFORE.json, clean-build ws from the submission,
activate/source that ws, and run check_runtime.py. It refuses to overwrite
an existing runtime_r2 output directory and shuts down its private roscore.
Do not delete previous evidence just to rerun a smoke test.

Existing submission worktree branch: lab3-lab4-submission; remote:
https://github.com/MIT-SPARK/VNAV-labs.git (official upstream).
Not confirmed as a student submission target. No Git state was changed.
