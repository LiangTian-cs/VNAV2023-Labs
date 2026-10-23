# VNAV2023 Lab5 submission collection

Package: feature_tracking/ (ROS package name lab_5).
Run/build commands: feature_tracking/README.md.
Report: report/autumn_lab5_report.pdf.
Theory: theory/D1_D2_DERIVATIONS.md and NUMERICAL_CHECK.json.
Representative figures: figures/.
Full official-sequence summaries and input hashes: evidence/.

Implemented/verified: C++ SIFT pair matching, SIFT sequence tracking and LK
tracking on both official bags, RANSAC statistics and explicit F-estimation
failure records. Evidence from 005 is historical full-sequence evidence;
006 separately verifies clean build/runtime after packaging changes.

Limitations: SURF skipped (nonfree unavailable). ORB/FAST+BRIEF/Farneback
comparisons were supplementary Python experiments, not implementations of
the unused C++ course templates retained in src/. No claim that every MIT
handout optional extension is complete. F-estimation failures are retained
as geometry_valid=0, not silently removed or counted as successful geometry.

No datasets, environments, build products, credentials or Git metadata are
included. Dataset SHA256 values are in evidence/INPUTS.json. Full data and
long logs remain on the server; this collection is intentionally small.
Submission target confirmed by the student: LiangTian-cs/VNAV2023-Labs,
main branch, lab5/ directory. Historical packaging checks describe the
pre-submission state; original 005 and 006 snapshots remain unchanged.
