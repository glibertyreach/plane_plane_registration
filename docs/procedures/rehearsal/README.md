# Dress rehearsal of the registration capture procedure

Synthetic sensor and robot, the procedure's whole chain at full scale, run with
`docs/procedures/build/dress_rehearsal.py` on 2026-10-08 (the third run, after the
tools were changed for what the first two found; decision 18 of the review).

Setup: the example sensor placement of the procedure (looking down from 1.2 m), a
640 x 480 sensor with a 49.9 x 38.5 degree field, the default plan (235 poses, 47
held out), 5 frames per pose, clutter and fly-aways on, and a backlash emulation of
0.03 degrees and 0.1 mm: random sign per pose in sub-procedure A, constant in B.

| Result | A, ignoring backlash | B, minimizing backlash |
|---|---|---|
| Check verdict | 11 of 235 flagged, all "board image too small" | 14 of 235, same flag |
| Registration (rigid) | Success, 179 poses fitted | Success, 178 poses fitted |
| RMS residuals, fitted poses | 0.024 deg, 0.125 mm | 0.013 deg, 0.057 mm |
| Held-out poses, RMS residuals | 45 poses, 0.024 deg, 0.132 mm, within limits | 44 poses, 0.008 deg, 0.066 mm, within limits |
| Error against the true transform | 0.0045 deg, 0.014 mm | 0.029 deg, 0.285 mm |
| Relative transform between A and B | 0.026 deg, 0.274 mm | |

What this shows. The file layout, names, pose log and manifest of sections 2 and 7
go through every tool unchanged. The bootstrap of section 4 recovered the sensor
placement to 0.01 mm from six captures (normal spread 0.197). Random backlash (A)
widens the residuals but averages out of the transform; constant backlash (B)
leaves clean residuals but shifts the transform by the amount absorbed, which is
exactly the comparison the two sub-procedures are for. The poses flagged as too
small are at the 950 mm standoff with 30 degrees of tilt (and one at 750 mm), where
the synthetic sensor's grazing-incidence no-read leaves under 1,000 pixels; whether
the real sensor reads them is for the first session to show.

Times on this container: about 5 minutes to render each run of 1,175 frames, 55 s
for the check, 170 s for the registration with figures. Capture files: 2.84 GB per
run of synthetic frames (the real sensor's files are larger; the procedure's 10 GB
per 1,000 frames stands).

Files: `summary.md` (steps, exit codes, seconds), `plan_summary.txt`, `boot.json`,
`registration_A.json`, `registration_B.json`, `comparison.json`, `report_A.md`, and
in `figures/` the A-against-B comparison and run A's residual fields and per-pose
residuals.
