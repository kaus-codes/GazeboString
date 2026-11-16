# Verified standard-mode campaign

Named final-controller standard-mode runs; development attempts are listed separately and are not hidden or combined into this rate. GUI/recording adds overhead.

| Seed | Placements / attempts | Failed | Incorrect | Wall s | Sim s | RTF | Pixel processing mean ms |
|---|---|---|---|---|---|---|---|
| 11 | 14 / 14 | 0 | 0 | 291.221 | 222.098 | 0.763 | 9.526 |
| 23 | 14 / 14 | 0 | 0 | 563.017 | 249.910 | 0.444 | 13.734 |
| 47 | 14 / 16 | 2 | 0 | 443.393 | 273.162 | 0.616 | 10.789 |

Each accepted run independently passed all 23 checks in its acceptance.json, including fresh final retention, complete bin inventory, continuous belt odometry, real contact evidence, exact alternation, and clean owned-process shutdown.

Seed 23 enables GUI software rendering and X11 recording. Seeds 11 and 47 are headless. These are different runtime loads, not controlled rendering benchmarks.

All per-operation timings, pair durations, mean/median/min/max/population standard deviation, calibration errors, and camera evaluation are in each aggregate.json. The CSV exports and plots are generated exclusively from recorded events.

Development failures are preserved. See campaign.json historical_summaries and docs/TROUBLESHOOTING.md; these three final runs do not imply that every development run passed.

Clutter mode and a clean Dockerfile rebuild are unverified. The latest noVNC layout could not be inspected through the browser because automatic approval review could not run with depleted workspace credits. The MP4 is an actual X11 recording; its codec, dimensions, duration and file integrity were checked.
