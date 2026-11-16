# Professor demonstration

1. Read `results/CAMPAIGN.md` and the selected run's `acceptance.json`. Play the recorded actual seed-23 MP4 before presenting and review the latest GUI layout yourself. Automated browser inspection of that layout was blocked by depleted workspace credits; successful physics checks do not constitute visual review.
2. Start Docker Desktop. Run `./scripts/setup.sh` once, then `./scripts/run_demo.sh 11`.
3. Open the printed localhost noVNC URL. The left window is the actual Gazebo 3D world; the other window shows the annotated actual RGB stream and controller state. Software rendering can be slow on an 8 GB M1.
4. Point out the moving physical objects, randomized color order, three shapes, rails, Panda joints, and separate bins. Explain that the belt mesh is fixed while TrackController imposes physical contact-surface velocity.
5. Explain the color rule: seven successful red-blue pairs. Distractors may be labeled but are not eligible. A failed approach does not change the required color.
6. Show one contact, lift, transport, and release. Explain that a contact-gated joint approximates vacuum suction; it is not a friction-only parallel-jaw grasp or fluid simulation.
7. Watch the verified placement count. A successful command or a closed tool is insufficient; the object must be released and settled inside the correct bin.
8. After automatic completion, use `./scripts/generate_report.sh 11`. Show the real event log, measured wall and simulated times, failures, and generated plots. Do not replace missing or censored timings with estimates.
9. For a repeat, launch a fresh run with another seed. The harness archives prior outputs and holds an exclusive project-run lock.

The GUI is optional for headless validation. A graphical run can have a lower real-time factor than a headless run, so compare simulated manipulation durations and wall processing measurements separately.

Allow roughly five to ten minutes for a full standard-mode sequence on this 8 GB M1, plus startup. These are observed approximate presentation allowances, not guarantees. Seed 11 took 291.22 seconds headless; seed 23 took 563.02 seconds with rendering and recording. Keep the saved actual recording available if live software rendering is slow.
