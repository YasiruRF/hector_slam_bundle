# Report content

Everything for the "Application Results and Performance Evaluation" part of the report
and the presentation, from the Webots and real-robot runs on 2026-10-24.

- **[METRICS_REPORT.md](METRICS_REPORT.md)**: all results, with metric definitions, tables, figures and discussion. Start here.
- `figures/`: numbered figures used in the report (maps vs ground truth, error plots, RViz/Webots screenshots, ROS graphs, TF tree, real-robot maps and replays).
- `videos/`: short sped-up clips for the slides.
- `webots/W1_mapping … W6_fast_turns/`: raw data of each Webots run (maps, pose logs, evaluator output, every screenshot, full screen recording, logs).
- `real_robot/`: maps extracted from the real-robot rosbag, heading-slip analysis, offline replays (`replays/R*`).
- `scripts/`: the scripts that ran the tests and made the figures.

| Run | What |
| --- | --- |
| W1 | phase 1, Hector mapping, patrol loop |
| W2 | phase 2, Nav2 + AMCL, 6 goals on W1's map |
| W3 | phase 2, AMCL kidnapped robot: no help, global relocalisation, manual pose |
| W4 | Hector, large kidnap (2 m, 91°) |
| W5 | Hector, small kidnap (0.22 m, 10°) |
| W6 | Hector, 3× patrol speed (fast turns) |
