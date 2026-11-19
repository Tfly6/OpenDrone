# Results Data and Reproduction

`samples/tracking_state.csv` and `samples/tracking_reference.csv` are synthetic examples with ten samples each, intended for quickly verifying tracking-error calculations:

```bash
python3 verify_example.py
```

The accompanying paper data package contains per-trial results, measured signals, and configuration snapshots. The paths in the table below are relative to the extracted `data/` directory:

| File | Purpose |
| --- | --- |
| `revision_audit/trials.csv` | Per-trial status, tracking metrics, inclusion decisions, and mappings to original archives |
| `revision_audit/plan_to_result.csv` | Checks whether each planned trial has an actual configuration, bag, and report |
| `revision_audit/joint_outcomes.csv` | Joint counts of native success, geometric completion, and contact |
| `tracking_sensitivity.csv`, `coverage_sensitivity.csv` | Retained trials and metric changes under different coverage thresholds |
| `online_combinations.csv`, `planner_summary.csv` | Complete online combinations and planner summaries |
| `revision_review_20261116/` | Odometry/ground-truth and evaluation-window review for controlled cross-scenario comparisons |
| `worked_example/` | Raw states, reference points, contact data, and reproduced results for one actual flight |
| `configuration_records.zip` | Complete configuration, parameters, reports, and run-metadata snapshots for 646 attempts |
| `replay_source_snapshot.zip` | Implementation snapshot used to generate the archive audit results |

To reproduce the paper's values, set `OPENDRONE_DATA_DIR` to the data package's `data/` directory:

```bash
export OPENDRONE_DATA_DIR=/path/to/extracted_package/data
python3 summarize_planners.py
python3 verify_release.py
python3 build_evidence.py
```

The first two statistical reproduction scripts use only the Python standard library; see `requirements.txt` for dependencies required to generate figures. The complete documentation in the data package describes the signals, clocks, task windows, inclusion criteria, and column definitions.

To recompute results from raw rosbags, place the archives under `OPENDRONE_WORKSPACE/eval_runs/batches/<batch>/cases/<case>/`. See [README.md](README.md), under "Analysis and Reproduction," for instructions.
