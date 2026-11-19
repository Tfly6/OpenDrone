"""Analytically labelled audit fixtures and same-input evidence ablations.

This validates the audit helpers on synthetic evidence, not physical simulator
truth, planner correctness, or superiority over another benchmarking tool.
"""
import argparse
from pathlib import Path

from .audit import contact_episodes, diagnostic_labels, ordered_visitation, write_csv, write_json
from .paths import artifacts_root


def fixtures():
    # Labels are explicitly specified before execution, rather than generated
    # by calling the evaluator under test. All distances are exact 1D examples.
    base = {"positions": [[0, 0, 0], [2, 0, 0], [4, 0, 0]], "times": [0, 1, 2],
            "goals": [[2, 0, 0], [4, 0, 0]], "native": "succeeded", "coverage": 1.0,
            "events": [], "expected_geometry": True, "expected_contacts": 0,
            "expected_labels": []}
    cases = []
    def add(name, **changes):
        cases.append(dict(base, id=name, **changes))
    add("normal_success")
    add("native_success_before_arrival", positions=[[0, 0, 0], [1, 0, 0]], times=[0, 1],
        expected_geometry=False, expected_labels=["native_success_without_geometric_fulfillment"])
    add("visited_but_native_failed", native="not_succeeded",
        expected_labels=["geometric_fulfillment_without_native_success"])
    add("final_goal_only", positions=[[0, 0, 0], [4, 0, 0]], times=[0, 2],
        expected_geometry=False, expected_labels=["native_success_without_geometric_fulfillment"])
    add("wrong_order", positions=[[4, 0, 0], [2, 0, 0]], times=[0, 1],
        expected_geometry=False, expected_labels=["native_success_without_geometric_fulfillment"])
    add("missing_states", positions=[], times=[], expected_geometry=None,
        expected_labels=["geometric_evidence_unknown"])
    add("missing_task", goals=[], expected_geometry=None,
        expected_labels=["geometric_evidence_unknown"])
    add("clock_reversed", times=[0, 2, 1], expected_geometry=None,
        expected_labels=["geometric_evidence_unknown"])
    add("low_reference_support", coverage=0.1, expected_labels=["reference_coverage_below_0.9"])
    add("contact_before_success", events=[{"time": 1, "other_collision": "tree"}],
        expected_contacts=1, expected_labels=["recorded_contact"])
    add("contact_after_window", events=[{"time": 3, "other_collision": "tree"}])
    add("ground_excluded", events=[{"time": 1, "other_collision": "ground_plane::link"}])
    add("continuous_contact_debounced", events=[{"time": t, "other_collision": "tree"} for t in (0, 0.4, 0.8, 1.2)],
        expected_contacts=1, expected_labels=["recorded_contact"])
    add("renewed_contact", events=[{"time": t, "other_collision": "tree"} for t in (0, 0.6)],
        expected_contacts=2, expected_labels=["recorded_contact"])
    add("different_obstacles", events=[{"time": 1, "other_collision": x} for x in ("tree", "cylinder")],
        expected_contacts=2, expected_labels=["recorded_contact"])
    add("inclusive_tolerance", positions=[[1.5, 0, 0], [3.5, 0, 0]], times=[0, 1])
    add("outside_tolerance", positions=[[1.499, 0, 0], [3.5, 0, 0]], times=[0, 1],
        expected_geometry=False, expected_labels=["native_success_without_geometric_fulfillment"])
    add("missing_contact_evidence", contact_unknown=True, expected_labels=["contact_evidence_unknown"])
    return cases


def validate(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    cases = fixtures()
    rows = []
    protocol_rows = []
    for case in cases:
        geometry = ordered_visitation(case["positions"], case["times"], case["goals"], 0.5)["complete"]
        contacts = len(contact_episodes(case["events"], 0, 2))
        contact = None if case.get("contact_unknown") else contacts > 0
        labels = diagnostic_labels(case["native"], geometry, contact, case["coverage"])
        passed = geometry == case["expected_geometry"] and contacts == case["expected_contacts"] and sorted(labels) == sorted(case["expected_labels"])
        rows.append({"id": case["id"], "passed": passed, "geometry": geometry, "contact_count": contacts,
                     "expected_labels": ";".join(case["expected_labels"]), "labels": ";".join(labels)})
        protocols = {
            "native_only": [],
            "geometry_contact": [x for x in labels if x in ("recorded_contact", "geometric_evidence_unknown", "contact_evidence_unknown")],
            "native_geometry_contact": [x for x in labels if x != "reference_coverage_below_0.9"],
            "full_evidence": labels,
        }
        for name, detected in protocols.items():
            expected = set(case["expected_labels"])
            actual = set(detected)
            protocol_rows.append({"id": case["id"], "protocol": name,
                                  "tp": len(expected & actual), "fp": len(actual - expected), "fn": len(expected - actual),
                                  "native_status": case["native"] if name != "geometry_contact" else "not_consumed",
                                  "geometric_completion": geometry if name != "native_only" else "not_consumed",
                                  "recorded_contact": contact if name != "native_only" else "not_consumed",
                                  "diagnoses": ";".join(detected)})
    write_json(output / "fixtures.json", {"evidence_level": "synthetic_analytical", "cases": cases})
    write_csv(output / "validation.csv", rows)
    write_csv(output / "protocol_ablation.csv", protocol_rows)
    summary = {"evidence_level": "synthetic_analytical", "cases": len(rows), "passed": sum(r["passed"] for r in rows),
               "limitation": "Channel-availability ablation on constructed examples; not an external-framework comparison or closed-loop validation."}
    write_json(output / "summary.json", summary)
    print(summary)
    if not all(r["passed"] for r in rows):
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(artifacts_root() / "revision_validation"))
    validate(parser.parse_args().output)
