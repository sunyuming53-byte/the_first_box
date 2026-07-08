#!/usr/bin/env python3
"""Convert CSV waypoints to YAML inspection points format."""
import argparse
import csv
import yaml


def main() -> None:
    p = argparse.ArgumentParser(description="Convert CSV waypoints to YAML")
    p.add_argument("input_csv", help="Path to CSV waypoint file")
    p.add_argument("output_yaml", help="Path to output YAML file")
    args = p.parse_args()

    waypoints: list[dict] = []
    with open(args.input_csv, newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            name = (
                f'wp_{int(row["seq"]):03d}'
                if row.get("seq")
                else f"wp_{i + 1:03d}"
            )
            wp: dict = {
                "name": name,
                "pose": {
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row.get("z", 0)),
                    "qx": float(row["qx"]),
                    "qy": float(row["qy"]),
                    "qz": float(row["qz"]),
                    "qw": float(row["qw"]),
                },
            }
            task = row.get("task", "none")
            if task and task != "none":
                wp["task"] = task
            waypoints.append(wp)

    with open(args.output_yaml, "w") as f:
        yaml.dump(
            {"waypoints": waypoints},
            f,
            default_flow_style=False,
            sort_keys=False,
        )

    print(f"Converted {len(waypoints)} waypoints to {args.output_yaml}")


if __name__ == "__main__":
    main()
