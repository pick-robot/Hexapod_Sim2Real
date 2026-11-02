#!/usr/bin/env python3
"""Plot the main tethered deployment signals from one comprehensive CSV."""

import csv
import os
from pathlib import Path

import matplotlib

# Save figures without requiring a desktop/display server.
matplotlib.use("Agg")
matplotlib.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "font.size": 11,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "legend.frameon": False,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    }
)
import matplotlib.pyplot as plt
import numpy as np


# Set the absolute path of the CSV to plot here. No command-line argument is
# required. The PNG is saved beside this CSV as ``<csv_stem>_plot.png``.
CSV_PATH = Path("/home/lgl/Hexapod_Sim2Real/deploy/deploy_real/data_logs/deploy_real_hexapod_tethered_plot_torque_data_estimate/deploy_real_hexapod_tethered_plot_torque_data_estimate_20261031_203522_137343/deploy_real_hexapod_tethered_plot_torque_data_estimate_trajectory_20261031_203522_137343.csv"
)

FIGURE_SIZE = (18, 12)
OUTPUT_DPI = 300

REQUIRED_COLUMNS = (
    "timestamp_ns",
    "force_raw_n",
    "force_reference_n",
    "velocity_command_0",
    "torque_command_issued_nm",
    "torque_actual_nm",
)


def _read_csv(csv_path: Path) -> dict[str, np.ndarray]:
    if not csv_path.is_absolute():
        raise ValueError(f"CSV_PATH must be an absolute path: {csv_path}")
    if not csv_path.is_file():
        raise FileNotFoundError(
            f"CSV file does not exist: {csv_path}\n"
            "Edit CSV_PATH at the top of this script."
        )

    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError(
                f"CSV is missing required columns: {', '.join(missing)}"
            )
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV contains a header but no data rows: {csv_path}")

    columns: dict[str, np.ndarray] = {}
    for name in REQUIRED_COLUMNS:
        values = []
        for row_number, row in enumerate(rows, start=2):
            try:
                value = float(row[name])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid numeric value in column {name!r}, row {row_number}"
                ) from exc
            if not np.isfinite(value):
                raise ValueError(
                    f"Non-finite value in column {name!r}, row {row_number}"
                )
            values.append(value)
        columns[name] = np.asarray(values, dtype=np.float64)

    timestamp_ns = columns["timestamp_ns"]
    time_s = (timestamp_ns - timestamp_ns[0]) * 1.0e-9
    if np.any(np.diff(time_s) < 0.0):
        raise ValueError("timestamp_ns must be monotonically non-decreasing")
    columns["time_s"] = time_s
    return columns


def create_plot(csv_path: Path) -> Path:
    """Create the three-panel plot and return the output PNG path."""

    csv_path = csv_path.expanduser().resolve()
    output_path = csv_path.with_name(f"{csv_path.stem}_plot.png")
    if not os.access(csv_path.parent, os.W_OK):
        raise PermissionError(
            "CSV directory is not writable, so the PNG cannot be saved beside it: "
            f"{csv_path.parent}\n"
            "This is usually an old sudo-created log directory. Change its "
            "ownership to your normal user, then run this script again."
        )
    data = _read_csv(csv_path)
    time_s = data["time_s"]

    figure, axes = plt.subplots(
        3,
        1,
        figsize=FIGURE_SIZE,
        sharex=True,
        constrained_layout=True,
    )
    figure.suptitle(csv_path.stem, fontsize=16)

    axes[0].plot(
        time_s,
        data["force_raw_n"],
        label="Measured tension",
        color="tab:blue",
        linewidth=1.4,
    )
    axes[0].plot(
        time_s,
        data["force_reference_n"],
        label="Reference tension",
        color="tab:orange",
        linewidth=1.4,
        linestyle="--",
    )
    axes[0].set_ylabel("Tension (N)")
    axes[0].legend(loc="best")

    axes[1].plot(
        time_s,
        data["velocity_command_0"],
        label="Cmd x",
        color="tab:green",
        linewidth=1.4,
    )
    axes[1].set_ylabel("Command x (m/s)")
    axes[1].legend(loc="best")

    axes[2].plot(
        time_s,
        data["torque_command_issued_nm"],
        label="Commanded motor torque",
        color="tab:red",
        linewidth=1.4,
    )
    axes[2].plot(
        time_s,
        data["torque_actual_nm"],
        label="Measured motor torque",
        color="tab:purple",
        linewidth=1.4,
        linestyle="--",
    )
    axes[2].set_ylabel("Motor torque (Nm)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best")

    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.margins(x=0.0)

    figure.savefig(output_path, dpi=OUTPUT_DPI, bbox_inches="tight")
    plt.close(figure)
    return output_path


def main() -> None:
    output_path = create_plot(CSV_PATH)
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()
