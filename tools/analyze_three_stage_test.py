#!/usr/bin/env python3
"""Analyze and plot the completed three-stage temperature test."""

import argparse
import csv
import json
import math
from html import escape
from pathlib import Path


PHASE1 = "stage1_room_to_40_hold"
PHASE2 = "stage2_40_to_70_hold"
LID = "lid_open_10s"


def sustained_index(values, target, tolerance):
    for index in range(len(values)):
        if all(abs(value - target) <= tolerance for value in values[index:]):
            return index
    return None


def stage_metrics(rows, target, hold_seconds, stage_name):
    values = [row["temperature_c"] for row in rows]
    elapsed = [row["elapsed_s"] for row in rows]
    stage_elapsed = [row["stage_elapsed_s"] for row in rows]
    first_cross = next((index for index, value in enumerate(values) if value >= target), None)
    strict_index = sustained_index(values, target, 0.2)
    hold_index = sustained_index(values, target, 0.5)
    tail = values[-min(len(values), round(hold_seconds)) :]
    result = {
        "stage": stage_name,
        "target_c": target,
        "samples": len(rows),
        "stage_start_elapsed_s": float(stage_elapsed[0]),
        "first_crossing_elapsed_s": None if first_cross is None else float(stage_elapsed[first_cross]),
        "first_crossing_global_elapsed_s": None if first_cross is None else float(elapsed[first_cross]),
        "settling_band": "target +/- 0.2 C, remains in band through stage end",
        "settling_time_s": None if strict_index is None else float(stage_elapsed[strict_index]),
        "settling_global_elapsed_s": None if strict_index is None else float(elapsed[strict_index]),
        "control_band_time_s": None if hold_index is None else float(stage_elapsed[hold_index]),
        "control_band": "target +/- 0.5 C, remains in band through stage end",
        "peak_temperature_c": float(max(values)),
        "minimum_temperature_c": float(min(values)),
        "overshoot_c": float(max(values) - target),
        "tail_mean_c": float(sum(tail) / len(tail)),
        "tail_min_c": float(min(tail)),
        "tail_max_c": float(max(tail)),
        "tail_rmse_c": float(math.sqrt(sum((value - target) ** 2 for value in tail) / len(tail))),
    }
    return result


def event_time(events, name):
    return next(event["elapsed_s"] for event in events if event["event"] == name)


def svg_polyline(points, color, width=2.0):
    return '<polyline fill="none" stroke="{}" stroke-width="{}" points="{}"/>'.format(
        color, width, " ".join(f"{x:.1f},{y:.1f}" for x, y in points))


def overview_svg(rows, events, metrics):
    width, height = 1280, 900
    left, right = 92, 28
    plot_width = width - left - right
    panel_top = [100, 370, 610]
    panel_height = [210, 150, 150]
    x_max = max(row["elapsed_s"] for row in rows) + 10
    y_ranges = [(28, 73), (0, 105), (0, 40)]
    labels = ["Temperature (C)", "Heater output (%)", "Humidity (%)"]
    colors = {"command_wait": "#64748b", PHASE1: "#2563eb", PHASE2: "#d97706",
              LID: "#dc2626", "stopped": "#475569"}

    def sx(value):
        return left + value / x_max * plot_width

    def sy(value, panel):
        low, high = y_ranges[panel]
        return panel_top[panel] + panel_height[panel] - (value - low) / (high - low) * panel_height[panel]

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           '<style>text{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;fill:#1f2937} .grid{stroke:#e5e7eb;stroke-width:1} .axis{stroke:#374151;stroke-width:1} .small{font-size:12px} .label{font-size:14px} .title{font-size:22px;font-weight:600}</style>',
           '<text x="92" y="38" class="title">Three-stage temperature test and 10-second lid-open response</text>',
           f'<text x="92" y="63" class="small">40 C settling +/-0.2 C: {metrics["stage1_40c"]["settling_time_s"]:.1f}s | overshoot: {metrics["stage1_40c"]["overshoot_c"]:.2f} C</text>',
           f'<text x="520" y="63" class="small">70 C settling +/-0.2 C: {metrics["stage2_70c"]["settling_time_s"]:.1f}s | overshoot: {metrics["stage2_70c"]["overshoot_c"]:.2f} C</text>']
    for panel in range(3):
        top = panel_top[panel]
        ph = panel_height[panel]
        low, high = y_ranges[panel]
        out.append(f'<rect x="{left}" y="{top}" width="{plot_width}" height="{ph}" fill="none" class="axis"/>')
        for tick in np_ticks(low, high, 5):
            y = sy(tick, panel)
            out.append(f'<line x1="{left}" x2="{left + plot_width}" y1="{y:.1f}" y2="{y:.1f}" class="grid"/>')
            out.append(f'<text x="{left - 12}" y="{y + 4:.1f}" text-anchor="end" class="small">{tick:g}</text>')
        out.append(f'<text x="20" y="{top + ph / 2:.1f}" transform="rotate(-90 20 {top + ph / 2:.1f})" text-anchor="middle" class="label">{labels[panel]}</text>')
    for tick in np_ticks(0, x_max, 8):
        x = sx(tick)
        out.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="100" y2="760" class="grid"/>')
        out.append(f'<text x="{x:.1f}" y="785" text-anchor="middle" class="small">{tick:g}</text>')
    out.append('<text x="650" y="820" text-anchor="middle" class="label">Elapsed time (s)</text>')

    lid_start = event_time(events, "lid_open_started")
    lid_end = event_time(events, "lid_window_completed")
    out.append(f'<rect x="{sx(lid_start):.1f}" y="100" width="{sx(lid_end) - sx(lid_start):.1f}" height="660" fill="#dc2626" opacity="0.08"/>')
    for event_name, color, dash in (("stage1_hold_started_global_s", "#2563eb", "4 4"),
                                    ("stage2_hold_started_global_s", "#d97706", "4 4"),
                                    ("lid_open_started", "#dc2626", "2 3"),
                                    ("lid_window_completed", "#dc2626", "2 3")):
        if event_name in metrics["events"]:
            event_value = metrics["events"][event_name]
        else:
            event_value = event_time(events, event_name)
        out.append(f'<line x1="{sx(event_value):.1f}" x2="{sx(event_value):.1f}" y1="100" y2="760" stroke="{color}" stroke-dasharray="{dash}" stroke-width="1.2"/>')
    for phase in ("command_wait", PHASE1, PHASE2, LID, "stopped"):
        phase_rows = [row for row in rows if row["phase"] == phase]
        if not phase_rows:
            continue
        out.append(svg_polyline([(sx(row["elapsed_s"]), sy(row["temperature_c"], 0)) for row in phase_rows], colors[phase], 2.4 if phase in (PHASE1, PHASE2, LID) else 1.2))
        out.append(svg_polyline([(sx(row["elapsed_s"]), sy(row["output_permille"] / 10, 1)) for row in phase_rows], colors[phase], 1.8))
        out.append(svg_polyline([(sx(row["elapsed_s"]), sy(row["humidity_pct"], 2)) for row in phase_rows], colors[phase], 1.8))
    out.extend([f'<line x1="{left}" x2="{left + plot_width}" y1="{sy(40, 0):.1f}" y2="{sy(40, 0):.1f}" stroke="#2563eb" stroke-dasharray="6 4"/>',
                f'<line x1="{left}" x2="{left + plot_width}" y1="{sy(70, 0):.1f}" y2="{sy(70, 0):.1f}" stroke="#d97706" stroke-dasharray="6 4"/>'])
    legend = [("40 C stage", "#2563eb"), ("70 C stage", "#d97706"), ("lid open", "#dc2626")]
    for index, (label, color) in enumerate(legend):
        x = 92 + index * 170
        out.append(f'<line x1="{x}" x2="{x + 24}" y1="850" y2="850" stroke="{color}" stroke-width="3"/>')
        out.append(f'<text x="{x + 32}" y="854" class="small">{escape(label)}</text>')
    out.append('</svg>')
    return "\n".join(out)


def np_ticks(low, high, count):
    step = (high - low) / count
    return [low + step * index for index in range(count + 1)]


def lid_svg(lid_rows, lid_metrics):
    width, height = 1100, 620
    left, right, top, bottom = 86, 30, 80, 62
    plot_width, plot_height = width - left - right, 220
    duration = lid_rows[-1]["stage_elapsed_s"] - lid_rows[0]["stage_elapsed_s"]
    t0 = lid_rows[0]["stage_elapsed_s"]

    def sx(value): return left + (value - t0) / duration * plot_width
    def sy_temp(value): return top + plot_height - (value - 64) / 8 * plot_height
    def sy_pct(value): return 365 + 150 - value / 105 * 150

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           '<style>text{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;fill:#1f2937}.grid{stroke:#e5e7eb;stroke-width:1}.axis{stroke:#374151;stroke-width:1}.small{font-size:12px}.label{font-size:14px}.title{font-size:21px;font-weight:600}</style>',
           '<text x="86" y="38" class="title">Dynamic response during the 10-second lid-open interval</text>',
           f'<text x="86" y="61" class="small">Temperature drop: {lid_metrics["temperature_drop_c"]:.2f} C | average slope: {lid_metrics["average_drop_rate_c_per_min"]:.2f} C/min | peak output: {lid_metrics["peak_output_percent"]:.0f}%</text>']
    for y, high, low, label in ((top, 72, 64, "Temperature (C)"), (365, 105, 0, "Output / humidity (%)")):
        ph = plot_height if y == top else 150
        out.append(f'<rect x="{left}" y="{y}" width="{plot_width}" height="{ph}" fill="none" class="axis"/>')
        for tick in np_ticks(low, high, 4 if y == top else 5):
            yy = y + ph - (tick - low) / (high - low) * ph
            out.append(f'<line x1="{left}" x2="{left + plot_width}" y1="{yy:.1f}" y2="{yy:.1f}" class="grid"/>')
            out.append(f'<text x="{left - 12}" y="{yy + 4:.1f}" text-anchor="end" class="small">{tick:g}</text>')
        out.append(f'<text x="20" y="{y + ph / 2:.1f}" transform="rotate(-90 20 {y + ph / 2:.1f})" text-anchor="middle" class="label">{label}</text>')
    for tick in np_ticks(0, duration, 5):
        x = left + tick / duration * plot_width
        out.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{top}" y2="515" class="grid"/>')
        out.append(f'<text x="{x:.1f}" y="540" text-anchor="middle" class="small">{tick:.1f}</text>')
    out.append('<text x="580" y="575" text-anchor="middle" class="label">Seconds after lid opened</text>')
    points_temp = [(sx(row["stage_elapsed_s"]), sy_temp(row["temperature_c"])) for row in lid_rows]
    points_out = [(sx(row["stage_elapsed_s"]), sy_pct(row["output_permille"] / 10)) for row in lid_rows]
    points_hum = [(sx(row["stage_elapsed_s"]), sy_pct(row["humidity_pct"])) for row in lid_rows]
    out.append(svg_polyline(points_temp, "#dc2626", 2.8))
    out.append(svg_polyline(points_out, "#2563eb", 2.2))
    out.append(svg_polyline(points_hum, "#16a34a", 2.2))
    for x, y in points_temp:
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3" fill="#dc2626"/>')
    out.append(f'<line x1="{left}" x2="{left + plot_width}" y1="{sy_temp(70):.1f}" y2="{sy_temp(70):.1f}" stroke="#d97706" stroke-dasharray="6 4"/>')
    for index, (label, color) in enumerate((("Temperature", "#dc2626"), ("Heater output", "#2563eb"), ("Humidity", "#16a34a"))):
        x = 86 + index * 180
        out.append(f'<line x1="{x}" x2="{x + 24}" y1="600" y2="600" stroke="{color}" stroke-width="3"/>')
        out.append(f'<text x="{x + 32}" y="604" class="small">{label}</text>')
    out.append('</svg>')
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    args = parser.parse_args()
    directory = Path(args.input)
    rows = []
    with (directory / "samples.csv").open() as handle:
        for row in csv.DictReader(handle):
            for key in ("elapsed_s", "stage_elapsed_s", "temperature_c", "humidity_pct", "age_ms"):
                row[key] = float(row[key])
            for key in ("stm32_tick_ms", "output_permille"):
                row[key] = int(float(row[key]))
            rows.append(row)
    result = json.loads((directory / "result.json").read_text())
    events = result["events"]
    rows1 = [row for row in rows if row["phase"] == PHASE1]
    rows2 = [row for row in rows if row["phase"] == PHASE2]
    lid_rows = [row for row in rows if row["phase"] == LID]
    metrics = {
        "source": str(directory / "samples.csv"),
        "analysis_definition": {
            "settling_time": "time from each stage's first sample until all remaining stage samples stay within +/-0.2 C",
            "control_band_time": "same calculation using +/-0.5 C, the band used to start the 120 s hold",
            "overshoot": "maximum recorded feedback temperature minus target",
            "temperature_source": "gateway fused temperature feedback",
        },
        "stage1_40c": stage_metrics(rows1, 40.0, result["hold_seconds"], "room_to_40C"),
        "stage2_70c": stage_metrics(rows2, 70.0, result["hold_seconds"], "40C_to_70C"),
    }
    lid_temperature = [row["temperature_c"] for row in lid_rows]
    lid_elapsed = [row["stage_elapsed_s"] for row in lid_rows]
    lid_humidity = [row["humidity_pct"] for row in lid_rows]
    lid_output = [row["output_permille"] / 10.0 for row in lid_rows]
    metrics["lid_open_10s"] = {
        "samples": len(lid_rows),
        "duration_s": float(lid_elapsed[-1] - lid_elapsed[0]),
        "start_temperature_c": float(lid_temperature[0]),
        "end_temperature_c": float(lid_temperature[-1]),
        "minimum_temperature_c": float(min(lid_temperature)),
        "temperature_drop_c": float(lid_temperature[0] - min(lid_temperature)),
        "average_drop_rate_c_per_min": float((lid_temperature[-1] - lid_temperature[0]) /
                                               (lid_elapsed[-1] - lid_elapsed[0]) * 60.0),
        "start_humidity_pct": float(lid_humidity[0]),
        "end_humidity_pct": float(lid_humidity[-1]),
        "start_output_percent": float(lid_output[0]),
        "end_output_percent": float(lid_output[-1]),
        "peak_output_percent": float(max(lid_output)),
    }
    metrics["events"] = {
        "stage1_hold_started_global_s": event_time(events, "hold_started"),
        "stage1_hold_completed_global_s": events[4]["elapsed_s"],
        "stage2_hold_started_global_s": events[7]["elapsed_s"],
        "stage2_hold_completed_global_s": next(event["elapsed_s"] for event in events
                                               if event["event"] == "hold_completed" and
                                               event.get("stage") == PHASE2),
        "lid_open_started_global_s": event_time(events, "lid_open_started"),
        "lid_window_completed_global_s": event_time(events, "lid_window_completed"),
        "stop_confirmed_global_s": event_time(events, "stop_confirmed"),
    }
    (directory / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n")

    (directory / "temperature_test_analysis.svg").write_text(overview_svg(rows, events, metrics))
    (directory / "lid_dynamic_response.svg").write_text(lid_svg(lid_rows, metrics["lid_open_10s"]))
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
