#!/usr/bin/env python3
"""Run the requested room->40C->70C->10s open-lid test with full telemetry logs."""

import argparse
import csv
import http.client
import json
import math
import sys
import time
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.60")
    parser.add_argument("--output", required=True)
    parser.add_argument("--hold-seconds", type=float, default=120.0)
    parser.add_argument("--lid-seconds", type=float, default=10.0)
    return parser.parse_args()


class Gateway:
    def __init__(self, host):
        self.host = host

    def request(self, path, body=None):
        last_error = None
        for attempt in range(3):
            connection = http.client.HTTPConnection(self.host, timeout=4)
            try:
                encoded = None if body is None else json.dumps(body)
                headers = {} if body is None else {"Content-Type": "application/json"}
                connection.request("GET" if body is None else "POST", path,
                                   body=encoded, headers=headers)
                response = connection.getresponse()
                payload = response.read()
                if response.status >= 400:
                    raise RuntimeError(f"HTTP {response.status}: {payload[:400]!r}")
                return payload if path.endswith(".csv") else json.loads(payload)
            except (OSError, http.client.HTTPException, json.JSONDecodeError) as error:
                last_error = error
                if attempt < 2:
                    time.sleep(0.25)
            finally:
                connection.close()
        raise RuntimeError(f"gateway request failed: {last_error}")

    def status(self):
        return self.request("/api/status")

    def command(self, running, target):
        return self.request("/api/control", {
            "running": running,
            "temperature_enabled": True,
            "target_temperature_c": target,
            "humidity_enabled": False,
            "target_humidity_pct": 19,
            "clear_fault": False,
            "persist": False,
        })


def validate(data, target=None):
    required = ("temperature_c", "humidity_pct")
    if (not data.get("present") or not data.get("valid") or data.get("stale") or
            data.get("age_ms", 999999) > 2500 or data.get("fault") or
            not data.get("fan_valid") or not data.get("fan_running") or
            not all(math.isfinite(float(data[key])) for key in required)):
        raise RuntimeError("unsafe or invalid telemetry: " + json.dumps(data, ensure_ascii=False))
    if target is not None and float(data["temperature_c"]) > target + 2.0:
        raise RuntimeError("experiment temperature limit exceeded: " + json.dumps(data, ensure_ascii=False))


def main():
    args = parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    gateway = Gateway(args.host)
    started = time.monotonic()
    last_tick = None
    events = []

    fields = [
        "elapsed_s", "phase", "stage_elapsed_s", "stm32_tick_ms", "temperature_c",
        "humidity_pct", "output_permille", "running", "heater_on", "motor_on",
        "fan_running", "fan_valid", "remote_owned", "target_temperature_c",
        "fault", "age_ms",
    ]
    samples_file = (output / "samples.csv").open("x", newline="")
    raw_file = (output / "raw_status.jsonl").open("x")
    samples = csv.DictWriter(samples_file, fieldnames=fields)
    samples.writeheader()

    def event(name, **values):
        record = {"elapsed_s": round(time.monotonic() - started, 3), "event": name, **values}
        events.append(record)
        print("EVENT " + json.dumps(record, ensure_ascii=False), flush=True)

    def capture(phase, stage_started, target=None):
        nonlocal last_tick
        status = gateway.status()
        data = status["data"]
        validate(data, target)
        tick = data["stm32_tick_ms"]
        if last_tick is not None and tick < last_tick:
            raise RuntimeError("STM32 tick moved backwards; possible reset")
        if tick != last_tick:
            last_tick = tick
            row = {
                "elapsed_s": round(time.monotonic() - started, 3),
                "phase": phase,
                "stage_elapsed_s": round(time.monotonic() - stage_started, 3),
                **{key: data.get(key) for key in fields
                   if key not in ("elapsed_s", "phase", "stage_elapsed_s")},
            }
            samples.writerow(row)
            samples_file.flush()
            raw_file.write(json.dumps({"recorded_at_unix": time.time(),
                                       "phase": phase, "status": status},
                                      ensure_ascii=False) + "\n")
            raw_file.flush()
        return status, data

    def wait_for_applied(target, phase_started):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status, data = capture("command_wait", phase_started, target)
            if (data.get("running") and data.get("target_temperature_c") == target and
                    data.get("temperature_enabled") and not data.get("humidity_enabled")):
                return status, data
            time.sleep(0.5)
        raise RuntimeError("start command was not applied")

    def run_to_target(target, phase_name):
        phase_started = time.monotonic()
        reply = gateway.command(True, target)
        event("stage_start_command", stage=phase_name, target_c=target, reply=reply)
        _, data = wait_for_applied(target, phase_started)
        event("stage_running", stage=phase_name, target_c=target,
              temperature_c=data["temperature_c"])
        in_band_since = None
        next_report = time.monotonic()
        while True:
            status, data = capture(phase_name, phase_started, target)
            temperature = float(data["temperature_c"])
            in_band = target - 0.5 <= temperature <= target + 0.5
            if in_band:
                if in_band_since is None:
                    in_band_since = time.monotonic()
                    event("hold_started", stage=phase_name, target_c=target,
                          temperature_c=temperature)
                elif time.monotonic() - in_band_since >= args.hold_seconds:
                    event("hold_completed", stage=phase_name, target_c=target,
                          temperature_c=temperature,
                          hold_seconds=round(time.monotonic() - in_band_since, 3))
                    return phase_started, reply, status, data
            else:
                if in_band_since is not None:
                    event("hold_reset", stage=phase_name, target_c=target,
                          temperature_c=temperature)
                in_band_since = None
            if time.monotonic() >= next_report:
                print("SAMPLE " + json.dumps({"stage": phase_name,
                      "temperature_c": round(temperature, 3),
                      "target_c": target,
                      "output_permille": data["output_permille"],
                      "hold_s": None if in_band_since is None else round(time.monotonic() - in_band_since, 1)},
                      ensure_ascii=False), flush=True)
                next_report += 30
            time.sleep(0.45)

    def stop_and_confirm(target):
        reply = gateway.command(False, target)
        event("stop_command", reply=reply)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status = gateway.status()
            data = status["data"]
            if (data.get("valid") and not data.get("stale") and not data.get("running") and
                    data.get("output_permille") == 0 and not data.get("motor_on") and
                    data.get("ack_sequence") == reply.get("sequence") and
                    data.get("ack_result") == 1):
                capture("stopped", time.monotonic(), target)
                event("stop_confirmed", temperature_c=data.get("temperature_c"),
                      stm32_tick_ms=data.get("stm32_tick_ms"))
                return data
            time.sleep(0.5)
        raise RuntimeError("stop command was not confirmed")

    result = {"completed": False, "host": args.host,
              "started_unix": time.time(), "hold_seconds": args.hold_seconds,
              "lid_seconds": args.lid_seconds}
    try:
        initial = gateway.status()
        validate(initial["data"])
        result["initial_status"] = initial["data"]
        event("initial_status", temperature_c=initial["data"]["temperature_c"],
              target_c=initial["data"].get("target_temperature_c"),
              running=initial["data"].get("running"))

        _, _, _, data40 = run_to_target(40, "stage1_room_to_40_hold")
        result["stage1_reached"] = data40
        _, _, _, data70 = run_to_target(70, "stage2_40_to_70_hold")
        result["stage2_reached"] = data70

        event("open_lid_ready", temperature_c=data70["temperature_c"],
              instruction="请打开盖子；打开后在终端按回车开始10秒动态记录")
        print("OPEN_LID_NOW 请打开盖子，打开后回复‘已打开’或在终端按回车。", flush=True)
        sys.stdout.flush()
        opened = input()
        open_started = time.monotonic()
        status, data = capture("lid_open_10s", open_started, 70)
        event("lid_open_started", user_input=opened,
              temperature_c=data["temperature_c"], stm32_tick_ms=data["stm32_tick_ms"])
        while time.monotonic() - open_started < args.lid_seconds:
            status, data = capture("lid_open_10s", open_started, 70)
            time.sleep(0.35)
        event("lid_window_completed", duration_s=round(time.monotonic() - open_started, 3),
              temperature_c=data["temperature_c"])
        print("CLOSE_LID_NOW 10秒动态记录完成，请关闭盖子。", flush=True)
        result["lid_end_status"] = data
        result["final_status"] = stop_and_confirm(70)
        result["completed"] = True
    except BaseException as error:
        result["error"] = repr(error)
        print("ERROR " + repr(error), flush=True)
        try:
            result["final_status"] = stop_and_confirm(70)
        except BaseException as stop_error:
            result["stop_error"] = repr(stop_error)
    finally:
        try:
            history = gateway.request("/api/history.csv")
            (output / "gateway_history.csv").write_bytes(history)
        except BaseException as history_error:
            result["history_error"] = repr(history_error)
        result["events"] = events
        result["finished_unix"] = time.time()
        (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        samples_file.close()
        raw_file.close()
        print("RESULT " + json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
