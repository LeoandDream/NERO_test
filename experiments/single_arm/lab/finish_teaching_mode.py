"""Exit idle teaching mode at S1 without publishing a joint target."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

from experiments.single_arm.can_setup.drag_start_guard import S1_JOINT_RAD
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


REPORT_DIR = Path("experiments/single_arm/lab/data/finish_teaching_mode")


def run(config_path=DEFAULT_CONFIG):
    config = load_config(Path(config_path))
    report = {"kind": "finish_idle_teaching_mode_at_s1",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "move_j_targets_published": 0}
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    path = REPORT_DIR / ("nero_finish_teaching_mode_" +
                         datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    try:
        with connected(config) as robot:
            state = read_state(robot)
            report["start_state"] = state
            if (config["mount"] != "left" or state["control_mode"] != 2
                    or state["arm_status"] != 0 or state["teach_status"] not in (0, 2, 6)
                    or state["error_code"] != 0 or state["joint_variation_0_25s_rad"] > 0.002
                    or any(not item["enabled"] or item["undervoltage"] or item["driver_error"]
                           for item in state["drivers"])):
                raise RuntimeError("控制器不是停稳、七轴使能的空闲示教模式")
            q = state["joint_rad"]
            if max(abs(a-b) for a, b in zip(q, S1_JOINT_RAD)) > 0.005:
                raise RuntimeError("当前关节角不在 S1 附近")
            execute_route(robot, config, [q], lambda: False, speed_percent=5,
                          target_tolerance_rad=0.005, waypoint_timeout_s=20,
                          waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                          path_mode="joint_box", joint_box_margin_rad=0.03)
            final = require_ready(read_state(robot))
            report["final_state"] = final
            if max(abs(a-b) for a, b in zip(final["joint_rad"], S1_JOINT_RAD)) > 0.005:
                raise RuntimeError("切回 CAN 后姿态偏离 S1")
            report["completed"] = True
            print("已切回 CAN 控制并停在 S1；未发布 move_j 目标。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"切回 CAN 未完成：{exc}；报告 {path}") from exc
    finally:
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("模式恢复报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    if not args.run:
        parser.error("须显式加 --run；此命令会发送 CAN 模式帧，但不会发布 move_j 目标")
    try:
        run()
    except (OSError, ValueError, RuntimeError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
