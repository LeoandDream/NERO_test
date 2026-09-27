"""只测纯数据、只读计划和安全拒绝；不用 CAN 或实体机械臂。"""

import csv
from contextlib import nullcontext
from datetime import datetime, timedelta, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.single_arm.lab import complete_recording, cube, pose_candidate, recordings, replay, site_recovery, smooth_replay, starts
from experiments.single_arm.lab.api import LabAPI
from experiments.single_arm.lab.cli import (
    cube_extrema_dispatch, cube_guide, parser_create, quick_dispatch,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class SealedRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.config = load_config(DEFAULT_CONFIG)
        self.q = self.config["safe_start_joint_rad"]

    def make_recording(self, name, *, complete=True, config_hash=None,
                       rows_count=4, post_stop_samples=2):
        path = self.folder / (name + ".csv")
        fields = ["sample_index", "elapsed_s", "joint_feedback_unix_s",
                  "pose_feedback_unix_s", "teach_status", "x_m", "y_m", "z_m"]
        fields += [f"joint_{i}_rad" for i in range(1,8)]
        with path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(rows_count):
                q = list(self.q)
                q[0] += .001*min(index,2)
                writer.writerow({"sample_index": index, "elapsed_s": index*.1,
                                 "joint_feedback_unix_s": 100+index*.1,
                                 "pose_feedback_unix_s":100+index*.1,
                                 "teach_status":"START_RECORDING" if index<rows_count-post_stop_samples else "DISABLED",
                                 "x_m":.38,"y_m":0,"z_m":.57,
                                 **{f"joint_{i+1}_rad": value
                                    for i,value in enumerate(q)}})
        metadata = {"schema_version":2, "robot_model":"NERO",
                    "firmware_driver":"V121", "recording":str(path),
                    "recording_id":path.stem,
                    "recording_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
                    "config_sha256": config_hash or hashlib.sha256(
                        Path(DEFAULT_CONFIG).read_bytes()).hexdigest(),
                    "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                    "max_teach_seconds":5, "stop_reason":"time_limit",
                    "return_completed":complete, "post_stop_samples":post_stop_samples,
                    "recorded_samples":rows_count, "mount":self.config["mount"],
                    "safe_start_name":self.config["safe_start_name"],
                    "original_joint_rad":self.q,
                    "emergency_stop_requested":False}
        path.with_suffix(".json").write_text(json.dumps(metadata), encoding="utf-8")
        return path

    def test_valid_latest_and_hash(self):
        valid = self.make_recording("nero_session_20260926T164431Z")
        failed = self.make_recording("nero_session_20260926T183143Z", complete=False)
        meta_path = failed.with_suffix(".json")
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta["error"] = "关节 4 超出限位"
        meta_path.write_text(json.dumps(meta), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "最近一次示教.*关节 4 超出限位"):
            recordings.resolve_recording("latest", directory=self.folder)
        self.assertEqual(recordings.resolve_recording(valid.stem, directory=self.folder).path,
                         valid)
        self.assertEqual([item.path for item in recordings.list_recordings(
            directory=self.folder)], [valid])
        with valid.open("a", encoding="utf-8") as stream:
            stream.write("\n")
        with self.assertRaisesRegex(ValueError, "CSV 已变化"):
            recordings.validate_recording(valid, directory=self.folder)
        self.assertEqual(recordings.list_recordings(directory=self.folder), [])

    def test_incomplete_config_mismatch_and_outside_rejected(self):
        incomplete = self.make_recording("nero_session_incomplete", complete=False)
        wrong = self.make_recording("nero_session_wrong", config_hash="0"*64)
        with self.assertRaisesRegex(ValueError, "完整限时"):
            recordings.validate_recording(incomplete, directory=self.folder)
        with self.assertRaisesRegex(ValueError, "配置"):
            recordings.validate_recording(wrong, directory=self.folder)
        with self.assertRaisesRegex(ValueError, "指定记录目录"):
            recordings.validate_recording(DEFAULT_CONFIG, directory=self.folder)

    def test_separate_return_attestation_preserves_failed_source_and_rejects_tampering(self):
        path = self.make_recording("nero_session_20260926T185010Z",
                                   complete=False, rows_count=12,
                                   post_stop_samples=5)
        metadata_path = path.with_suffix(".json")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["error"] = "当前未停稳或控制器/七轴状态不允许回位"
        metadata["finished_at_utc"] = (
            datetime.now(timezone.utc) - timedelta(seconds=5)).isoformat()
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "缺少独立回位凭证"):
            recordings.validate_recording(path, directory=self.folder)
        finished = datetime.fromisoformat(metadata["finished_at_utc"])
        plan_dir, run_dir = self.folder / "plans", self.folder / "runs"
        plan_dir.mkdir()
        run_dir.mkdir()
        plan_path, report_path = plan_dir / "return.json", run_dir / "report.json"
        last = list(self.q)
        last[0] += .002
        plan = {"kind": "live_planned_return_to_s1", "target_name": "S1",
                "config_sha256": metadata["config_sha256"],
                "source_recording": None, "reversed_recording": False,
                "controller_targets": 1, "target_joint_rad": self.q,
                "start_joint_rad": last,
                "created_at_utc": (finished + timedelta(seconds=1)).isoformat()}
        plan_path.write_text(json.dumps(plan), encoding="utf-8")
        ready = {"ready_for_motion": True, "control_mode": 1,
                 "arm_status": 0, "error_code": 0, "joint_rad": self.q}
        report = {"plan": str(plan_path), "completed": True,
                  "motion_attempted": True, "final_max_joint_error_rad": 0,
                  "final_state": ready,
                  "started_at_utc": (finished + timedelta(seconds=2)).isoformat(),
                  "finished_at_utc": (finished + timedelta(seconds=2, milliseconds=500)).isoformat()}
        report_path.write_text(json.dumps(report), encoding="utf-8")
        with (patch.object(complete_recording, "RECORDING_DIR", self.folder),
              patch.object(complete_recording, "PLAN_DIR", plan_dir),
              patch.object(complete_recording, "RUN_DIR", run_dir)):
            with (patch.object(complete_recording, "connected",
                               return_value=nullcontext(object())),
                  patch.object(complete_recording, "read_state", return_value=ready)):
                sidecar_path = complete_recording.attest(path, report_path)
            self.assertTrue(sidecar_path.exists())
            self.assertFalse(json.loads(metadata_path.read_text(encoding="utf-8"))[
                "return_completed"])
            self.assertEqual(recordings.validate_recording(path, directory=self.folder).path,
                             path)
            self.assertEqual(recordings.resolve_recording("latest", directory=self.folder).path,
                             path)
            report["completed"] = False
            report_path.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "报告已变化"):
                recordings.validate_recording(path, directory=self.folder)
            report["completed"] = True
            report_path.write_text(json.dumps(report), encoding="utf-8")
            metadata["error"] = "关节故障"
            metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "只允许完整限时示教"):
                recordings.validate_recording(path, directory=self.folder)

    def test_sealed_but_out_of_order_teach_rows_rejected(self):
        path=self.make_recording("nero_session_bad_order")
        with path.open(newline="",encoding="utf-8") as stream:
            rows=list(csv.DictReader(stream))
        rows[1]["teach_status"]="DISABLED"
        with path.open("w",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=rows[0].keys())
            writer.writeheader();writer.writerows(rows)
        meta_path=path.with_suffix(".json")
        meta=json.loads(meta_path.read_text(encoding="utf-8"))
        meta["recording_sha256"]=hashlib.sha256(path.read_bytes()).hexdigest()
        meta_path.write_text(json.dumps(meta),encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"顺序错误"):
            recordings.validate_recording(path,directory=self.folder)

    def test_brief_stop_recording_status_at_start_of_settling_is_valid(self):
        path = self.make_recording("nero_session_stop_transition",
                                   rows_count=12, post_stop_samples=5)
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))

        def reseal():
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            metadata_path = path.with_suffix(".json")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["recording_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

        rows[7]["teach_status"] = "STOP_RECORDING"
        reseal()
        self.assertEqual(recordings.validate_recording(path, directory=self.folder).path,
                         path)
        rows[9]["teach_status"] = "STOP_RECORDING"
        reseal()
        with self.assertRaisesRegex(ValueError, "顺序错误"):
            recordings.validate_recording(path, directory=self.folder)

    def test_first_drag_sample_allows_small_joint_motion_but_rejects_large_gap(self):
        path = self.make_recording("nero_session_first_motion")
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        def reseal():
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            meta_path = path.with_suffix(".json")
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta["recording_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
        rows[0]["joint_2_rad"] = str(self.q[1] + .012)
        reseal()
        recordings.validate_recording(path, directory=self.folder)
        rows[0]["joint_2_rad"] = str(self.q[1] + .021)
        reseal()
        with self.assertRaisesRegex(ValueError, "起点或停稳段"):
            recordings.validate_recording(path, directory=self.folder)

    def test_quick_plan_explicit_mode_and_tamper(self):
        path = self.make_recording("nero_session_replay")
        q = list(self.q)

        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def fk(self, joints): return [0.38, 0, 0.57, 0, 0, 0]

        with patch.object(replay, "read_state", return_value={
                "ready_for_motion":True, "joint_rad":q,
                "flange_pose_m_rad":[0.38,0,0.57,0,0,0]}) as observed:
            plan = replay.prepare_replay(path, recording_dir=self.folder,
                                         robot_factory=lambda _: Robot())
            observed.assert_called_once()
        self.assertEqual(plan["replay_mode"], "point_move_j")
        self.assertGreater(plan["total_steps"], 0)
        replay.verify_replay_plan(plan, recording_dir=self.folder)
        plan["route_joint_rad"][1][0] += .01
        with self.assertRaisesRegex(ValueError, "路线已改变"):
            replay.verify_replay_plan(plan, recording_dir=self.folder)
        with self.assertRaisesRegex(ValueError, "仅支持显式 point"):
            replay.prepare_replay(path, mode="continuous", recording_dir=self.folder)

    def test_new_continuous_plan_recomputes_before_any_publish(self):
        path = self.make_recording("nero_session_timed", rows_count=12,
                                   post_stop_samples=5)

        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def fk(self, joints): return [.38, 0, .57, 0, 0, 0]

        snapshot={"ready_for_motion":True,"joint_rad":list(self.q),
                  "flange_pose_m_rad":[.38,0,.57,0,0,0]}
        with patch.object(smooth_replay,"read_state",return_value=snapshot):
            plan=smooth_replay.build_continuous_plan(path,
                    recording_dir=self.folder,robot_factory=lambda _:Robot())
        self.assertEqual(plan["replay_mode"],"timed_move_j")
        self.assertEqual(plan["waypoints"][0]["joint_rad"],self.q)
        self.assertEqual(plan["waypoints"][-1]["joint_rad"],self.q)
        self.assertGreater(len(plan["waypoints"]),100)
        smooth_replay.verify_continuous_plan(plan,recording_dir=self.folder,
                                              robot_factory=lambda _:Robot())
        with patch.object(smooth_replay,"read_state",return_value=snapshot):
            slower=smooth_replay.build_continuous_plan(path,
                    recording_dir=self.folder,robot_factory=lambda _:Robot(),
                    min_time_scale=12.0)
        self.assertEqual(slower["min_time_scale"],12.0)
        self.assertGreaterEqual(slower["time_scale"],12.0)
        self.assertGreaterEqual(slower["total_duration_s"],plan["total_duration_s"])
        smooth_replay.verify_continuous_plan(slower,recording_dir=self.folder,
                                              robot_factory=lambda _:Robot())
        plan_path=self.folder/"timed_plan.json"
        plan_path.write_text(json.dumps(plan),encoding="utf-8")
        from experiments.single_arm.continuous_replay import run_stream
        with patch.object(run_stream,"run",return_value=self.folder/"safe.json") as publisher:
            smooth_replay.execute_continuous_plan(plan_path,recording_dir=self.folder,
                         robot_factory=lambda _:Robot())
            self.assertEqual(publisher.call_count,1)
            plan["waypoints"][3]["joint_rad"][1]+=.001
            plan_path.write_text(json.dumps(plan),encoding="utf-8")
            with self.assertRaisesRegex(ValueError,"waypoints"):
                smooth_replay.execute_continuous_plan(plan_path,
                         recording_dir=self.folder,robot_factory=lambda _:Robot())
            self.assertEqual(publisher.call_count,1)

    def test_replay_preflight_failure_keeps_report_without_motion(self):
        path = self.make_recording("nero_session_failure")
        q = list(self.q)

        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def fk(self, joints): return [0.38,0,0.57,0,0,0]

        with patch.object(replay,"read_state",return_value={
                "ready_for_motion":True,"joint_rad":q,
                "flange_pose_m_rad":[0.38,0,0.57,0,0,0]}):
            plan=replay.prepare_replay(path,recording_dir=self.folder,
                                       robot_factory=lambda _:Robot())
        plan_path=self.folder/"plan.json"
        plan_path.write_text(json.dumps(plan),encoding="utf-8")
        far=list(q);far[0]+=.02
        with patch.object(replay,"read_state",return_value={
                "ready_for_motion":True,"joint_rad":far,
                "flange_pose_m_rad":[0.38,0,0.57,0,0,0]}):
            with self.assertRaisesRegex(RuntimeError,"姿态已偏离"):
                replay.execute_replay(plan_path,recording_dir=self.folder,
                                      robot_factory=lambda _:Robot(),
                                      run_dir=self.folder/"runs")
        reports=list((self.folder/"runs").glob("nero_quick_replay_*.json"))
        self.assertEqual(len(reports),1)
        failed=json.loads(reports[0].read_text(encoding="utf-8"))
        self.assertFalse(failed["completed"])
        self.assertEqual(failed["last_published_step"],0)
        self.assertIn("姿态已偏离",failed["error"])

    def test_replay_rejects_changed_displayed_flange_range_before_publish(self):
        path = self.make_recording("nero_session_range")
        q = list(self.q)

        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def fk(self, joints): return [0.38, 0, 0.57, 0, 0, 0]

        snapshot = {"ready_for_motion":True, "joint_rad":q,
                    "flange_pose_m_rad":[0.38, 0, 0.57, 0, 0, 0]}
        with patch.object(replay,"read_state",return_value=snapshot):
            plan = replay.prepare_replay(path, recording_dir=self.folder,
                                         robot_factory=lambda _:Robot())
            plan["flange_xyz_envelope_m"]["x"][1] += .1
            plan_path = self.folder / "range_plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError,"法兰预测范围已改变"):
                replay.execute_replay(plan_path, recording_dir=self.folder,
                                      robot_factory=lambda _:Robot(),
                                      run_dir=self.folder/"range_runs")
        report = json.loads(next((self.folder/"range_runs").glob("*.json"))
                            .read_text(encoding="utf-8"))
        self.assertFalse(report["completed"])
        self.assertEqual(report["last_published_step"],0)


class CubeTests(unittest.TestCase):
    @staticmethod
    def samples(size=.20, omit=None):
        angle=.42
        rotation=[[math.cos(angle),-math.sin(angle),0],
                  [math.sin(angle),math.cos(angle),0],[0,0,1]]
        center=[.3,.1,.5]
        result=[]
        for axis in range(3):
            for sign in (-1,1):
                face=("+" if sign>0 else "-")+"XYZ"[axis]
                if face==omit: continue
                others=[index for index in range(3) if index!=axis]
                for u,v in ((-.35,-.35),(.35,-.35),(-.35,.35),(.35,.35)):
                    local=[0,0,0]
                    local[axis]=sign*(size[axis] if isinstance(size,tuple) else size)/2
                    local[others[0]]=u*(size[others[0]] if isinstance(size,tuple) else size)/2
                    local[others[1]]=v*(size[others[1]] if isinstance(size,tuple) else size)/2
                    point=[center[i]+sum(rotation[i][j]*local[j] for j in range(3))
                           for i in range(3)]
                    result.append({"face":face,"flange_pose_m_rad":point+[0,0,0]})
        return result

    def test_rotated_cube_is_geometry_only(self):
        fit=cube.fit_cube(self.samples())
        self.assertTrue(fit["geometry_valid"], fit["reason"])
        self.assertFalse(fit["motion_region_verified"])
        self.assertAlmostEqual(fit["side_length_m"], .2, places=5)

    def test_missing_face_and_rectangular_prism_rejected(self):
        missing=cube.fit_cube(self.samples(omit="-Z"))
        self.assertFalse(missing["geometry_valid"])
        self.assertIn("-Z", missing["missing_or_undercovered_faces"])
        prism=cube.fit_cube(self.samples(size=(.2,.2,.3)))
        self.assertFalse(prism["geometry_valid"])
        self.assertIn("不等长", prism["reason"])
        outlier=self.samples()
        outlier[0]["flange_pose_m_rad"][0]+=.04
        bad=cube.fit_cube(outlier)
        self.assertFalse(bad["geometry_valid"])
        self.assertIsNotNone(bad["reason"])

    def test_session_preserves_raw_samples_and_fit_artifact(self):
        with tempfile.TemporaryDirectory() as folder:
            session=cube.create_session(directory=Path(folder))
            manifest=json.loads((session/"manifest.json").read_text(encoding="utf-8"))
            for index,sample in enumerate(self.samples()):
                sample["config_sha256"]=manifest["config_sha256"]
                (session/f"sample_{index:03d}.json").write_text(
                    json.dumps(sample),encoding="utf-8")
            status=cube.session_status(session)
            self.assertEqual(status["sample_count"],24)
            output,fit=cube.fit_session(session)
            self.assertTrue(output.exists())
            self.assertTrue(fit["geometry_valid"])
            self.assertEqual(len(fit["raw_sample_sha256"]),24)
            self.assertFalse(fit["motion_region_verified"])

    def test_capture_accepts_controller_idle_teach_status_six(self):
        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def get_arm_status(self):
                msg=SimpleNamespace(ctrl_mode=1,arm_status=0,teach_status=6,err_code=0)
                return SimpleNamespace(timestamp=time.time(),msg=msg)
            def get_joint_angles(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.1]*7)
            def get_flange_pose(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.3,.1,.5,0,0,0])
            def fk(self,joints): return [.3,.1,.5,0,0,0]
        with tempfile.TemporaryDirectory() as folder:
            session=cube.create_session(directory=Path(folder))
            path=cube.capture_face(session,"+X",robot_factory=lambda _:Robot())
            sample=json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(sample["face"],"+X")
            self.assertEqual(sample["teach_status"],6)
            self.assertEqual(cube.session_status(session)["sample_count"],1)

    def test_capture_accepts_live_can_drag_mode(self):
        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def get_arm_status(self):
                msg=SimpleNamespace(ctrl_mode=2,arm_status=0,teach_status=1,err_code=0)
                return SimpleNamespace(timestamp=time.time(),msg=msg)
            def get_joint_angles(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.1]*7)
            def get_flange_pose(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.3,.1,.5,0,0,0])
            def fk(self,joints): return [.3,.1,.5,0,0,0]
        with tempfile.TemporaryDirectory() as folder:
            session=cube.create_session(directory=Path(folder))
            path=cube.capture_face(session,"+X",robot_factory=lambda _:Robot())
            sample=json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(sample["face"],"+X")
            self.assertEqual(sample["arm_status"],0)
            self.assertEqual(sample["teach_status"],1)

    def test_guided_capture_needs_no_predeclared_cube_size(self):
        class FakeAPI:
            counts={face:0 for face in cube.FACES}
            def cube_new(self): return {"session_path":"session"}
            def cube_status(self,session):
                return {"face_sample_counts":dict(self.counts),
                        "sample_count":sum(self.counts.values())}
            def cube_capture(self,session,face):
                self.counts[face]+=1
                return {"sample_path":f"{face}_{self.counts[face]}"}
            def cube_fit(self,session):
                return {"fit_path":"fit.json","fit":{"geometry_valid":True,
                        "side_length_m":.2}}
        args=parser_create().parse_args(["cube","guide"])
        api=FakeAPI()
        result=cube_guide(args,api,is_tty=True,input_fn=lambda _:"")
        self.assertFalse(result["partial"])
        self.assertEqual(result["status"]["sample_count"],18)
        self.assertEqual(result["fit"]["side_length_m"],.2)

    def test_extrema_candidate_keeps_six_faces_unverified(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"map.csv"
            columns=["level","source","source_index","time_utc","x_m","y_m","z_m","joint_rad"]
            points=[(0,0,0),(.2,0,0),(0,.16,0),(0,0,.18),(.2,.16,.18),(.1,.08,.09)]
            with path.open("w",newline="",encoding="utf-8") as stream:
                writer=csv.DictWriter(stream,fieldnames=columns)
                writer.writeheader()
                for index,xyz in enumerate(points):
                    writer.writerow({"level":"measured","source":"trial.json",
                                     "source_index":index,"time_utc":"2026-09-26T00:00:00Z",
                                     "x_m":xyz[0],"y_m":xyz[1],"z_m":xyz[2],
                                     "joint_rad":json.dumps([0]*7)})
            path.with_suffix(".json").write_text(
                json.dumps({"levels":{"measured":{"samples":len(points)}}}),
                encoding="utf-8")
            result=cube.extrema_candidate(path,level="measured",margin_m=.01)
            self.assertAlmostEqual(result["candidate_side_length_m"],.14)
            self.assertEqual(result["candidate_center_base_m"],[.1,.08,.09])
            self.assertEqual(result["candidate_face_centers"]["+X"]["plane_in_base"],"Y-Z")
            self.assertEqual(len(result["candidate_vertices_base_m"]),8)
            for actual, expected in zip(result["candidate_cube_bounds_base_m"]["x"],
                                        (.03,.17)):
                self.assertAlmostEqual(actual,expected)
            self.assertFalse(result["candidate_vertices_observed"])
            self.assertFalse(result["geometry_valid"])
            self.assertFalse(result["motion_region_verified"])
            inset=cube.extrema_candidate(
                path,level="measured",margin_m=.01,bottom_margin_m=.03)
            self.assertAlmostEqual(inset["inset_axis_ranges_m"]["x"][0],.03)
            self.assertAlmostEqual(inset["candidate_center_base_m"][0],.11)
            self.assertTrue(all(item["observed_point_only"]
                                for item in inset["recommended_measured_extrema"].values()))
            with self.assertRaisesRegex(ValueError,"不足 20 mm"):
                cube.extrema_candidate(path,level="measured",margin_m=.07)

    def test_hand_guided_recording_preserves_provenance_and_rejects_bad_state(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "drag.csv"
            columns = ["sample_index", "control_mode", "arm_status", "teach_status",
                       "captured_at_utc", "x_m", "y_m", "z_m",
                       *(f"joint_{i}_rad" for i in range(1, 8))]
            corners = [(0, 0, 0), (.2, 0, 0), (0, .16, 0),
                       (0, 0, .18), (.2, .16, .18), (.1, .08, .09)]
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                for index in range(24):
                    xyz = corners[index % len(corners)]
                    writer.writerow({"sample_index": index,
                                     "control_mode": "TEACHING_MODE",
                                     "arm_status": "NORMAL",
                                     "teach_status": "START_RECORDING",
                                     "captured_at_utc": "2026-09-26T00:00:00Z",
                                     "x_m": xyz[0], "y_m": xyz[1], "z_m": xyz[2],
                                     **{f"joint_{i}_rad": 0 for i in range(1, 8)}})
            mapped = cube.map_from_drag_recording(source, directory=folder)
            candidate = cube.extrema_candidate(
                mapped, level="new_hand_guided_sweep", margin_m=.01)
            self.assertEqual(candidate["sample_count"], 24)
            self.assertEqual(candidate["axis_extrema"]["x"]["max"]["source"], str(source))
            self.assertEqual(candidate["source_recording_csv"],str(source))
            self.assertEqual(candidate["source_recording_sha256"],
                             hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertAlmostEqual(candidate["candidate_side_length_m"], .14)
            self.assertFalse(candidate["motion_region_verified"])
            text = source.read_text(encoding="utf-8").replace(
                "TEACHING_MODE", "CAN_CTRL", 1)
            source.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "不是连续的正常拖动反馈"):
                cube.map_from_drag_recording(source, directory=folder)

    def test_cube_extrema_recording_cli_keeps_source_level_distinct(self):
        class API:
            def cube_extrema_from_recording(self, recording, *, margin_m):
                return {"recording": str(recording), "margin_m": margin_m}
        parser = parser_create()
        args = parser.parse_args(["cube", "extrema", "--recording", "drag.csv"])
        self.assertEqual(cube_extrema_dispatch(args, API())["recording"], "drag.csv")
        args = parser.parse_args(["cube", "extrema", "--recording", "drag.csv",
                                  "--level", "measured"])
        with self.assertRaisesRegex(ValueError, "不接受 --level"):
            cube_extrema_dispatch(args, API())


class StartsAndImportTests(unittest.TestCase):
    def test_import_has_no_robot_creation(self):
        from pyAgxArm import AgxArmFactory
        with patch.object(AgxArmFactory, "create_arm", side_effect=AssertionError("CAN created")):
            importlib.reload(importlib.import_module("experiments.single_arm.lab.api"))

    def test_integer_degree_planar_candidate_stays_near_s1_and_rejects_tamper(self):
        config=load_config(DEFAULT_CONFIG)
        start=config["safe_start_joint_rad"]
        state={"ready_for_motion":True,"joint_rad":start,
               "flange_pose_m_rad":[.382,-.014,.572,0,0,0]}
        plan=pose_candidate.prepare(snapshot=state)
        self.assertEqual(plan["geometry"]["candidate_joint_deg"],
                         [0,-33,0,-17,0,12,-27])
        self.assertLess(plan["geometry"]["max_flange_shift_m"],.025)
        self.assertLess(max(abs(plan["geometry"]["candidate_link_origin_y_m"][f"link{i}"])
                            for i in range(1,7)),1e-5)
        pose_candidate.verify(plan)
        plan["geometry"]["candidate_joint_deg"][2]=1
        with self.assertRaisesRegex(ValueError,"固定候选"):
            pose_candidate.verify(plan)

    def test_quick_cli_interactive_and_noninteractive_gates(self):
        class FakeAPI:
            calls=[]
            def plan_replay(self,selector,*,mode,speed_percent):
                self.calls.append(("plan",selector,mode,speed_percent))
                return {"plan_path":"plan.json","plan":{"recording_id":"r1",
                        "forward_steps":2,"route_joint_rad":[[0]*7,[0]*7]}}
            def run_replay(self,path):
                self.calls.append(("run",path))
                return {"report_path":"run.json"}
            def recordings(self):
                return [{"recording_id":"r1","path":"r1.csv"}]
        args=parser_create().parse_args(["quick","replay","--mode","point",
                                         "--speed-percent","5"])
        with self.assertRaisesRegex(ValueError,"非交互"):
            quick_dispatch(args,FakeAPI(),is_tty=False)
        api=FakeAPI();api.calls=[]
        answers=iter(["", "取消"])
        with self.assertRaisesRegex(InterruptedError,"用户取消"):
            quick_dispatch(args,api,is_tty=True,input_fn=lambda _:next(answers))
        self.assertEqual(api.calls,[("plan","latest","point",5)])
        approved=parser_create().parse_args(["quick","replay","--recording","latest",
                    "--mode","point","--speed-percent","5","--yes"])
        api.calls=[]
        result=quick_dispatch(approved,api,is_tty=False)
        self.assertEqual(api.calls,[("plan","latest","point",5),("run","plan.json")])
        self.assertEqual(result["replay"]["report_path"],"run.json")

        class TimedAPI:
            def __init__(self): self.calls=[]
            def plan_replay(self,selector,*,mode,frequency_hz,min_time_scale):
                self.calls.append(("plan",selector,mode,frequency_hz,min_time_scale))
                return {"plan_path":"timed.json","plan":{"replay_mode":"timed_move_j",
                        "total_duration_s":60,"waypoints":[{"joint_rad":[0]*7}]}}
            def run_replay(self,path):
                self.calls.append(("run",path))
                return {"report_path":"timed_run.json"}
        timed=TimedAPI()
        missing=parser_create().parse_args(["quick","replay","--recording","latest",
                    "--mode","continuous","--frequency-hz","20","--yes"])
        with self.assertRaisesRegex(ValueError,"--yes 须明确"):
            quick_dispatch(missing,timed,is_tty=False)
        wrong=parser_create().parse_args(["quick","replay","--recording","latest",
                    "--mode","continuous","--speed-percent","5","--frequency-hz","20",
                    "--min-time-scale","6","--yes"])
        with self.assertRaisesRegex(ValueError,"不能传 --speed-percent"):
            quick_dispatch(wrong,timed,is_tty=False)
        self.assertEqual(timed.calls,[])
        approved=parser_create().parse_args(["quick","replay","--recording","latest",
                    "--mode","continuous","--frequency-hz","20",
                    "--min-time-scale","6","--yes"])
        quick_dispatch(approved,timed,is_tty=False)
        self.assertEqual(timed.calls,[('plan','latest','continuous',20,6.0),('run','timed.json')])

        interactive=parser_create().parse_args(["quick","replay","--recording","latest"])
        timed.calls=[]
        answers=iter(["", "开始"])
        quick_dispatch(interactive,timed,is_tty=True,input_fn=lambda _:next(answers))
        self.assertEqual(timed.calls,[('plan','latest','continuous',20,6.0),('run','timed.json')])

    def test_replay_mode_must_be_explicit_outside_interactive_quick(self):
        with self.assertRaises(SystemExit):
            parser_create().parse_args(["replay", "plan", "--recording", "latest"])
        connections = []
        api = LabAPI(robot_factory=lambda _: connections.append("connected"))
        with self.assertRaisesRegex(ValueError, "回放模式必须"):
            api.plan_replay("latest")
        with self.assertRaisesRegex(ValueError, "回放模式必须"):
            api.quick_replay("latest")
        self.assertEqual(connections, [])

    def test_quick_demo_teaches_then_previews_and_confirms_new_replay(self):
        class FakeAPI:
            def __init__(self):
                self.calls = []
            def teach_preflight(self, seconds):
                self.calls.append(("teach_preflight", seconds))
                return {"start_name": "S1", "motion_sent": False}
            def teach_run(self, seconds):
                self.calls.append(("teach_run", seconds))
                return {"recording_id": "new_recording"}
            def plan_replay(self, selector, *, mode, frequency_hz, min_time_scale):
                self.calls.append(("plan_replay", selector, mode, frequency_hz,
                                   min_time_scale))
                return {"plan_path": "new_plan.json",
                        "plan": {"recording_id": selector, "total_duration_s": 60}}
            def run_replay(self, path):
                self.calls.append(("run_replay", path))
                return {"report_path": "new_run.json"}

        args = parser_create().parse_args(["quick", "demo", "--max-seconds", "5"])
        api = FakeAPI()
        answers = iter(["开始", "", "取消"])
        with self.assertRaisesRegex(InterruptedError, "用户取消"):
            quick_dispatch(args, api, is_tty=True, input_fn=lambda _: next(answers))
        self.assertEqual(api.calls, [
            ("teach_preflight", 5.0), ("teach_run", 5.0),
            ("plan_replay", "new_recording", "continuous", 20, 6.0)])

        api = FakeAPI()
        answers = iter(["开始", "", "开始"])
        result = quick_dispatch(args, api, is_tty=True,
                                input_fn=lambda _: next(answers))
        self.assertEqual(api.calls[-1], ("run_replay", "new_plan.json"))
        self.assertEqual(result["teaching"]["recording_id"], "new_recording")
        self.assertEqual(result["replay"]["report_path"], "new_run.json")

    def test_pose_recording_is_read_only_and_writes_units(self):
        class Robot:
            connects=0
            disconnects=0
            def connect(self): self.connects+=1
            def disconnect(self): self.disconnects+=1
            def get_joint_angles(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.1]*7)
            def get_flange_pose(self):
                return SimpleNamespace(timestamp=time.time(),msg=[.3,.1,.5,0,0,0])
            def get_arm_status(self):
                msg=SimpleNamespace(ctrl_mode=1,arm_status=0,teach_status=0,motion_status=0)
                return SimpleNamespace(timestamp=time.time(),msg=msg)
        robot=Robot()
        with tempfile.TemporaryDirectory() as folder:
            result=LabAPI(robot_factory=lambda _:robot).record_poses(
                duration_s=.06,rate_hz=20,output_dir=folder)
            self.assertGreater(result["samples"],0)
            path=Path(result["recording_path"])
            with path.open(newline="",encoding="utf-8") as stream:
                row=next(csv.DictReader(stream))
            self.assertEqual(float(row["joint_1_rad"]),.1)
            self.assertEqual(float(row["x_m"]),.3)
        self.assertEqual(robot.connects,1)
        self.assertEqual(robot.disconnects,1)

    def test_known_start_and_unknown_pose(self):
        catalog=starts.load_starts()
        self.assertEqual(catalog["default_start"], "S1")
        q=catalog["starts"]["S1"]["joint_rad"]
        state={"ready_for_motion":True,"joint_rad":q,
               "flange_pose_m_rad":[.38,0,.57,0,0,0]}
        plan=starts.prepare_start("C2",snapshot=state)
        self.assertEqual(plan["source_name"],"S1")
        self.assertEqual(plan["target_name"],"C2")
        self.assertTrue(plan["one_way_mode"])
        starts.verify_start_plan(plan)
        plan["waypoints"][1]["joint_rad"][0]+=.01
        with self.assertRaisesRegex(ValueError,"目标被修改"):
            starts.verify_start_plan(plan)
        state["joint_rad"]=[x+.1 for x in q]
        with self.assertRaisesRegex(ValueError,"未知姿态"):
            starts.prepare_start("C2",snapshot=state)
        with self.assertRaisesRegex(ValueError,"starts plan/run 统一命名目标入口"):
            starts.prepare_start("S1",snapshot=state)
        with self.assertRaisesRegex(ValueError,"历史倒序 S1 计划已停用"):
            starts.verify_start_plan({"target_name":"S1"})

    def test_site_recovery_plan_is_tied_to_current_low_pose(self):
        from experiments.single_arm.continuous_replay.continuous_session import robot_instance
        config=load_config(DEFAULT_CONFIG)
        q=[0.11604694196510297,0.2293362637120549,0.0075223690760955605,
           -0.3440043955680824,0.028029987787028934,0.015271630954950384,
           -0.18015288539085472]
        state={"ready_for_motion":True,"joint_rad":q,
               "flange_pose_m_rad":robot_instance("can0").fk(q)}

        class Robot:
            def connect(self): pass
            def disconnect(self): pass
            def fk(self, joints): return robot_instance("can0").fk(joints)

        with patch.object(site_recovery,"read_state",return_value=state):
            plan=site_recovery.prepare(robot_factory=lambda _:Robot())
        self.assertEqual(len(plan["targets_joint_rad"]),4)
        site_recovery.verify(plan)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            path=site_recovery.save(plan,root/"plans")
            changed=list(q);changed[0]+=.02
            different={**state,"joint_rad":changed}
            with patch.object(site_recovery,"read_state",return_value=different):
                with self.assertRaisesRegex(RuntimeError,"实时姿态偏离"):
                    site_recovery.run(path,1,robot_factory=lambda _:Robot(),
                                      run_dir=root/"runs",countdown_s=0)
            reports=list((root/"runs").glob("*.json"))
            self.assertEqual(len(reports),1)
            self.assertFalse(json.loads(reports[0].read_text(encoding="utf-8"))["completed"])
        plan["targets_joint_rad"][0][1]+=.01
        with self.assertRaisesRegex(ValueError,"目标被修改"):
            site_recovery.verify(plan)
        state["joint_rad"]=[0]*7
        with patch.object(site_recovery,"read_state",return_value=state):
            with self.assertRaisesRegex(ValueError,"当前不是"):
                site_recovery.prepare(robot_factory=lambda _:Robot())


if __name__ == "__main__":
    unittest.main()
