"""六面分段拖动入口：实测时间窗选点、失败收尾与无运动发布。"""

import csv
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from experiments.single_arm.lab import cube
from experiments.single_arm.lab.cube_face_sweep import (
    run_face_sweep, select_dwell_triplet,
)
from experiments.single_arm.lab.cli import parser_create, quick_dispatch
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG


class CubeFaceSweepTest(unittest.TestCase):
    def test_noninteractive_face_sweep_rejects_missing_parameters_before_status(self):
        class API:
            def status(self):
                raise AssertionError("参数缺失时不得连接设备")

        args=parser_create().parse_args(["quick","cube-faces","--yes"])
        with self.assertRaisesRegex(ValueError,"--yes 须明确"):
            quick_dispatch(args,API(),is_tty=False)

    def make_file(self, path, face):
        axis = "XYZ".index(face[1])
        sign = 1 if face[0] == "+" else -1
        other = [i for i in range(3) if i != axis]
        center = [.35, 0, .50]
        points = []
        for u, v in ((-.05,-.05),(.05,-.05),(0,.05)):
            xyz = list(center)
            xyz[axis] += sign*.10
            xyz[other[0]] += u
            xyz[other[1]] += v
            points.append(xyz)
        fieldnames = ["sample_index","captured_at_utc","elapsed_s",
                      "joint_feedback_unix_s","pose_feedback_unix_s"]
        fieldnames += [f"joint_{i}_rad" for i in range(1,8)]
        fieldnames += ["x_m","y_m","z_m","roll_rad","pitch_rad","yaw_rad"]
        with path.open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=fieldnames)
            writer.writeheader()
            for index in range(24):
                xyz=points[index//8]
                row={"sample_index":index,
                     "captured_at_utc":"2026-09-26T00:00:00+00:00",
                     "elapsed_s":.05*index,
                     "joint_feedback_unix_s":100+.05*index,
                     "pose_feedback_unix_s":100+.05*index}
                row.update({f"joint_{i}_rad":0.1 for i in range(1,8)})
                row.update(dict(zip(("x_m","y_m","z_m"),xyz)))
                row.update({"roll_rad":0,"pitch_rad":0,"yaw_rad":0})
                writer.writerow(row)
        return path

    def test_six_labeled_trace_segments_fit_geometry_without_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            sources=[self.make_file(root/(face.replace("+","p").replace("-","m")+".csv"),face)
                     for face in cube.FACES]

            class API:
                config_path=DEFAULT_CONFIG
                index=0
                def status(self):
                    raise AssertionError("六面入口不应在拖动开始命令之外重复检查 CAN 模式")
                def record_poses(self,*,duration_s,rate_hz,required_state):
                    self.assertion=(duration_s,rate_hz,required_state)
                    source=sources[self.index]
                    self.index+=1
                    return {"recording_path":str(source),"samples":24,
                            "sha256":hashlib.sha256(source.read_bytes()).hexdigest()}

            calls=[]
            def runner(command,**kwargs):
                calls.append(command)
                return SimpleNamespace(returncode=0,stdout="ok",stderr="")

            api=API()
            result=run_face_sweep(api,seconds_per_face=3,rate_hz=20,
                    command_runner=runner,report_dir=root/"reports",
                    session_dir=root/"sessions")
            self.assertEqual(api.assertion,(3,20,"drag"))
            self.assertEqual(api.index,6)
            self.assertEqual(len(calls),12)
            for index in range(6):
                self.assertIn("--start",calls[2*index])
                self.assertIn("--stop",calls[2*index+1])
            self.assertTrue(result["geometry_valid"])
            self.assertFalse(json.loads(Path(result["report_path"]).read_text())[
                "motion_region_verified"])
            session=Path(result["session_path"])
            self.assertEqual(len(list(session.glob("sample_*.json"))),18)
            self.assertAlmostEqual(json.loads(Path(result["fit_path"]).read_text())[
                "side_length_m"],.2,places=4)
            calls.clear()
            result=run_face_sweep(api,seconds_per_face=3,rate_hz=20,
                    command_runner=runner,report_dir=root/"reports",
                    session_path=session)
            self.assertTrue(result["geometry_valid"])
            self.assertEqual(calls,[])
            self.assertEqual(len(list(session.glob("sample_*.json"))),18)

    def test_no_stationary_face_points_is_not_six_face_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            path=self.make_file(Path(folder)/"face.csv","+X")
            with path.open(newline="",encoding="utf-8") as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(select_dwell_triplet(rows)),3)
            for row in rows:
                row["x_m"]=str(.1+int(row["sample_index"])*.004)
            self.assertEqual(select_dwell_triplet(rows),[])

    def test_cancel_before_first_face_does_not_enter_drag(self):
        class API:
            config_path=DEFAULT_CONFIG
            def status(self): return {"ready_for_motion":True,"reasons":[]}
            def record_poses(self,**kwargs):
                raise AssertionError("取消后不得采样")

        calls=[]
        def runner(command,**kwargs):
            calls.append(command)
            return SimpleNamespace(returncode=0,stdout="ok",stderr="")

        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            with self.assertRaisesRegex(RuntimeError,"已采文件保留"):
                run_face_sweep(API(),seconds_per_face=3,rate_hz=20,
                    phase_gate=lambda face: (_ for _ in ()).throw(
                        InterruptedError("已采文件保留")),
                    command_runner=runner,report_dir=root/"reports",
                    session_dir=root/"sessions")
            self.assertEqual(calls,[])
            report=json.loads(next((root/"reports").glob("*.json")).read_text())
            self.assertFalse(report["completed"])
            self.assertEqual(report["error_type"],"InterruptedError")

    def test_feedback_failure_stops_drag_before_next_face(self):
        class API:
            config_path=DEFAULT_CONFIG
            def status(self): return {"ready_for_motion":True,"reasons":[]}
            def record_poses(self,**kwargs):
                raise RuntimeError("连续一秒没有有效反馈")

        calls=[]
        def runner(command,**kwargs):
            calls.append(command)
            return SimpleNamespace(returncode=0,stdout="ok",stderr="")

        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            with self.assertRaisesRegex(RuntimeError,"连续一秒没有有效反馈"):
                run_face_sweep(API(),seconds_per_face=3,rate_hz=20,
                    command_runner=runner,report_dir=root/"reports",
                    session_dir=root/"sessions")
            self.assertEqual(len(calls),2)
            self.assertIn("--start",calls[0])
            self.assertIn("--stop",calls[1])

    def test_resume_keeps_existing_face_and_only_records_missing_faces(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            sources=[self.make_file(root/(face.replace("+","p").replace("-","m")+".csv"),face)
                     for face in cube.FACES]

            class API:
                config_path=DEFAULT_CONFIG
                index=0
                fail_second=True
                def status(self): return {"ready_for_motion":True,"reasons":[]}
                def record_poses(self,**kwargs):
                    if self.index == 1 and self.fail_second:
                        raise RuntimeError("第二面反馈丢失")
                    source=sources[self.index]
                    self.index+=1
                    return {"recording_path":str(source),"samples":24,
                            "sha256":hashlib.sha256(source.read_bytes()).hexdigest()}

            calls=[]
            def runner(command,**kwargs):
                calls.append(command)
                return SimpleNamespace(returncode=0,stdout="ok",stderr="")

            api=API()
            with self.assertRaisesRegex(RuntimeError,"第二面反馈丢失"):
                run_face_sweep(api,seconds_per_face=3,rate_hz=20,
                    command_runner=runner,report_dir=root/"reports",
                    session_dir=root/"sessions")
            first_report=json.loads(sorted((root/"reports").glob("*.json"))[0].read_text())
            session=Path(first_report["session_path"])
            self.assertEqual(len(list(session.glob("sample_*.json"))),3)
            api.fail_second=False
            calls.clear()
            result=run_face_sweep(api,seconds_per_face=3,rate_hz=20,
                    command_runner=runner,report_dir=root/"reports",
                    session_path=session)
            self.assertTrue(result["geometry_valid"])
            self.assertEqual(api.index,6)
            self.assertEqual(len(calls),10)
            self.assertEqual(len(list(session.glob("sample_*.json"))),18)
            report=json.loads(Path(result["report_path"]).read_text())
            self.assertEqual(report["reused_faces"],["+X"])

    def test_resume_rejects_missing_raw_source_before_device_status(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            session=cube.create_session(DEFAULT_CONFIG,directory=root)
            (session/"sample_invalid.json").write_text(json.dumps({
                "face":"+X", "config_sha256":json.loads((session/"manifest.json")
                    .read_text())["config_sha256"],
                "source_recording":str(root/"missing.csv"),
                "source_sha256":"wrong"}),encoding="utf-8")
            class API:
                config_path=DEFAULT_CONFIG
                def status(self):
                    raise AssertionError("来源坏时不能连接设备")
            with self.assertRaisesRegex(ValueError,"原始录制缺失"):
                run_face_sweep(API(),session_path=session)


if __name__ == "__main__":
    unittest.main()
