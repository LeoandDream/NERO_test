"""F7 Environment：合成 F1 点位、版本文件与原件完整性。"""

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest

from nero_runtime.cli import main
from nero_runtime.environment import load_environment_profile, save_environment_profile
from nero_runtime.waypoints import capture_waypoint, save_waypoint
from test_motion import FIRST, SECOND, START, feedback


class EnvironmentFixture:
    def __init__(self, directory, *, robot_id="synthetic-nero"):
        self.root = Path(directory)
        self.robot_id = robot_id
        self.paths = {}
        self.points = []
        self.routes = []

    def add_point(self, name, role, joints, *, state="validated", snapshot=None):
        path = self.root / "points" / f"{name}.json"
        snap = feedback(joints, 100.0) if snapshot is None else snapshot
        record = capture_waypoint(name, snap, source="F7 synthetic observation")
        save_waypoint(record, path)
        self.paths[name] = path
        self.points.append({"name": name, "role": role, "state": state,
                            "source_waypoint_path": os.path.relpath(path, self.root/"environment"/"site-a"),
                            "source_waypoint_sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

    def add_route(self, route_id, origin, destination, operation, *, state="validated",
                  refs=None, directional=True):
        self.routes.append({"route_id": route_id, "from_point": origin,
                            "to_point": destination,
                            "ordered_point_refs": list(refs or (origin, destination)),
                            "state": state, "directional": directional,
                            "evidence_ref": "synthetic fixture only; no field approval",
                            "allowed_operations": [operation]})

    def data(self):
        return {"schema_version": 1, "environment_id": "site-a",
                "environment_version": "v1", "robot_id": self.robot_id,
                "mount": "left", "tool_id": "bare_flange",
                "points": self.points, "routes": self.routes}

    def save(self):
        path = self.root / "environment" / "site-a" / "v1.json"
        return save_environment_profile(self.data(), path)


class EnvironmentTests(unittest.TestCase):
    def fixture(self, directory):
        f = EnvironmentFixture(directory)
        f.add_point("READY", "ready", START)
        f.add_point("PARK", "park", FIRST)
        f.add_point("WORK", "waypoint", SECOND)
        f.add_route("park-ready", "PARK", "READY", "start")
        f.add_route("ready-park", "READY", "PARK", "park")
        f.add_route("work-ready", "WORK", "READY", "return_ready")
        return f

    def test_f1_files_to_versioned_profile_and_offline_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            path = f.save()
            profile = load_environment_profile(path)
            self.assertEqual((profile.environment_id, profile.environment_version,
                              profile.robot_id, profile.mount, profile.tool_id),
                             ("site-a", "v1", "synthetic-nero", "left", "bare_flange"))
            self.assertEqual(profile.point("READY").joint_rad, START)
            self.assertEqual(profile.route("ready-park").ordered_point_refs,
                             ("READY", "PARK"))
            with self.assertRaises(FileExistsError):
                f.save()
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = main(["environment", "show", str(path)],
                            source_factory=lambda _: self.fail("device opened"))
            self.assertEqual((code, err.getvalue()), (0, ""))
            self.assertIn("Profile asset state does not grant", out.getvalue())

    def test_waypoint_hash_and_identity_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            path = f.save()
            f.paths["READY"].write_text(f.paths["READY"].read_text() + " ")
            with self.assertRaisesRegex(ValueError, "SHA256"):
                load_environment_profile(path)
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            f.data()
            f.points[0]["source_waypoint_sha256"] = hashlib.sha256(f.paths["PARK"].read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError, "SHA256"):
                f.save()
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            f.robot_id = "other"
            with self.assertRaisesRegex(ValueError, "identity"):
                f.save()

    def test_missing_refs_direction_and_duplicate_ids_rejected(self):
        for mutation in (lambda f: f.routes[0].update(ordered_point_refs=["PARK", "LOST", "READY"]),
                         lambda f: f.routes[0].update(ordered_point_refs=["READY", "PARK"]),
                         lambda f: f.routes[1].update(route_id="park-ready")):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory)
                mutation(f)
                with self.assertRaises(ValueError):
                    f.save()

    def test_strict_schema_and_no_boolean_shortcut(self):
        for kind in ("unknown", "safe", "operation"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory)
                data = f.data()
                if kind == "unknown":
                    data["unknown"] = True
                elif kind == "safe":
                    f.routes[0]["safe"] = True
                else:
                    f.routes[0]["allowed_operations"] = ["start", "force"]
                with self.assertRaises(ValueError):
                    save_environment_profile(data, f.root/"environment"/"site-a"/"v1.json")

    def test_role_name_does_not_auto_validate(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            f.points[0]["state"] = "candidate"
            profile = load_environment_profile(f.save())
            self.assertEqual(profile.point("READY").role, "ready")
            self.assertEqual(profile.point("READY").state, "candidate")


if __name__ == "__main__":
    unittest.main()
