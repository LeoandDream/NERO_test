"""F8：CLI 帮助、离线资产路径和未晋升命令边界。"""

import argparse
from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest

from nero_runtime.cli import main, parser_create
from test_environment import EnvironmentFixture
from test_motion import FIRST, START
import test_replay


class RuntimeCLITests(unittest.TestCase):
    def call_offline(self, argv):
        out, err = io.StringIO(), io.StringIO()

        def forbidden_source(_channel):
            raise AssertionError("offline CLI opened device source")

        with redirect_stdout(out), redirect_stderr(err):
            code = main(argv, source_factory=forbidden_source)
        self.assertEqual((code, err.getvalue()), (0, ""), argv)
        self.assertTrue(out.getvalue(), argv)
        return out.getvalue()

    def test_help_labels_connection_and_locked_boundary(self):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            with self.assertRaises(SystemExit) as raised:
                main(["--help"], source_factory=lambda _: self.fail("device opened"))
        self.assertEqual(raised.exception.code, 0)
        self.assertEqual(err.getvalue(), "")
        for text in ("READ-ONLY DEVICE", "status", "waypoint capture",
                     "trajectory record", "OFFLINE", "route show",
                     "environment show", "LOCKED / NOT PROMOTED",
                     "replay execution", "disable", "reset"):
            self.assertIn(text, out.getvalue())

    def test_no_execution_command_or_run_flag(self):
        parser = parser_create()
        actions = [action for action in parser._actions]
        groups = next(action for action in actions
                      if isinstance(action, argparse._SubParsersAction))
        self.assertEqual(set(groups.choices),
                         {"status", "waypoint", "route", "trajectory", "replay",
                          "environment"})
        for name, subparser in groups.choices.items():
            nested = [action for action in subparser._actions
                      if isinstance(action, argparse._SubParsersAction)]
            for action in nested:
                self.assertFalse({"run", "start", "stop", "enter", "exit",
                                  "execute", "disable", "reset"} & set(action.choices), name)
            self.assertFalse(any("--run" in action.option_strings
                                 for action in subparser._actions), name)

    def test_every_show_and_list_branch_uses_local_synthetic_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = EnvironmentFixture(directory)
            fixture.add_point("READY", "ready", START)
            fixture.add_point("PARK", "park", FIRST)
            fixture.add_route("ready-park", "READY", "PARK", "park")
            environment_path = fixture.save()
            ready_path = fixture.paths["READY"]
            park_path = fixture.paths["PARK"]
            recording_path, _ = test_replay.ReplayTests().make_recording(directory)
            cases = (
                (["waypoint", "show", str(ready_path)], "READY"),
                (["waypoint", "list", str(ready_path.parent)], "READY"),
                (["route", "show", str(ready_path), str(park_path)], "目标 1"),
                (["trajectory", "show", str(recording_path)], "replay-1"),
                (["trajectory", "list", str(Path(directory))], "replay-1"),
                (["replay", "show", str(recording_path)], "Replay"),
                (["environment", "show", str(environment_path)], "site-a"),
            )
            for argv, expected in cases:
                with self.subTest(argv=argv):
                    self.assertIn(expected, self.call_offline(argv))


if __name__ == "__main__":
    unittest.main()
