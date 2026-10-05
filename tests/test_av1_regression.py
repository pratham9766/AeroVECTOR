"""Exact-output regression lock for the three AV0 reference scenarios."""

from __future__ import annotations

import hashlib
from pathlib import Path
import unittest


BASELINES = Path(__file__).resolve().parent / "baselines"
EXPECTED = {
    "dhruva_dual_motor_sitl": {
        "aerodynamics.json": "a559115c717f1ce528009a0ce36c41c1a9380d84e5981ab8cb0971f962cc1323",
        "trace.csv": "6fb72dd956b975a6aef881c69ebb2a6b4ff88bee1f894b83191364596c4e53bd",
    },
    "legacy_active_fins": {
        "aerodynamics.json": "def7c97dbc9bd1d7903b8459a8af5cefd95262e3d1bd74b5b69988072d4cc6a8",
        "trace.csv": "e51c281f76aec188a8acdf9e69ef5d302925c5067e4b120895406119cbfa61cc",
    },
    "legacy_tvc": {
        "aerodynamics.json": "29ee037ff1e905bd1f3e67ba8f5d879c5be3be2f69ba416cd0dbed0133d90b29",
        "trace.csv": "64bc3e3156a0d006199fa3dfb068bcf5f3d0f335d2b9f62b7e124b4e6d7eb0f3",
    },
}


class AV1RegressionTests(unittest.TestCase):
    def test_frozen_av0_numerical_artifacts_match_exactly(self) -> None:
        for scenario, files in EXPECTED.items():
            for name, expected in files.items():
                with self.subTest(scenario=scenario, file=name):
                    actual = hashlib.sha256((BASELINES / scenario / name).read_bytes()).hexdigest()
                    self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
