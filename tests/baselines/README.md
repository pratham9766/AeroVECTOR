# AV0 baseline fixtures

These fixtures capture the behavior of the existing AeroVECTOR implementation
before any AV1+ model or solver work. They are evidence, not declarations that
the current results are physically correct.

Run from the repository root with the existing environment:

```powershell
.\.venv\Scripts\python.exe .\tests\baselines\capture_av0_baselines.py
```

The harness pins Python's random seed to `20261003`, copies each exact input
configuration, records its SHA-256 hash and the Git revision, writes the final
state and detected events to `summary.json`, stores a legacy-playback-cadence
trace in `trace.csv`, and renders an altitude/vertical-velocity plot.

The harness deliberately preserves failed runs. A failure is an AV0 baseline
finding and must not be silently converted into a passing fixture.
