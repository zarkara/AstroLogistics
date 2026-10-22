# AstroLogistics — Implementation Plan

Builds the testable core described in [design.md](design.md): a database of tracked objects,
TLE history and robot status reports, plus SGP4-based prediction of each robot's state at any
time. Each milestone ends with a passing test suite and can be merged on its own.

## Milestone 0: Project skeleton

- `pyproject.toml` (Python ≥ 3.11), package `astrologistics/`
- Runtime deps: `sgp4>=2.23`, `numpy`, `sqlalchemy>=2.0`, `alembic`
- Dev deps: `pytest`, `pytest-cov`, `ruff`, `mypy`
- GitHub Actions: ruff → mypy → pytest
- `tests/conftest.py`: `session` fixture (in-memory SQLite, schema per test), `fixed_clock`
  fixture, TLE fixtures loaded from `tests/data/`

**Done when:** `pytest` runs and CI is green.

## Milestone 1: Orbit basics (no database)

| Task | Module | Tests |
|---|---|---|
| UTC `datetime` ↔ `(jd, fr)`; `Clock` protocol + `SystemClock` / `FixedClock` | `timeutil.py` | round trip to < 1 µs; naive datetimes rejected |
| Parse/validate TLE → `Satrec` + epoch + parsed elements; OMM via `sgp4.omm` | `orbit/tle.py` | bad checksum / malformed line rejected; epoch parsed correctly (incl. 2-digit year rollover 57→1957, 56→2056); TLE and OMM of the same object give identical `Satrec` output |
| `propagate(satrec, t) -> OrbitState` | `orbit/propagate.py` | matches `sgp4` library output; error codes passed through as warnings |
| `orbit_normal`, `plane_angle`, `altitude_km`, `inclination_raan(r, v)` | `orbit/elements.py` | plane angle of i=0° vs i=90° is 90°; same orbit → 0; inclination taken from the predicted state matches the TLE's inclination to < 0.1° |

**Done when:** ≥ 95% coverage on these modules; `mypy --strict` passes.

## Milestone 2: Database

| Task | Detail |
|---|---|
| Models | `TrackedObject`, `TleRecord`, `Robot`, `Capability`, `robot_capabilities`, `StatusReport` as in design §5.2, with the check constraints |
| Session | `make_engine(url)`, `session_scope()`; URL from `ASTROLOG_DB_URL` (default `sqlite:///astrolog.db`) |
| Migrations | Alembic initial migration |
| Repository | `add_object`, `add_robot`, `add_tle`, `add_status`, `latest_tle(catalog, at)`, `latest_status(robot, at)`, `list_robots` |

Tests:
- `latest_tle` / `latest_status` return the newest row with time ≤ `at` and ignore later ones.
- Check constraints: BUSY without `expected_free_at` fails; READY with destination fails.
- Duplicate key: identical re-insert is a no-op; different content raises.
- `alembic upgrade head` on an empty SQLite file produces the same schema as the models.

**Done when:** repository tests pass on SQLite (and on PostgreSQL in CI, using a service container).

## Milestone 3: Ingestion

| Module | Input | Notes |
|---|---|---|
| `ingest/tracking.py` | Text file of TLEs (3-line format with name, or 2-line), or OMM JSON | Loads many at once; reports accepted/rejected counts with reasons; unknown catalog numbers rejected |
| `ingest/status.py` | JSON status report | Example below; enforces that the destination is `PASSIVE` |

```json
{"robot": "R-01", "reported_at": "2026-10-22T12:00:00Z", "status": "BUSY",
 "expected_free_at": "2026-10-23T06:00:00Z",
 "destination": 48274, "eta": "2026-10-22T20:00:00Z"}
```

Tests: one per rule in design §6.3; loading the same file twice changes nothing.

## Milestone 4: Prediction

`status/predict.py`: `predict(session, robot_id, at) -> RobotSnapshot`, `predict_fleet(session, at)`.

Scenario tests (all with `FixedClock`; T = robot TLE epoch):

| # | Setup | Expectation |
|---|---|---|
| 1 | Robot TLE at T, READY status | position at T+6h equals direct SGP4 output; `orbit_basis=OWN_TLE`; no warnings |
| 2 | Robot TLEs at T and T+1d | queries before T+1d use the first, after use the second |
| 3 | BUSY, `expected_free_at = T+2h` | BUSY at T+1h; READY (`status_predicted`) at T+3h |
| 4 | BUSY → destination D, `eta = T+4h`, no newer robot TLE | at T+2h: robot on own TLE, `destination` filled; at T+5h: `orbit_basis=DESTINATION_TLE`, robot position == D's position |
| 5 | Same as 4, plus robot TLE at T+6h | at T+7h: back to `OWN_TLE` |
| 6 | TLE 4 days old | `STALE_TLE` warning |
| 7 | Status 2 days old | `STALE_STATUS` warning |
| 8 | Decayed / invalid orbit | `SGP4_ERROR`, NaN position, no exception |
| 9 | No status report | status `UNKNOWN`, orbit still predicted |
| 10 | No TLE before `at` | `NoOrbitData` |

## Milestone 5: CLI and sample data

- `astrolog init-db`
- `astrolog add-object <catalog> <name> --robot|--passive [--capability …]`
- `astrolog ingest-tle <file>`, `astrolog ingest-status <file|dir>`
- `astrolog where <robot|--all> [--at ISO8601]`: table of status, altitude, inclination,
  RAAN, TLE age, destination, warnings
- `examples/`: ~8 robots and ~5 passive targets using real published TLEs (robots
  stand in for actual satellites), plus status reports
- End-to-end test: init → ingest examples → `where --all --at …` output matches a stored expected result

## Phase 2 (after M5)

1. `orbit/cost.py`: plane change + Hohmann estimate (design §6.2), checked against textbook examples
2. `jobs` / `job_requirements` tables (job = target object + required capabilities + start time)
3. `matching/candidates.py`: capability filter → READY at start → cost ranking → `astrolog candidates <job>`
4. Live tracking feed adapter (depends on open question 1)
5. Greedy multi-job assignment

## Risks

| Risk | Mitigation |
|---|---|
| Robot TLEs lag behind maneuvers (tracking networks need several passes after a burn) | Use the destination's TLE after arrival (design §6.1 rule 4); robot appears slightly wrong while in transit, acceptable for ranking |
| TLE feed outages | Staleness warnings; predictions keep working on old data |
| SQLite and PostgreSQL behaving differently | Run the repository tests against both in CI from M2 |
