# AstroLogistics — Implementation Plan

Builds the testable core described in [design.md](design.md): a database of orbital objects
(robots, passive objects, parking orbits), element-set history and robot status reports, plus
SGP4-based prediction of each robot's state at any time. Each milestone ends with a passing test suite and can be merged on its own.

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
| `ElementSet` value type ↔ `Satrec`; parse/validate TLE; read OMM via `sgp4.omm` | `orbit/elementset.py`, `orbit/tle.py` | bad checksum / malformed line rejected; epoch parsed correctly (incl. 2-digit year rollover 57→1957, 56→2056); TLE and OMM of the same object give identical `Satrec` output; OMM with a 6-digit catalog number works |
| Parking orbit (altitude, i, Ω, e, slot, epoch) → `ElementSet` | `orbit/parking.py` | mean predicted altitude over 1 day within 2 km of the requested value (instantaneous altitude swings ≈ ±8 km from Earth's oblateness, so don't test single points); inclination matches to < 0.1°; RAAN drifts in the expected direction (westward for prograde orbits); rejects < 160 km or e ≥ 1 |
| `propagate(satrec, t) -> OrbitState` | `orbit/propagate.py` | matches `sgp4` library output; error codes passed through as warnings |
| `orbit_normal`, `plane_angle`, `altitude_km`, `inclination_raan(r, v)` | `orbit/elements.py` | plane angle of i=0° vs i=90° is 90°; same orbit → 0; inclination taken from the predicted state matches the TLE's inclination to < 0.1° |

**Done when:** ≥ 95% coverage on these modules; `mypy --strict` passes.

## Milestone 2: Database

| Task | Detail |
|---|---|
| Models | `OrbitalObject`, `ElementSet`, `Robot`, `Capability`, `robot_capabilities`, `StatusReport` as in design §5.2, with the check constraints |
| Session | `make_engine(url)`, `session_scope()`; URL from `ASTROLOG_DB_URL` (default `sqlite:///astrolog.db`) |
| Migrations | Alembic initial migration |
| Repository | `add_object`, `add_robot`, `add_elements`, `add_status`, `latest_elements(object, at)`, `latest_status(robot, at)`, `last_arrival(robot, at)`, `list_robots` |

Tests:
- `latest_elements` / `latest_status` return the newest row with time ≤ `at` and ignore later ones.
- `last_arrival` finds the most recent report with destination and `eta ≤ at`, even when later reports have no destination.
- Check constraints: BUSY without `expected_free_at` fails; READY with destination fails; `PARKING` with a catalog number fails; `ROBOT`/`PASSIVE` without one fails.
- Duplicate key: identical re-insert is a no-op; different content raises.
- `alembic upgrade head` on an empty SQLite file produces the same schema as the models.

**Done when:** repository tests pass on SQLite (and on PostgreSQL in CI, using a service container).

## Milestone 3: Ingestion

| Module | Input | Notes |
|---|---|---|
| `ingest/tracking.py` | Text file of TLEs (3-line format with name, or 2-line), or OMM JSON | Loads many at once; reports accepted/rejected counts with reasons; unknown catalog numbers rejected |
| `ingest/status.py` | JSON status report | Example below; enforces that the destination is `PASSIVE` or `PARKING` |
| `ingest/parking.py` | Parking orbit definition (JSON or CLI) | Creates the `PARKING` object and its element set; redefining adds a new element set |

```json
{"robot": "R-01", "reported_at": "2026-10-22T12:00:00Z", "status": "BUSY",
 "expected_free_at": "2026-10-23T06:00:00Z",
 "destination": {"catalog_number": 48274}, "eta": "2026-10-22T20:00:00Z"}
```
A move to a parking orbit uses `"destination": {"parking": "STAGE-LEO-53"}`.

Tests: one per rule in design §6.3; loading the same file twice changes nothing.

## Milestone 4: Prediction

`status/predict.py`: `predict(session, robot_id, at) -> RobotSnapshot`, `predict_fleet(session, at)`.

Scenario tests (all with `FixedClock`; T = robot element-set epoch):

| # | Setup | Expectation |
|---|---|---|
| 1 | Robot elements at T, READY status | position at T+6h equals direct SGP4 output; `orbit_basis=OWN`; no warnings |
| 2 | Robot elements at T and T+1d | queries before T+1d use the first, after use the second |
| 3 | BUSY, `expected_free_at = T+2h` | BUSY at T+1h; READY (`status_predicted`) at T+3h |
| 4 | BUSY → passive object D, `eta = T+4h`, no newer robot elements | at T+2h: robot on own elements, `destination` filled; at T+5h: `orbit_basis=ARRIVED_AT`, robot position == D's position |
| 5 | Same as 4, plus robot elements at T+6h | at T+7h: back to `OWN` |
| 5a | BUSY → parking orbit P, `eta = T+4h`; then READY report with no destination at T+5h | at T+8h: status READY, `orbit_basis=ARRIVED_AT`, `located_at=P`, position == P's slot |
| 5b | Two robots parked in P | both placed at P's slot |
| 6 | Robot elements 4 days old | `STALE_ELEMENTS` warning; parking orbit elements never stale |
| 7 | Status 2 days old | `STALE_STATUS` warning |
| 8 | Decayed / invalid orbit | `SGP4_ERROR`, NaN position, no exception |
| 9 | No status report | status `UNKNOWN`, orbit still predicted |
| 10 | No robot elements before `at` | `NoOrbitData` |

## Milestone 5: CLI and sample data

- `astrolog init-db`
- `astrolog add-object <catalog> <name> --robot|--passive [--capability …]`
- `astrolog add-parking <name> --alt-km … --inc-deg … --raan-deg … [--ecc …] [--slot-deg …] [--epoch …]`
- `astrolog ingest-elements <file>` (TLE or OMM), `astrolog ingest-status <file|dir>`
- `astrolog where <robot|--all> [--at ISO8601]`: table of status, altitude, inclination,
  RAAN, elements age, located at / destination, warnings
- `examples/`: ~8 robots, ~5 passive targets and 2 parking orbits, using real published element sets (robots
  stand in for actual satellites), plus status reports
- End-to-end test: init → ingest examples → `where --all --at …` output matches a stored expected result

## Phase 2 (after M5)

1. `orbit/cost.py`: plane change + Hohmann estimate (design §6.2), checked against textbook examples
2. `jobs` / `job_requirements` tables (job = target object + required capabilities + start time)
3. `matching/candidates.py`: capability filter → READY at start → cost ranking → `astrolog candidates <job>`
4. Staging: suggest a parking orbit for expected future work and which robot to send there
5. Live tracking feed adapter (depends on open question 1)
6. Greedy multi-job assignment

## Risks

| Risk | Mitigation |
|---|---|
| Robot elements lag behind maneuvers (tracking networks need several passes after a burn) | Use the destination's elements after arrival (design §6.1 rule 4); robot appears slightly wrong while in transit, acceptable for ranking |
| Tracking feed outages | Staleness warnings; predictions keep working on old data |
| SQLite and PostgreSQL behaving differently | Run the repository tests against both in CI from M2 |
