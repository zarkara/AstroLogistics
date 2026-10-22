# AstroLogistics — Implementation Plan

Builds the testable core described in [design.md](design.md): a database of robots and
status readings, plus SGP4-based prediction of each robot's state at any time.
Each milestone ends with a passing test suite and can be merged on its own.

## Tooling (Milestone 0: project skeleton)

- `pyproject.toml` (Python ≥ 3.11), package `astrologistics/`
- Runtime deps: `sgp4>=2.23`, `numpy`, `sqlalchemy>=2.0`, `alembic`
- Dev deps: `pytest`, `pytest-cov`, `hypothesis`, `ruff`, `mypy`
- GitHub Actions: ruff → mypy → pytest on 3.11 and 3.13
- `tests/conftest.py`: `session` fixture (in-memory SQLite, schema created per test),
  `fixed_clock` fixture, `iss_tle` fixture

**Done when:** `pytest` runs (one placeholder test) and CI is green.

## Milestone 1: Orbit math (no database)

| Task | Module | Tests |
|---|---|---|
| UTC `datetime` ↔ `(jd, fr)`; `Clock` protocol + `SystemClock` / `FixedClock` | `timeutil.py` | round trip to < 1 µs; naive datetimes rejected |
| `rv_to_coe`, `coe_to_rv`, `orbit_normal`, `plane_angle` | `orbit/elements.py` | round trip on random orbits (hypothesis); plane angle of i=0° vs i=90° is 90°; same orbit gives 0 |
| `satrec_from_tle`, `satrec_from_elements`, `propagate(satrec, t)`, `propagate_many` | `orbit/propagate.py` | matches `sgp4` reference output for ISS TLE; `SatrecArray` output equals per-robot loop; error codes passed through |
| `fit_state_vector(r, v, epoch) -> MeanElements` | `orbit/fit.py` | residual < 1 m at epoch for random LEO/MEO; near-circular, near-equatorial, retrograde cases; clear exception when it doesn't converge |

**Done when:** these modules have ≥ 95% test coverage and `mypy --strict` passes.

## Milestone 2: Database

| Task | Detail |
|---|---|
| Models | `Robot`, `Capability`, `robot_capabilities`, `StatusReading` exactly as design §4.2, enums `Status`, `DestinationKind`, `ElementSource` |
| Session | `make_engine(url)`, `session_scope()`; URL from `ASTROLOG_DB_URL` (default `sqlite:///astrolog.db`) |
| Migrations | Alembic initial migration generated from the models |
| Repository | `add_robot`, `get_robot`, `list_robots`, `add_reading`, `latest_reading(robot_id, at)`, `latest_readings(at)` (one query for the whole fleet), `readings_between` |

Tests:
- `latest_reading` returns the reading with the greatest `epoch ≤ at`, ignores later ones.
- `latest_readings` returns one row per robot (check that the query count is 1).
- Duplicate `(robot_id, epoch)` raises; identical re-insert is a no-op.
- Migration test: `alembic upgrade head` on an empty SQLite file produces the same schema
  as `Base.metadata`.

**Done when:** the repository tests pass on SQLite, and on PostgreSQL in CI (service
container), so no SQLite-only behaviour has crept in.

## Milestone 3: Ingestion

`status/ingest.py`: `ingest(session, report: dict, clock) -> StatusReading`

- Accepts JSON reports in either form:
  ```json
  {"robot": "R-01", "epoch": "2026-10-22T12:00:00Z", "status": "BUSY",
   "expected_free_at": "2026-10-22T15:00:00Z",
   "tle": ["1 …", "2 …"],
   "destination": {"kind": "ORBIT", "inclination_deg": 53.0, "raan_deg": 120.0,
                   "sma_km": 6928.0, "eta": "2026-10-22T14:30:00Z"}}
  ```
  or with `"r_km": […], "v_km_s": […]` instead of `"tle"` (→ `fit_state_vector`).
- Fills in all the derived columns (state vector, elements, orbit normal) so every stored
  row is complete.
- Checks the input (§6 of the design): future epoch, unknown robot, bad status/destination
  combinations (e.g. `READY` with an `eta`), fit not converging.

Tests: one test per check; TLE input and vector input of the *same* orbit produce
positions within 1 km of each other 1 h later.

## Milestone 4: Prediction

`status/extrapolate.py`: `predict(session, robot_id, at) -> RobotSnapshot` and
`predict_fleet(session, at) -> list[RobotSnapshot]` (one query + `SatrecArray`).

Scenario tests (all with `FixedClock`):

| # | Setup | Expectation |
|---|---|---|
| 1 | Reading at t₀ | `predict(t₀)` equals the stored vector (< 1 m); `age == 0`, not stale |
| 2 | Same | `predict(t₀ + 90 min)` equals direct SGP4 propagation; `age == 90 min` |
| 3 | Reading at t₀, new reading at t₁ | queries after t₁ use the new reading; queries between t₀ and t₁ still use the old one |
| 4 | BUSY, `expected_free_at = t₀+2h` | status BUSY at t₀+1h, predicted READY at t₀+3h |
| 5 | BUSY, destination ORBIT, `eta = t₀+1h` | at t₀+2h: `stale`, reason `MANEUVER_EXPECTED` |
| 6 | Age > `max_age` | `stale=True`, reason `AGE` |
| 7 | Reading that decays | `sgp4_error ≠ 0`, NaN position, no exception |
| 8 | Query before first reading | `NoReading` |
| 9 | 1 000 robots | `predict_fleet` < 1 s on CI hardware (guards against slow per-robot loops) |

## Milestone 5: CLI and sample data

- `astrolog init-db`, `astrolog ingest <file.json|dir>`, `astrolog robots`,
  `astrolog where <robot|--all> [--at ISO8601]` (table: status, age, stale, altitude,
  inclination, RAAN, destination)
- `examples/fleet/`: ~10 robots across LEO shells and planes, generated by a script
  from public TLEs, so the demo uses realistic orbits
- End-to-end test: `init-db` → `ingest examples/fleet` → `where --all --at …` output matches a stored expected result

## Phase 2 (after M5; planned separately)

1. `orbit/cost.py`: plane change + Hohmann estimate (design §5.3), checked against
   textbook examples
2. `jobs` / `job_requirements` tables; destination `JOB` foreign key
3. `matching/candidates.py`: capability filter → READY at job start → cost ranking
4. Greedy multi-job assignment, then evaluate an auction or Hungarian algorithm
5. Accurate transfer planning (Lambert solver) for the top-N candidates
6. Frame conversion at the edges (TEME → GCRS/ITRS) via `skyfield`

## Risks

| Risk | Mitigation |
|---|---|
| Fit unstable for e≈0 / i≈0 orbits | Non-singular element formulation; dedicated tests in M1 |
| SGP4 accuracy is only ~km/day; not enough for close approach | Fine for screening; design marks stale readings; accurate planning is phase 2 |
| Maneuvering robots make SGP4 wrong straight away | `MANEUVER_EXPECTED` staleness flag; robots expected to report after each burn |
| SQLite and PostgreSQL behaving differently | Run the repository tests against both in CI from M2 |
