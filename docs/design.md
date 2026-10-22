# AstroLogistics — Design

Status: draft v0.1 · Language: Python 3.11+ · Propagator: [`sgp4`](https://pypi.org/project/sgp4/) (≥ 2.23)

## 1. Purpose

Assign work to orbital robots the way land and sea logistics assign work to vehicles, but
using orbital mechanics instead of road distance. Given a job (a location in orbit plus the
tools it needs), find available robots that have the tools, rank them by how cheap they are
to get there, and propose an assignment.

This first version is about **knowing where every robot is and whether it is free, at any
moment**, from status readings that arrive irregularly. Ranking and assignment build on that.

## 2. Scope

| In scope (v0.1) | Out of scope for now |
|---|---|
| Robot registry and status readings in a database | Real-time telemetry ingestion protocols |
| Predicting robot position/velocity at any time from the latest reading (SGP4) | Collision avoidance, conjunction screening |
| Predicting busy → ready from an expected-free time | Detailed maneuver planning (Lambert solving, phasing) |
| Quick first-pass cost estimate (plane change + altitude change) | Multi-job optimal assignment (auction or Hungarian algorithm) |
| Capability (tool) matching | Power, comms, and thermal constraints |

## 3. Key concepts

### 3.1 Readings are snapshots; the system predicts between them

A robot reports a **status reading**: its status, orbital state, and destination, valid at
a specific moment called the *epoch*. Readings are stored and never changed. To answer
"where is robot X at time t?", the system takes the latest reading with epoch ≤ t and
propagates it forward with SGP4. When a new reading arrives it replaces the old one as the
basis for predictions. The old one stays in the table as history.

Every predicted value includes its **age** (`t − epoch`). Predictions older than a set
limit are marked `stale`, so callers can down-rank or ignore them.

### 3.2 Why SGP4, and what that means for the data

SGP4 doesn't propagate a raw position/velocity vector. It propagates **mean orbital
elements**, the same set a TLE (two-line element set) or OMM (CCSDS Orbit Mean-Elements
Message) carries. A reading can therefore arrive in one of two forms:

1. **Element set** (TLE lines or OMM fields). Used directly with `Satrec.twoline2rv` /
   `sgp4.omm`.
2. **State vector** (position + velocity at epoch). Converted to SGP4 mean elements at
   ingestion with an iterative fit (§5.2). The fitted elements are stored alongside the
   original vector.

Either way, each stored reading has both the SGP4 elements (used for prediction) and the
position/velocity at epoch (used for display and checks).

### 3.3 Reference frames, units and time

| Item | Convention |
|---|---|
| Frame | **TEME** (True Equator Mean Equinox), SGP4's native frame. Everything inside the system uses TEME; conversion to GCRS/ITRS happens only at the edges (future: via `skyfield` or `astropy`). |
| Distance / velocity | km, km/s |
| Angles | Radians internally; degrees only in user-facing input/output |
| Time | Timezone-aware UTC `datetime` at the edges; SGP4 Julian date pairs `(jd, fr)` internally. All database timestamps are UTC. |
| Earth model | WGS72 (the model SGP4 element sets are defined against) |

### 3.4 Orbital plane

The "plane" of a robot is described by **inclination `i`** and **right ascension of the
ascending node `Ω`**, or equally by the unit orbit normal `ĥ = (r × v)/|r × v|`. The angle
between two robots' normals is the plane-change angle Δθ, which is usually the largest
single cost in reaching a target. The plane changes slowly over time because of the
J2 perturbation (Earth's equatorial bulge), so it is stored at epoch *and* recalculated at
query time.

## 4. Data model

### 4.1 Entities

```
Robot 1───* StatusReading
Robot *───* Capability          (robot_capabilities)
Job   *───* Capability          (job_requirements)          [phase 2]
StatusReading *───0..1 Job      (destination)               [job FK added in phase 2]
```

### 4.2 Tables

**`robots`**: registry, rarely changes.

| column | type | notes |
|---|---|---|
| id | int PK | |
| name | text unique | |
| norad_id | int null | catalog number, if tracked |
| created_at | timestamptz | |

**`capabilities`**: (`id`, `code` unique, `description`), e.g. `GRAPPLE`, `WELD`, `REFUEL`.

**`robot_capabilities`**: (`robot_id`, `capability_id`) primary key.

**`status_readings`**: append-only; one row per report.

| group | column | type | notes |
|---|---|---|---|
| key | id | int PK | |
| | robot_id | FK robots | indexed with `epoch` |
| time | epoch | timestamptz | when the state is valid |
| | received_at | timestamptz | when we stored it |
| status | status | enum `READY`, `BUSY`, `OFFLINE` | |
| | expected_free_at | timestamptz null | when BUSY: predicted time of becoming READY |
| state vector (TEME) | rx, ry, rz | float | km |
| | vx, vy, vz | float | km/s |
| SGP4 mean elements | inclination, raan, eccentricity, arg_perigee, mean_anomaly | float | rad |
| | mean_motion | float | rad/min (Kozai convention, as `sgp4init` expects) |
| | bstar, ndot, nddot | float | drag terms; 0 when fitted from a vector |
| | tle_line1, tle_line2 | text null | original element set, if that was the input |
| | element_source | enum `TLE`, `OMM`, `FIT` | |
| plane | normal_x, normal_y, normal_z | float | unit `ĥ` at epoch |
| destination | destination_kind | enum `NONE`, `JOB`, `ORBIT`, `ROBOT` | |
| | destination_ref | text null | job id / robot id |
| | dest_inclination, dest_raan, dest_sma_km | float null | for `ORBIT` destinations |
| | eta | timestamptz null | expected arrival |
| | source | text | `telemetry`, `operator`, `test`, … |

Constraint: `UNIQUE(robot_id, epoch)`. Indexes: `(robot_id, epoch DESC)` for "latest
reading", and `(status)`.

**Why append-only?** Readings never change after they're stored, so history comes for
free, ingestion can be replayed, and tests can set up a scenario by inserting rows.
"Current state" is always derived, never stored.

### 4.3 Database technology

SQLAlchemy 2.0 ORM, with **SQLite** for development and tests (in-memory, no setup) and
**PostgreSQL** as the production target. Migrations use Alembic. No SQLite-specific SQL, so
the switch to PostgreSQL is a configuration change.

## 5. Components

```
astrologistics/
  timeutil.py          UTC ↔ (jd, fr), clock abstraction
  orbit/
    elements.py        state vector ↔ classical elements, orbit normal, plane angle
    propagate.py       SGP4 wrapper: Satrec from a stored reading → (r, v) at t; batch via SatrecArray
    fit.py             state vector → SGP4 mean elements (iterative fit)
    cost.py            first-pass delta-v estimate                                   [phase 2]
  db/
    models.py          SQLAlchemy models (§4.2)
    session.py         engine/session factory from config
    repository.py      add_robot, add_reading, latest_reading(robot, at), readings_between
  status/
    ingest.py          check + normalise a raw report → StatusReading
    extrapolate.py     RobotSnapshot at time t from latest reading
  matching/
    candidates.py      capability filter + READY-at-t filter + cost ranking       [phase 2]
  cli.py               `astrolog robots`, `astrolog where <robot> --at …`, `astrolog ingest file.json`
```

### 5.1 Prediction (`status/extrapolate.py`)

```python
@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: int
    at: datetime                      # query time
    reading_epoch: datetime
    age: timedelta                    # at - reading_epoch
    stale: bool                       # age > max_age (config; default 6 h)
    status: Status                    # predicted (see rules)
    r_km: Vec3                        # TEME at `at`
    v_km_s: Vec3
    normal: Vec3                      # orbit normal at `at`
    inclination: float
    raan: float
    destination: Destination
    sgp4_error: int                   # 0 = ok; non-zero = SGP4 error code (e.g. decayed)
```

Rules for `predict(robot, at)`:

1. Load the latest reading with `epoch ≤ at`. If there is none, raise `NoReading`.
2. Build the `Satrec` from the stored elements and propagate to `at`. Use `SatrecArray` for
   fleet-wide queries.
3. Status:
   - `BUSY` and `expected_free_at ≤ at` → predicted `READY` (marked as `predicted=True`).
   - `OFFLINE` stays `OFFLINE`.
   - Otherwise unchanged.
4. **Maneuver caveat:** if the robot is BUSY with `destination_kind = ORBIT` and
   `eta ≤ at`, the robot has presumably moved to its destination orbit, so SGP4 from the
   old orbit is wrong. v0.1 marks the snapshot `stale=True` with reason `MANEUVER_EXPECTED`
   rather than guessing a new orbit.
5. Querying at a time *before* the reading's epoch (propagating backwards) is allowed but
   flagged; normal use only propagates forwards.

### 5.2 State vector → SGP4 elements (`orbit/fit.py`)

1. Convert the reported `(r, v)` to osculating classical elements (actual elements at that
   instant); use them as the first guess for the mean elements.
2. Loop: build a `Satrec` with `sgp4init(WGS72, …)` at the reading's epoch, propagate to
   the epoch, convert the result back to osculating elements, add the difference to the
   guess (angles wrapped to ±π).
3. Stop when the position residual < 1 m (typical: ~10 iterations), or fail after 30.

A prototype matched an ISS state vector to < 1 mm at epoch, and stayed within 1 km of the
reference TLE after 24 h (the difference is drag, because a fit from a single vector sets
`bstar = 0`). Near-circular (`e ≈ 0`) and equatorial (`i ≈ 0`) orbits are numerically
awkward: `fit.py` must handle them by working in non-singular (equinoctial) elements or by
nudging those values slightly, and this needs dedicated tests.

### 5.3 First-pass cost (`orbit/cost.py`, phase 2)

Cheap screening that needs no maneuver planning:

- Plane change: `Δv_plane = 2·v·sin(Δθ/2)` with Δθ = angle between orbit normals.
- Altitude change: Hohmann transfer `Δv` between the two orbit radii (circular approximation).
- Estimate = `Δv_plane + Δv_hohmann` (an upper bound; combining burns is cheaper).
- Phasing (waiting to line up along the orbit) is ignored in the first pass.

Candidates are filtered by capability and by predicted `READY` at the job's start time,
then sorted by this estimate. Only the top N go on to accurate planning in a later phase.

## 6. Error handling

| Situation | Behaviour |
|---|---|
| SGP4 returns error code (decay, eccentricity out of range, …) | Snapshot returned with `sgp4_error ≠ 0`, position `NaN`; never silently dropped |
| Reading with epoch in the future (> 60 s clock skew) | Rejected at ingestion |
| Duplicate `(robot_id, epoch)` | Rejected (same content → no-op, idempotent) |
| Fit does not converge | Reading rejected with a reason; raw payload logged |
| Non-UTC / naive datetimes | Rejected at the API edge |

## 7. Testing strategy

- **Unit**: element conversions round-trip; plane angle for known cases; time conversions.
- **SGP4 wrapper**: matches the reference output of the `sgp4` library itself for known TLEs
  (the library ships Vallado's verification set).
- **Fit**: property-based tests (`hypothesis`) over random LEO/MEO orbits, so that the
  fitted Satrec reproduces the input vector to < 1 m. Separate tests for near-circular,
  near-equatorial and retrograde orbits.
- **Repository**: in-memory SQLite per test; latest-reading selection, history ordering,
  uniqueness.
- **Prediction**: an injected clock (`Clock` protocol), so tests choose "now". Scenario tests
  cover a reading at t₀, queries at t₀+Δ, a new reading at t₁ replacing the basis, BUSY → READY
  after `expected_free_at`, and the maneuver caveat.
- **No network, no wall-clock** in tests. CI: `ruff`, `mypy --strict`, `pytest` with coverage.

## 8. Open questions

1. Where do readings come from in practice: robots reporting their own vectors, or
   tracking-network TLEs? (This affects whether `FIT` or `TLE` is the main path.)
2. Required prediction accuracy and staleness limit per use case (screening vs. rendezvous).
3. Should `destination` be able to point to a moving target (another robot) in v0.1, or only
   to a fixed target orbit?
4. Multi-job assignment: is greedy acceptable at first, or is global optimisation needed?
5. Fleet size (tens vs. thousands) decides whether batch propagation and PostGIS-style
   indexing matter.
