# AstroLogistics — Design

Status: draft v0.2 · Language: Python 3.11+ · Propagator: [`sgp4`](https://pypi.org/project/sgp4/) (≥ 2.23)

## 1. Purpose

Assign work to orbital robots the way land and sea logistics assign work to vehicles, but
using orbital mechanics instead of road distance. Given a job (an orbital object to service
plus the tools needed), find available robots that have the tools, rank them by how cheap
they are to get there, and propose an assignment.

This first version is about **knowing roughly where every robot and target object is, and
whether each robot is free, at any moment**, from data that arrives irregularly. Ranking and
assignment build on that.

## 2. Assumptions (decided)

| # | Assumption | Effect on the design |
|---|---|---|
| A1 | Orbit data comes from a **tracking network** as TLEs (two-line element sets) | TLE is the only orbit input; no fitting from raw position/velocity vectors |
| A2 | Predictions only need to be **approximate**: good enough to rank the nearest robot | SGP4's accuracy (≈ 1–3 km/day in low Earth orbit) is enough; generous staleness limits |
| A3 | The robot network is **sparse** (tens of robots, not thousands) | Simple per-robot loops; no spatial indexing or batch-performance work |
| A4 | A destination is an **unpowered orbital object** (debris, defunct satellite, depot) tracked by the same network, never another robot | Destinations are catalog objects with their own TLEs; their predicted paths can be trusted because they don't maneuver |

## 3. Scope

| In scope (v0.2) | Out of scope for now |
|---|---|
| Catalog of tracked objects with TLE history | Raw position/velocity input from robots |
| Robot registry, capabilities, status reports | Collision avoidance, conjunction screening |
| Predicted robot and target position at any time (SGP4) | Accurate maneuver planning (Lambert solving, phasing) |
| Busy → ready from expected-free time; robot assumed co-located with its target after arrival | Powered or moving destinations (another robot) |
| Quick first-pass cost estimate (plane change + altitude change) | Multi-job optimal assignment |

## 4. Key concepts

### 4.1 Two independent data streams

Knowing a robot's situation needs two things, and they come from different places at
different times:

| Stream | Source | Contents | Changes when |
|---|---|---|---|
| **Orbit** | Tracking network | TLE per tracked object (robots *and* targets) | New tracking pass, typically every few hours to a day |
| **Status** | Robot operator / fleet control | READY / BUSY / OFFLINE, expected-free time, destination object, ETA | A job starts, ends, or is re-planned |

Both are stored as history; rows are only ever added, never changed. The current picture is
**always derived** by taking the latest row of each stream at or before the query time and
predicting forward:

```
predict(robot, t) = SGP4(latest TLE for robot's catalog number, t)
                  + latest status report for robot
                  + SGP4(latest TLE for destination object, t)   if it has a destination
```

When new data arrives it replaces the old as the basis for predictions; nothing has to be
recalculated or invalidated.

### 4.2 TLEs and SGP4

A TLE holds mean orbital elements at an epoch: inclination, RAAN, eccentricity, argument
of perigee, mean anomaly, mean motion, and a drag term (B*). The `sgp4` library turns a
TLE plus any time into position and velocity:
`Satrec.twoline2rv(line1, line2).sgp4(jd, fr)`. The tracking network may also deliver the
same data as OMM (CCSDS Orbit Mean-Elements Message, JSON/XML); `sgp4.omm` reads that into
the same `Satrec`, so both formats share one path.

### 4.3 Reference frames, units and time

| Item | Convention |
|---|---|
| Frame | **TEME** (True Equator Mean Equinox), SGP4's native frame, used everywhere internally. Robot-to-target comparisons happen in one frame, so no conversion is needed for v0.2. |
| Distance / velocity | km, km/s |
| Angles | Radians internally; degrees only in user-facing input/output |
| Time | Timezone-aware UTC `datetime` at the edges; SGP4 Julian date pairs `(jd, fr)` internally. All database timestamps are UTC. |
| Earth model | WGS72 (the model TLEs are defined against) |

### 4.4 Orbital plane

The "plane" of an orbit is described by **inclination `i`** and **right ascension of the
ascending node `Ω`**, or equally by the unit orbit normal `ĥ = (r × v)/|r × v|`. The angle
between two normals is the plane-change angle Δθ, usually the largest single cost in reaching a
target. The plane changes slowly over time (because Earth's equatorial bulge makes RAAN drift),
so the plane is computed **at query time** from the predicted state rather than stored.

## 5. Data model

### 5.1 Entities

```
TrackedObject 1───* TleRecord
TrackedObject 1───0..1 Robot            (a robot is a tracked object we control)
Robot         1───* StatusReport
StatusReport  *───0..1 TrackedObject    (destination; must NOT be a robot)
Robot         *───* Capability
```

### 5.2 Tables

**`tracked_objects`**: everything the tracking network follows that we care about.

| column | type | notes |
|---|---|---|
| catalog_number | int PK | NORAD catalog number from the tracking network |
| name | text | |
| object_type | enum `ROBOT`, `PASSIVE` | `PASSIVE` = unpowered (debris, defunct satellite, depot) |
| created_at | timestamptz | |

**`tle_records`**: orbit history, rows only ever added.

| column | type | notes |
|---|---|---|
| id | int PK | |
| catalog_number | FK tracked_objects | |
| epoch | timestamptz | parsed from line 1 |
| line1, line2 | text | stored exactly as received, so it is the source of truth |
| inclination, raan, eccentricity, mean_motion | float | parsed copies, for filtering and display only |
| source | text | e.g. `space-track`, `celestrak`, `test` |
| received_at | timestamptz | |

`UNIQUE(catalog_number, epoch)`; index `(catalog_number, epoch DESC)`.

**`robots`**

| column | type | notes |
|---|---|---|
| id | int PK | |
| catalog_number | FK tracked_objects, unique | object_type must be `ROBOT` |
| name | text unique | |

**`capabilities`** (`id`, `code` unique, `description`), e.g. `GRAPPLE`, `WELD`, `REFUEL`;
**`robot_capabilities`** (`robot_id`, `capability_id`).

**`status_reports`**: status history, rows only ever added.

| column | type | notes |
|---|---|---|
| id | int PK | |
| robot_id | FK robots | |
| reported_at | timestamptz | when the status is valid |
| status | enum `READY`, `BUSY`, `OFFLINE` | |
| expected_free_at | timestamptz null | required when BUSY |
| destination_catalog_number | FK tracked_objects null | must be a `PASSIVE` object |
| eta | timestamptz null | expected arrival at destination |
| source | text | |
| received_at | timestamptz | |

`UNIQUE(robot_id, reported_at)`; index `(robot_id, reported_at DESC)`.
Checks: `BUSY` ⇒ `expected_free_at` set; destination set ⇒ `eta` set; `READY` ⇒ no destination.

The rule "a destination must be passive" is enforced in the ingestion layer (§6.3), because it
depends on another table's column.

**Why rows are only ever added:** history comes for free, data loads can be replayed, and
tests set up a scenario just by inserting rows. "Current state" is never stored.

### 5.3 Database technology

SQLAlchemy 2.0 ORM; **SQLite** for development and tests (in-memory, no setup),
**PostgreSQL** as the production target; Alembic for migrations. No SQLite-specific SQL.

## 6. Components

```
astrologistics/
  timeutil.py          UTC ↔ (jd, fr), Clock protocol
  orbit/
    tle.py             parse/validate TLE (and OMM) → Satrec + parsed fields
    elements.py        orbit normal, plane angle, altitude, rv → (i, Ω)
    propagate.py       SGP4 wrapper: (Satrec, t) → OrbitState
    cost.py            first-pass delta-v estimate                           [phase 2]
  db/
    models.py          SQLAlchemy models (§5.2)
    session.py         engine/session factory from config
    repository.py      latest_tle(catalog, at), latest_status(robot, at), add_* helpers
  ingest/
    tracking.py        TLE file / Space-Track-style feed → tle_records
    status.py          status report (JSON) → status_reports
  status/
    predict.py         RobotSnapshot at time t
  matching/
    candidates.py      capability filter + READY-at-t + cost ranking         [phase 2]
  cli.py
```

### 6.1 Prediction (`status/predict.py`)

```python
@dataclass(frozen=True)
class OrbitState:
    r_km: Vec3; v_km_s: Vec3          # TEME at `at`
    inclination: float; raan: float   # from the predicted state
    altitude_km: float
    tle_epoch: datetime
    tle_age: timedelta
    sgp4_error: int                   # 0 = ok

@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: int
    at: datetime
    status: Status
    status_predicted: bool            # True if derived (e.g. BUSY→READY), not reported
    status_age: timedelta
    orbit: OrbitState                 # where we think the robot is
    orbit_basis: Literal["OWN_TLE", "DESTINATION_TLE"]
    destination: OrbitState | None    # where its target is at `at`
    warnings: frozenset[Warning]      # STALE_TLE, STALE_STATUS, SGP4_ERROR, ...
```

Rules for `predict(robot, at)`:

1. Load the latest status report with `reported_at ≤ at`; none → status `UNKNOWN`.
2. Load the robot's latest TLE with `epoch ≤ at`; none → raise `NoOrbitData`.
3. **Status:** `BUSY` and `expected_free_at ≤ at` → `READY` (`status_predicted=True`).
4. **Position after arrival:** if the robot has a destination, `eta ≤ at`, *and* its own
   latest TLE is older than `eta`, the robot has maneuvered since that TLE, so its own TLE
   is no longer valid. Since the target is unpowered and its TLE is reliable, use the
   **destination's** TLE for the robot's position (`orbit_basis = DESTINATION_TLE`).
   As soon as the tracking network publishes a robot TLE newer than `eta`, rule 2 takes over again.
5. **Before arrival** (in transit, `reported_at < at < eta`): the robot's own TLE is used
   as-is. This is approximate while the robot is maneuvering; acceptable under A2.
6. **Staleness** (warnings, not errors):
   `STALE_TLE` if TLE age > `max_tle_age` (default 3 days);
   `STALE_STATUS` if status age > `max_status_age` (default 24 h).
7. SGP4 error code (e.g. decayed orbit) → `SGP4_ERROR` warning, position NaN; never raises.

`predict_fleet(at)` is a simple loop over robots (A3).

### 6.2 First-pass cost (`orbit/cost.py`, phase 2)

Cheap screening, enough to rank the nearest robot (A2):

- Plane change: `Δv_plane = 2·v·sin(Δθ/2)`, where Δθ is the angle between the robot's
  and target's orbit normals at the job time.
- Altitude change: Hohmann transfer `Δv` between the two mean radii (circular approximation).
- Estimate = `Δv_plane + Δv_hohmann` (an upper bound; ignores phasing and combined burns).

Candidates: robots that have the required capabilities, predicted `READY` at the job's
start time, and no `SGP4_ERROR`, sorted by estimate. Stale data is shown, not hidden.

### 6.3 Ingestion

| Situation | Behaviour |
|---|---|
| Bad TLE (checksum, format, epoch parse) | Rejected with reason |
| TLE for unknown catalog number | Rejected (objects are registered explicitly) |
| Duplicate `(catalog_number, epoch)` / `(robot_id, reported_at)` | Same content → no-op; different content → rejected |
| Status/TLE time more than 60 s in the future | Rejected |
| Destination is a `ROBOT` object, or unknown | Rejected |
| Non-UTC / naive datetimes | Rejected at the edges |

## 7. Testing strategy

- **Unit**: TLE parsing (checksum, epoch), plane angle for known cases, time conversions.
- **SGP4 wrapper**: matches the `sgp4` library's own output for known TLEs.
- **Repository**: in-memory SQLite per test; latest-row selection for both streams, uniqueness.
- **Prediction scenarios**: an injected `Clock`, so tests choose "now"; cover each rule in §6.1.
- **Test data**: fixed TLEs checked into `tests/data/`, so tests never need the network.
- CI: `ruff`, `mypy --strict`, `pytest` with coverage.

## 8. Open questions

1. Which tracking-network feed, and how is access arranged (Space-Track account, CelesTrak
   public data, or an internal feed)? Affects only `ingest/tracking.py`.
2. Where do status reports come from (operator UI, fleet control API, file drop)?
3. Should destinations also allow a fixed *orbit* (e.g. a parking orbit) with no object in it?
   Currently not supported.
