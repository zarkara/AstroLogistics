# AstroLogistics — Design

Status: draft v0.4 · Language: Python 3.11+ · Propagator: [`sgp4`](https://pypi.org/project/sgp4/) (≥ 2.23)

## 1. Purpose

Assign work to orbital robots the way land and sea logistics assign work to vehicles, but
using orbital mechanics instead of road distance. Given a job (an orbital object to service
plus the tools needed), find available robots that have the tools, rank them by how cheap
they are to get there, and propose an assignment.

Robots can also be **staged**: sent ahead of time to a parking orbit close to where future
work is expected, so they are cheap to dispatch when the job arrives (like positioning
trucks near a port before the ship docks). Over time, some parking orbits are expected to
become **conventional parking locations**: long-lived, shared orbits where idle robots wait
between tasks, much like a truck depot or an anchorage.

This first version is about **knowing roughly where every robot, target object and parking
orbit is, and whether each robot is free, at any moment**, from data that arrives
irregularly. Ranking and assignment build on that.

## 2. Assumptions (decided)

| # | Assumption | Effect on the design |
|---|---|---|
| A1 | Orbit data for real objects comes from a **tracking network** as mean element sets (TLE / OMM) | No fitting from raw position/velocity vectors |
| A2 | Predictions only need to be **approximate**: good enough to rank the nearest robot | SGP4's accuracy (≈ 1–3 km/day in low Earth orbit) is enough; generous staleness limits |
| A3 | The robot network is **sparse** (tens of robots, not thousands) | Simple per-robot loops; no spatial indexing or batch-performance work |
| A4 | A destination is either an **unpowered tracked object** (debris, defunct satellite, depot) or a **parking orbit** defined by us; never another robot | Every destination has elements whose prediction can be trusted, because nothing at the destination maneuvers |

## 3. Scope

| In scope (v0.3) | Out of scope for now |
|---|---|
| Catalog of orbital objects (robots, passive objects, parking orbits) with element-set history | Raw position/velocity input from robots |
| Robot registry, capabilities, status reports | Collision avoidance, conjunction screening |
| Defining parking orbits (conventional and staging) with a unique slot per robot | Choosing parking orbits / sending idle robots home automatically (phase 2) |
| Predicted robot and destination position at any time (SGP4) | Accurate maneuver planning (Lambert solving, phasing) |
| Busy → ready from expected-free time; robot assumed at its destination after arrival | Powered or moving destinations (another robot) |
| Quick first-pass cost estimate (plane change + altitude change) | Multi-job optimal assignment |

## 4. Key concepts

### 4.1 Two independent data streams

Knowing a robot's situation needs two things, and they come from different places at
different times:

| Stream | Source | Contents | Changes when |
|---|---|---|---|
| **Orbit** | Tracking network (robots, passive objects); operators (parking orbits) | Mean element set per object | New tracking pass, typically every few hours to a day; parking orbits when (re)defined |
| **Status** | Robot operator / fleet control | READY / BUSY / OFFLINE, expected-free time, destination, ETA | A job or move starts, ends, or is re-planned |

Both are stored as history; rows are only ever added, never changed. The current picture is
**always derived** by taking the latest row of each stream at or before the query time and
predicting forward:

```
predict(robot, t) = SGP4(robot's best element set at t, t)       (see §6.1 rule 4)
                  + latest status report for robot
                  + SGP4(destination's latest element set, t)    if it has a destination
```

When new data arrives it replaces the old as the basis for predictions; nothing has to be
recalculated or invalidated.

### 4.2 Element sets and SGP4

An element set holds mean orbital elements at an epoch: inclination, RAAN, eccentricity,
argument of perigee, mean anomaly, mean motion, and drag terms (B*, ṅ, n̈). It is what a TLE
or an OMM (CCSDS Orbit Mean-Elements Message) carries, and what SGP4 needs. The `sgp4` library
turns one into a `Satrec` and predicts position and velocity at any time:

| Source | How the `Satrec` is built |
|---|---|
| TLE text | `Satrec.twoline2rv(line1, line2)` (limited to 5-digit catalog numbers) |
| OMM (CSV/XML/JSON fields) | `sgp4.omm.initialize(satrec, fields)` (any catalog number) |
| Parking orbit (our own elements) | `Satrec.sgp4init(WGS72, …)` with drag terms = 0 |

All three are stored in one `element_sets` table, so prediction doesn't care where the
elements came from.

### 4.3 Parking orbits and slots

A parking orbit is a **virtual object**: an orbit that we define rather than track. It is
divided into **slots**, evenly spaced positions along the orbit. Each robot parks in its own
slot, so robots sharing an orbit have different positions on it.

| Input | Default | Notes |
|---|---|---|
| name | (required) | e.g. `PARK-LEO-550-53`, `STAGE-GEO-W` |
| purpose | `CONVENTIONAL` | `CONVENTIONAL` = long-lived shared parking for idle robots between tasks; `STAGING` = temporary, near expected work |
| altitude_km (or semi-major axis) | (required) | mean altitude above the WGS72 equatorial radius (6378.135 km) |
| inclination, RAAN | (required) | at the given epoch |
| eccentricity | 0 | near-circular by default |
| argument of perigee | 0 | |
| reference mean anomaly | 0 | position of slot 0 at epoch |
| slot_count | (required) | slots are evenly spaced: slot k is `k × 360°/slot_count` ahead of slot 0 |
| epoch | time of definition | |

The orbit is stored as an orbital object of type `PARKING` with one element set
(`source = defined`) describing slot 0. **Slot k uses the same element set with
`k × 2π/slot_count` added to the mean anomaly.** Because every slot shares the same mean
elements, SGP4 moves them all identically (including the slow RAAN drift caused by Earth's
equatorial bulge), so the spacing between slots never changes and the orbit stays next to real
objects in similar orbits.

**Spacing check.** The distance along the orbit between neighbouring slots,
`2π·a / slot_count`, must be at least `min_slot_spacing_km` (config, default 50 km),
because positions are approximate (A2). Example: 550 km altitude with 100 slots gives ≈ 435 km, and an SGP4 check shows
neighbouring slots stay within 1 km of that spacing over 30 days.

**Occupancy is derived, never stored.** At time t a slot is:

| State | Meaning |
|---|---|
| `OCCUPIED` | a robot's last arrival (§6.1 rule 4) is this slot, and it hasn't reported a move elsewhere since |
| `RESERVED` | a robot is BUSY on its way to this slot (`eta > t`) |
| `FREE` | neither |

A move to a slot that is `OCCUPIED` or `RESERVED` by another robot at the move's `eta` is
rejected at ingestion. That is how "one robot per slot" is enforced.

**Changing a parking orbit.** `slot_count` is fixed when the orbit is created. Updating the
orbit's elements (e.g. to fine-tune it) adds a new element set and keeps the slots. A different
slot layout means defining a new parking orbit and moving robots across. A parking orbit can be
`retired`: no new arrivals, existing occupants stay until moved.

**Conventional parking orbits.** These are expected to be few and long-lived, chosen so that
reaching common work areas is cheap. Two consequences for the design:

- Idle robots will cluster in a handful of orbits. All robots in one parking orbit share the
  same first-pass cost to a target (§6.2), so the cost is calculated once per parking orbit.
  Slot position (phase along the orbit) only matters for the more accurate planning in phase 2.
- "Send this robot to the nearest free conventional slot" becomes a routine operation after
  each task (phase 2).

### 4.4 Reference frames, units and time

| Item | Convention |
|---|---|
| Frame | **TEME** (True Equator Mean Equinox), SGP4's native frame, used everywhere internally. Robot-to-target comparisons happen in one frame, so no conversion is needed for v0.3. |
| Distance / velocity | km, km/s |
| Angles | Radians internally; degrees only in user-facing input/output |
| Time | Timezone-aware UTC `datetime` at the edges; SGP4 Julian date pairs `(jd, fr)` internally. All database timestamps are UTC. |
| Earth model | WGS72 (the model SGP4 element sets are defined against) |

### 4.5 Orbital plane

The "plane" of an orbit is described by **inclination `i`** and **right ascension of the
ascending node `Ω`**, or equally by the unit orbit normal `ĥ = (r × v)/|r × v|`. The angle
between two normals is the plane-change angle Δθ, usually the largest single cost in reaching a
target. The plane changes slowly over time (because of RAAN drift), so it is computed
**at query time** from the predicted state rather than stored.

## 5. Data model

### 5.1 Entities

```
OrbitalObject 1───* ElementSet
OrbitalObject 1───0..1 Robot            (object_type = ROBOT)
OrbitalObject 1───0..1 ParkingOrbit     (object_type = PARKING)
ParkingOrbit  1───* ParkingSlot
Robot         1───* StatusReport
StatusReport  *───0..1 OrbitalObject    (destination object; object_type PASSIVE)
StatusReport  *───0..1 ParkingSlot      (destination slot)
Robot         *───* Capability
```

### 5.2 Tables

**`orbital_objects`**: everything that has an orbit we care about.

| column | type | notes |
|---|---|---|
| id | int PK | internal ID |
| catalog_number | int null, unique | tracking-network catalog number (up to 9 digits); null for `PARKING` |
| name | text unique | |
| object_type | enum `ROBOT`, `PASSIVE`, `PARKING` | `PASSIVE` = unpowered tracked object; `PARKING` = defined parking orbit |
| created_at | timestamptz | |

Check: `catalog_number` is required for `ROBOT` and `PASSIVE`, and must be null for `PARKING`.

**`element_sets`**: orbit history, rows only ever added.

| column | type | notes |
|---|---|---|
| id | int PK | |
| object_id | FK orbital_objects | |
| epoch | timestamptz | |
| inclination, raan, eccentricity, arg_perigee, mean_anomaly | float | rad |
| mean_motion | float | rev/day |
| bstar, mean_motion_dot, mean_motion_ddot | float | 0 for parking orbits |
| source | text | `celestrak`, `space-track`, `defined`, `test`, … |
| raw | text null | original TLE lines / OMM record exactly as received, for audit |
| received_at | timestamptz | |

`UNIQUE(object_id, epoch)`; index `(object_id, epoch DESC)`.

**`robots`**

| column | type | notes |
|---|---|---|
| id | int PK | |
| object_id | FK orbital_objects, unique | object_type must be `ROBOT` |
| name | text unique | |

**`parking_orbits`**: extra details for `PARKING` objects.

| column | type | notes |
|---|---|---|
| object_id | FK orbital_objects, PK | object_type must be `PARKING` |
| purpose | enum `CONVENTIONAL`, `STAGING` | |
| slot_count | int | ≥ 1; fixed at creation |
| retired_at | timestamptz null | no arrivals after this time |

**`parking_slots`**: one row per slot, created with the parking orbit.

| column | type | notes |
|---|---|---|
| id | int PK | |
| parking_object_id | FK parking_orbits | |
| slot_index | int | 0 … slot_count−1 |
| phase_offset | float | rad, `slot_index × 2π / slot_count` |

`UNIQUE(parking_object_id, slot_index)`. Slots are rows (rather than just a number) so
status reports can reference a specific slot by foreign key.

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
| destination_object_id | FK orbital_objects null | must be `PASSIVE` |
| destination_slot_id | FK parking_slots null | a specific slot in a parking orbit |
| eta | timestamptz null | expected arrival at destination |
| source | text | |
| received_at | timestamptz | |

`UNIQUE(robot_id, reported_at)`; index `(robot_id, reported_at DESC)`.
Checks: at most one of `destination_object_id` / `destination_slot_id`; `BUSY` ⇒
`expected_free_at` set; destination set ⇒ `eta` set; `READY` ⇒ no destination.

A move to a parking slot is reported like any other: `BUSY`, destination = the slot,
`eta` = arrival, `expected_free_at` = arrival (or later, if the robot needs set-up time).
Once parked, the robot normally reports `READY` with no destination; it stays in the slot
until a later report gives it a new destination.

The rules "destination object must be passive" and "slot must be free" are enforced in the
ingestion layer (§6.3), because they depend on other rows.

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
    elementset.py      ElementSet value type; ↔ Satrec (TLE, OMM, defined elements)
    tle.py             parse/validate TLE text → ElementSet
    parking.py         parking-orbit inputs (altitude, i, Ω, …) → ElementSet; slot k → ElementSet;
                       spacing check
    elements.py        orbit normal, plane angle, altitude, rv → (i, Ω)
    propagate.py       SGP4 wrapper: (Satrec, t) → OrbitState
    cost.py            first-pass delta-v estimate                           [phase 2]
  db/
    models.py          SQLAlchemy models (§5.2)
    session.py         engine/session factory from config
    repository.py      latest_elements(object, at), latest_status(robot, at),
                       last_arrival(robot, at), slot_occupancy(parking, at), add_* helpers
  ingest/
    tracking.py        TLE file / OMM records → element_sets
    status.py          status report (JSON) → status_reports (incl. slot-free check)
    parking.py         parking orbit definition → orbital_objects + parking_orbits + slots
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
    elements_epoch: datetime
    elements_age: timedelta
    sgp4_error: int                   # 0 = ok

@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: int
    at: datetime
    status: Status
    status_predicted: bool            # True if derived (e.g. BUSY→READY), not reported
    status_age: timedelta
    orbit: OrbitState                 # where we think the robot is
    orbit_basis: Literal["OWN", "ARRIVED_AT"]
    located_at: Location | None       # passive object or parking slot it is assumed to be at
    destination: OrbitState | None    # where its current destination is at `at`
    warnings: frozenset[Warning]      # STALE_ELEMENTS, STALE_STATUS, SGP4_ERROR, ...
```

Rules for `predict(robot, at)`:

1. Load the latest status report with `reported_at ≤ at`; none → status `UNKNOWN`.
2. Load the robot's own latest element set with `epoch ≤ at`; none → raise `NoOrbitData`.
3. **Status:** `BUSY` and `expected_free_at ≤ at` → `READY` (`status_predicted=True`).
4. **Position after arrival.** Find the robot's **last arrival**: the most recent status
   report (at or before `at`) that has a destination with `eta ≤ at`. This looks at the whole
   history, not just the latest report, because a parked robot normally reports `READY` with
   no destination afterwards. If there is a last arrival *and* the robot's own element set is
   older than that `eta`, the robot has maneuvered since its own elements were measured, so
   place it at the arrival destination's predicted position (`orbit_basis = ARRIVED_AT`,
   `located_at = destination`). For a parking slot that is the slot's own position
   (parking elements + the slot's phase offset), so two robots in one parking orbit are
   never placed at the same point.
   As soon as the tracking network publishes robot elements newer than the `eta`, rule 2
   takes over again.
5. **Before arrival** (in transit, `reported_at < at < eta`): the robot's own elements are
   used as-is. This is approximate while the robot is maneuvering; acceptable under A2.
6. **Staleness** (warnings, not errors):
   `STALE_ELEMENTS` if the age of the elements used > `max_elements_age` (default 3 days; not
   applied to parking slots, which have no measurement to go stale);
   `STALE_STATUS` if status age > `max_status_age` (default 24 h).
7. SGP4 error code (e.g. decayed orbit) → `SGP4_ERROR` warning, position NaN; never raises.

`predict_fleet(at)` is a simple loop over robots (A3).

### 6.2 First-pass cost (`orbit/cost.py`, phase 2)

Cheap screening, enough to rank the nearest robot (A2):

- Plane change: `Δv_plane = 2·v·sin(Δθ/2)`, where Δθ is the angle between the two
  orbit normals at the job time.
- Altitude change: Hohmann transfer `Δv` between the two mean radii (circular approximation).
- Estimate = `Δv_plane + Δv_hohmann` (an upper bound; ignores phasing and combined burns).

Candidates: robots that have the required capabilities, predicted `READY` at the job's
start time, and no `SGP4_ERROR`, sorted by estimate. Stale data is shown, not hidden.

The same estimate works between any two orbits, so it also supports parking:
`cost(robot → parking orbit)` now, versus `cost(parking orbit → expected target)` later.
Since it ignores phase, it is the same for every slot in a parking orbit and is calculated
once per parking orbit (useful as idle robots cluster in conventional orbits). Choosing
parking orbits and free slots automatically is phase 2.

### 6.3 Ingestion

| Situation | Behaviour |
|---|---|
| Bad TLE (checksum, format, epoch parse) or OMM missing required fields | Rejected with reason |
| Elements for an unknown catalog number | Rejected (objects are registered explicitly) |
| Duplicate `(object_id, epoch)` / `(robot_id, reported_at)` | Same content → no-op; different content → rejected |
| Status/element epoch more than 60 s in the future | Rejected (parking orbit definitions may use a future epoch) |
| Destination object is not `PASSIVE` (a robot or parking orbit object), or unknown | Rejected (parking is addressed by slot) |
| Destination slot is `OCCUPIED` or `RESERVED` by another robot at the move's `eta`, or its parking orbit is retired | Rejected, with the robot holding it |
| Parking orbit below 160 km altitude, eccentricity ≥ 1, or slot spacing < `min_slot_spacing_km` | Rejected |
| Non-UTC / naive datetimes | Rejected at the edges |

## 7. Testing strategy

- **Unit**: TLE parsing (checksum, epoch), OMM parsing, parking orbit and slot → elements,
  slot spacing, plane angle for known cases, time conversions.
- **SGP4 wrapper**: matches the `sgp4` library's own output for known element sets.
- **Repository**: in-memory SQLite per test; latest-row selection for both streams, last
  arrival lookup, slot occupancy, uniqueness.
- **Prediction scenarios**: an injected `Clock`, so tests choose "now"; cover each rule in §6.1.
- **Test data**: fixed element sets checked into `tests/data/`, so tests never need the network.
- CI: `ruff`, `mypy --strict`, `pytest` with coverage.

## 8. Open questions

1. Which tracking-network feed, and how is access arranged (Space-Track account, CelesTrak
   public data, or an internal feed)? Affects only `ingest/tracking.py`.
2. Where do status reports come from (operator UI, fleet control API, file drop)?
3. ~~Should destinations allow a parking orbit?~~ **Yes** (§4.3): used for staging robots near
   expected future work.
4. ~~Do robots sharing a parking orbit need separate slots?~~ **Yes** (§4.3): one robot per
   slot, slots evenly spaced, occupancy derived from status history.
5. Who decides where conventional parking orbits go, and what makes a good one (e.g. near
   common work areas, low-drag altitude, away from busy orbital shells)? Phase 2 could
   suggest candidates from past job locations.
