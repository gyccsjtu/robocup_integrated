"""Independent acceptance checks over the controller's recorded actual telemetry."""
import json
import math
from pathlib import Path
import sys

path = Path(sys.argv[1])
rows = [json.loads(line) for line in path.read_text().splitlines()]
config = rows[0]["config"]
arrivals = [r for r in rows if r["event"] == "ARRIVED"]
assert [r["name"] for r in arrivals] == ["TAKEOFF", "A", "B"], "wrong arrival sequence"
assert all(r["distance_m"] <= config["arrival_tolerance_m"] and r["speed_mps"] <= config["arrival_speed_mps"] for r in arrivals)
assert len([r for r in rows if r["event"] == "HOVER_DONE"]) == 3
assert rows[-1]["event"] == "RESULT" and rows[-1]["code"] == 0
assert rows[-1]["armed"] is False and rows[-1]["landed_state"] == 1
assert not any(r["event"] in ("FAILURE", "LANDING_UNCONFIRMED") for r in rows)
origin = next(r["command"] for r in rows if r["command"] is not None)
positions = [r["position"] for r in rows if r["position"] is not None]
max_height = max(p[2] - origin[2] for p in positions)
assert max_height <= config["max_altitude_m"], "altitude limit violated"
assert max(math.hypot(p[0] - origin[0], p[1] - origin[1]) for p in positions) <= config["max_radius_m"]
speed = [r["velocity"] for r in rows if r["velocity"] is not None]
print(json.dumps(dict(passed=True, log=str(path), wall_s=rows[-1]["elapsed_s"],
    sim_s=rows[-1]["ros_s"] - rows[0]["ros_s"], max_height_m=max_height,
    max_horizontal_speed_mps=max(math.hypot(*v[:2]) for v in speed),
    max_vertical_speed_mps=max(abs(v[2]) for v in speed),
    arrivals=[dict(name=r["name"], distance_m=r["distance_m"], speed_mps=r["speed_mps"]) for r in arrivals]), indent=2))
