"""
RoadSense — Multi-Car CARLA Simulation
=======================================
Spawns 30 cars on autopilot. Pre-seeds pothole zones and cleared zones
on the map. Cars vote as they pass zones. When 10 cars vote 'pothole'
for the same cell, a confirmed pothole marker appears on the dashboard.
When 15 cars vote 'clear', the marker is removed.

Cars briefly raise an i flag (sent via the vehicle position update)
when they cast a vote, so the dashboard can show the bubble above them.
"""

import carla
import math
import time
import random
import uuid
import threading
import requests

# ── CARLA connection ──────────────────────────────────────────────────────────
client    = carla.Client('localhost', 2000)
client.set_timeout(15.0)
world     = client.get_world()
carla_map = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bps = blueprint_library.filter('vehicle.*')
car_bps = [bp for bp in vehicle_bps if int(bp.get_attribute('number_of_wheels')) == 4]
print(f"[CARLA] Connected to {carla_map.name} | {len(car_bps)} car blueprints available")

# ── Road network export ───────────────────────────────────────────────────────
def export_road_network():
    print("[CARLA] Building road segments from topology...")
    topology = carla_map.get_topology()
    segments = []
    seen     = set()

    for start_wp, end_wp in topology:
        wp   = start_wp
        prev = None
        for _ in range(600):
            loc = wp.transform.location
            pt  = (round(loc.x, 1), round(loc.y, 1))
            if prev is not None:
                key = (prev, pt)
                if key not in seen:
                    seen.add(key)
                    segments.append([list(prev), list(pt)])
            prev  = pt
            nexts = wp.next(2.5)
            if not nexts:
                break
            wp = nexts[0]
            if wp.road_id == end_wp.road_id and wp.lane_id == end_wp.lane_id:
                loc2 = end_wp.transform.location
                pt2  = (round(loc2.x, 1), round(loc2.y, 1))
                if prev and (prev, pt2) not in seen:
                    seen.add((prev, pt2))
                    segments.append([list(prev), list(pt2)])
                break

    sidewalk_pts = []
    for wp in carla_map.generate_waypoints(4.0):
        if wp.lane_type == carla.LaneType.Sidewalk:
            loc = wp.transform.location
            sidewalk_pts.append([round(loc.x, 1), round(loc.y, 1)])

    print(f"[CARLA] {len(segments)} road segments, {len(sidewalk_pts)} sidewalk pts")
    return segments, sidewalk_pts

road_segments, sidewalk_pts = export_road_network()

# ── API session ───────────────────────────────────────────────────────────────
ROADSENSE_URL = "https://visionx-production-425c.up.railway.app"
ROADSENSE_KEY = "dev-device-key-carla-sim"
session = requests.Session()
session.headers.update({"X-API-Key": ROADSENSE_KEY})

def push_road_network():
    try:
        r = session.post(f"{ROADSENSE_URL}/api/road_network", json={
            "segments":  road_segments,
            "sidewalks": sidewalk_pts,
            "map":       carla_map.name,
        }, timeout=30)
        print(f"[CARLA] Road network -> backend: HTTP {r.status_code}")
    except Exception as e:
        print(f"[CARLA] Road push failed: {e}")

threading.Thread(target=push_road_network, daemon=True).start()

# ── GPS helper ────────────────────────────────────────────────────────────────
ANCHOR_LAT, ANCHOR_LON = 13.0827, 80.2707

def to_gps(x, y):
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(ANCHOR_LAT))
    return round(ANCHOR_LAT + y / mlat, 6), round(ANCHOR_LON + x / mlon, 6)

# ── Pre-seed pothole & cleared zones ─────────────────────────────────────────
def pick_zone_waypoints(n_pothole=10, n_clear=4):
    """
    Sample drivable waypoints spread across the map (20m grid buckets).
    Returns lists of (x, y) tuples for pothole zones and clear zones.
    """
    all_wps  = carla_map.generate_waypoints(8.0)
    drivable = [wp for wp in all_wps if wp.lane_type == carla.LaneType.Driving]
    random.shuffle(drivable)

    pothole_zones = []
    clear_zones   = []
    used_cells    = set()

    for wp in drivable:
        x, y = wp.transform.location.x, wp.transform.location.y
        cell = (int(x / 20), int(y / 20))
        if cell in used_cells:
            continue
        used_cells.add(cell)
        if len(pothole_zones) < n_pothole:
            pothole_zones.append((round(x, 1), round(y, 1)))
        elif len(clear_zones) < n_clear:
            clear_zones.append((round(x, 1), round(y, 1)))
        if len(pothole_zones) >= n_pothole and len(clear_zones) >= n_clear:
            break

    print(f"[Zones] {len(pothole_zones)} pothole zones, {len(clear_zones)} clear zones")
    for i, z in enumerate(pothole_zones):
        print(f"  Pothole zone {i+1:02d}: x={z[0]:8.1f}  y={z[1]:8.1f}")
    for i, z in enumerate(clear_zones):
        print(f"  Clear   zone {i+1:02d}: x={z[0]:8.1f}  y={z[1]:8.1f}")
    return pothole_zones, clear_zones

POTHOLE_ZONES, CLEAR_ZONES = pick_zone_waypoints(n_pothole=10, n_clear=4)
DETECTION_RADIUS = 12.0   # metres — car must be within this to trigger a vote

# ── Spawn 30 cars ─────────────────────────────────────────────────────────────
NUM_CARS  = 30
spawn_pts = carla_map.get_spawn_points()
random.shuffle(spawn_pts)

vehicles = []
for i in range(min(NUM_CARS, len(spawn_pts))):
    bp = random.choice(car_bps)
    if bp.has_attribute('color'):
        bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
    try:
        v = world.spawn_actor(bp, spawn_pts[i])
        v.set_autopilot(True)
        vehicles.append(v)
    except Exception as e:
        print(f"[CARLA] Spawn {i} failed: {e}")

print(f"[CARLA] Spawned {len(vehicles)} cars")

# ── Spectator camera ──────────────────────────────────────────────────────────
spectator = world.get_spectator()

def update_spectator(v):
    tf  = v.get_transform()
    fwd = tf.get_forward_vector()
    spectator.set_transform(carla.Transform(
        carla.Location(x=tf.location.x - fwd.x*8,
                       y=tf.location.y - fwd.y*8,
                       z=tf.location.z + 4),
        carla.Rotation(pitch=-15, yaw=tf.rotation.yaw, roll=0)
    ))

# ── API helpers ───────────────────────────────────────────────────────────────
def send_vehicle_position(vehicle_id, x, y, heading, flagged=False, flag_type=None):
    lat, lon = to_gps(x, y)
    try:
        session.post(f"{ROADSENSE_URL}/api/vehicle", json={
            "vehicle_id": vehicle_id,
            "source":     "carla",
            "latitude":   lat,
            "longitude":  lon,
            "carla_x":    round(x, 2),
            "carla_y":    round(y, 2),
            "heading":    heading,
            "ts":         time.time(),
            "flagged":    flagged,
            "flag_type":  flag_type or "",
        }, timeout=2)
    except Exception:
        pass

def send_pothole_vote(vehicle_id, x, y, vote):
    """vote = 'pothole' | 'clear'"""
    try:
        session.post(f"{ROADSENSE_URL}/api/pothole_vote", json={
            "vehicle_id": vehicle_id,
            "carla_x":    round(x, 2),
            "carla_y":    round(y, 2),
            "vote":       vote,
        }, timeout=3)
    except Exception:
        pass

# ── Per-car thread ────────────────────────────────────────────────────────────
def euclid(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def car_loop(vehicle, vehicle_id, car_index):
    """
    Runs continuously for one car.
    - Broadcasts position every ~0.4s including flag state
    - Detects proximity to pothole/clear zones and sends votes
    - Votes once per zone per pass; re-enables after car leaves the zone
    """
    voted_pothole = set()   # zone indices already voted this pass
    voted_clear   = set()
    flag_until    = 0.0
    flag_type_cur = None

    while True:
        try:
            loc     = vehicle.get_location()
            tf      = vehicle.get_transform()
            x, y    = loc.x, loc.y
            heading = tf.rotation.yaw
            now     = time.time()

            flagged   = now < flag_until
            flag_type = flag_type_cur if flagged else None

            # ── Check pothole zones ───────────────────────────────────────
            for idx, (zx, zy) in enumerate(POTHOLE_ZONES):
                d = euclid(x, y, zx, zy)
                if d < DETECTION_RADIUS:
                    if idx not in voted_pothole:
                        voted_pothole.add(idx)
                        # Small random delay to stagger simultaneous votes
                        time.sleep(random.uniform(0.0, 0.4))
                        threading.Thread(
                            target=send_pothole_vote,
                            args=(vehicle_id, zx, zy, "pothole"),
                            daemon=True
                        ).start()
                        flag_until    = time.time() + 2.5
                        flag_type_cur = "pothole"
                        flagged       = True
                        flag_type     = "pothole"
                        print(f"[Car {car_index:02d}] FLAG pothole zone {idx+1:02d}  x={zx} y={zy}")
                else:
                    # Allow re-vote once the car has clearly left the zone
                    if idx in voted_pothole and d > DETECTION_RADIUS * 1.8:
                        voted_pothole.discard(idx)

            # ── Check clear zones ─────────────────────────────────────────
            for idx, (zx, zy) in enumerate(CLEAR_ZONES):
                d = euclid(x, y, zx, zy)
                if d < DETECTION_RADIUS:
                    if idx not in voted_clear:
                        voted_clear.add(idx)
                        time.sleep(random.uniform(0.0, 0.4))
                        threading.Thread(
                            target=send_pothole_vote,
                            args=(vehicle_id, zx, zy, "clear"),
                            daemon=True
                        ).start()
                        flag_until    = time.time() + 2.5
                        flag_type_cur = "clear"
                        flagged       = True
                        flag_type     = "clear"
                        print(f"[Car {car_index:02d}] FLAG clear   zone {idx+1:02d}  x={zx} y={zy}")
                else:
                    if idx in voted_clear and d > DETECTION_RADIUS * 1.8:
                        voted_clear.discard(idx)

            # ── Broadcast position ────────────────────────────────────────
            threading.Thread(
                target=send_vehicle_position,
                args=(vehicle_id, x, y, heading, flagged, flag_type),
                daemon=True
            ).start()

        except Exception as e:
            print(f"[Car {car_index:02d}] Error: {e}")

        time.sleep(0.4)

# ── Launch all car threads ────────────────────────────────────────────────────
for i, v in enumerate(vehicles):
    vid = f"carla-car-{i+1:02d}"
    t   = threading.Thread(target=car_loop, args=(v, vid, i + 1), daemon=True)
    t.start()
    time.sleep(0.06)   # stagger startup

print(f"[RoadSense] {len(vehicles)} car threads running. Ctrl-C to stop.\n")

# ── Main thread — spectator ───────────────────────────────────────────────────
try:
    while True:
        if vehicles:
            update_spectator(vehicles[0])
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\n[RoadSense] Stopping...")
finally:
    print("[RoadSense] Destroying vehicles...")
    for v in vehicles:
        try:
            v.destroy()
        except Exception:
            pass
    print("[RoadSense] Done.")