"""
RoadSense — Smart Traffic Simulation
Exports CARLA road SEGMENTS (connected lines) so the dashboard
draws proper streets like a real map, not just dots.
"""

import carla
import math
import time
import random
import uuid
import threading
import requests

# -------- Connect to CARLA --------
client    = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world     = client.get_world()
carla_map = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp  = blueprint_library.filter('vehicle.*')[0]
spawn_point = carla_map.get_spawn_points()[0]
vehicle     = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)
print(f"[CARLA] Spawned {vehicle.type_id} on {carla_map.name}")

# -------- Export road SEGMENTS (pairs of connected points) -------------------
def export_road_network():
    """
    Uses CARLA's topology to walk each lane and produce connected
    line segments [[x1,y1],[x2,y2]]. Dashboard draws these as
    proper road lines — looks like a real street map.
    """
    print("[CARLA] Building road segments from topology...")
    topology  = carla_map.get_topology()
    segments  = []
    seen      = set()

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
                pt2  = (round(loc2.x,1), round(loc2.y,1))
                if prev and (prev, pt2) not in seen:
                    seen.add((prev, pt2))
                    segments.append([list(prev), list(pt2)])
                break

    # Also add sidewalk/boundary points as separate thin lines
    sidewalk_pts = []
    for wp in carla_map.generate_waypoints(4.0):
        if wp.lane_type == carla.LaneType.Sidewalk:
            loc = wp.transform.location
            sidewalk_pts.append([round(loc.x,1), round(loc.y,1)])

    print(f"[CARLA] {len(segments)} road segments, {len(sidewalk_pts)} sidewalk pts")
    return segments, sidewalk_pts

road_segments, sidewalk_pts = export_road_network()

# -------- API setup -----------------------------------------------------------
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
        print(f"[CARLA] Road network → backend: HTTP {r.status_code}")
    except Exception as e:
        print(f"[CARLA] Road push failed: {e}")

threading.Thread(target=push_road_network, daemon=True).start()

# -------- GPS conversion (for DB records only) --------------------------------
ANCHOR_LAT, ANCHOR_LON = 13.0827, 80.2707

def to_gps(x, y):
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(ANCHOR_LAT))
    return round(ANCHOR_LAT + y / mlat, 6), round(ANCHOR_LON + x / mlon, 6)

# -------- Detections ----------------------------------------------------------
def send_detection(x, y, defect_type="pothole"):
    lat, lon = to_gps(x, y)
    r    = random.random()
    sev  = "HIGH" if r < 0.25 else ("MEDIUM" if r < 0.6 else "LOW")
    conf = round(random.uniform(0.50, 0.96), 3)
    det_id = f"CRK-{uuid.uuid4().hex[:8].upper()}" if defect_type == "crack" \
             else f"PTH-{uuid.uuid4().hex[:8].upper()}"
    try:
        resp = session.post(f"{ROADSENSE_URL}/api/detections", json={
            "id": det_id, "latitude": lat, "longitude": lon,
            "confidence": conf, "severity": sev,
            "source": "crack" if defect_type == "crack" else "carla",
            "vehicle_id": str(vehicle.id),
            "frame_id": int(time.time() * 10) % 100000,
            "bbox_w": random.randint(40, 140), "bbox_h": random.randint(30, 100),
            "carla_x": round(x, 2), "carla_y": round(y, 2),
        }, timeout=5)
        icon = "⚡" if defect_type == "crack" else "⚠"
        if resp.status_code == 201:
            print(f"[✓] {icon} {defect_type.upper()} | {sev} | x={x:.1f} y={y:.1f}")
    except requests.ConnectionError:
        print("[!] Backend unreachable")

def send_vehicle_position(x, y, heading):
    lat, lon = to_gps(x, y)
    try:
        session.post(f"{ROADSENSE_URL}/api/vehicle", json={
            "vehicle_id": str(vehicle.id), "source": "carla",
            "latitude": lat, "longitude": lon,
            "carla_x": round(x, 2), "carla_y": round(y, 2),
            "heading": heading, "ts": time.time(),
        }, timeout=2)
    except Exception:
        pass

# -------- Spectator camera ----------------------------------------------------
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

# -------- Main loop -----------------------------------------------------------
print("[RoadSense] Running — Ctrl-C to stop\n")
pos_tick = 0
try:
    while True:
        loc     = vehicle.get_location()
        tf      = vehicle.get_transform()
        x, y    = loc.x, loc.y
        heading = tf.rotation.yaw

        update_spectator(vehicle)

        pos_tick += 1
        if pos_tick % 2 == 0:
            threading.Thread(target=send_vehicle_position,
                             args=(x, y, heading), daemon=True).start()

        roll = random.random()
        if   roll < 0.03: send_detection(x, y, "pothole")
        elif roll < 0.05: send_detection(x, y, "crack")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n[RoadSense] Stopping...")
finally:
    vehicle.destroy()
    print("[RoadSense] Done.")