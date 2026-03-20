import os
import json
import time
import queue
import uuid
import threading
from datetime import datetime, timezone
from functools import wraps
from flask import Flask, request, jsonify, Response, send_from_directory, abort
from flask_cors import CORS
from db import init_db, save_detection, get_detections, get_stats
from config import Config

app = Flask(__name__, static_folder="static")
CORS(app)

class SSEBroker:
    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()

    def subscribe(self):
        q = queue.Queue(maxsize=50)
        with self.lock:
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            try:
                self.clients.remove(q)
            except ValueError:
                pass

    def publish(self, event, data):
        msg = f"event: {event}\ndata: {json.dumps(data)}\n\n"
        with self.lock:
            dead = []
            for q in self.clients:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                try:
                    self.clients.remove(q)
                except ValueError:
                    pass

broker = SSEBroker()

# ── In-memory stores ───────────────────────────────────────────────────────
_vehicle_positions = {}
_vehicle_lock      = threading.Lock()

_road_network      = {"segments": [], "sidewalks": [], "map": ""}
_road_lock         = threading.Lock()

# ── Pothole consensus vote store ───────────────────────────────────────────
# Key: snapped grid cell string "x,y"  (5-metre grid)
# Value: { "pothole": set(vehicle_ids), "clear": set(vehicle_ids),
#          "confirmed": bool, "resolved": bool,
#          "carla_x": float, "carla_y": float }
_vote_store      = {}
_vote_lock       = threading.Lock()
POTHOLE_THRESHOLD = 10   # votes to confirm a pothole
CLEAR_THRESHOLD   = 15   # votes to resolve / remove it
SNAP_GRID         = 5.0  # metres — coordinate snapping granularity

def _snap(val):
    """Snap a coordinate to the nearest grid cell centre."""
    return round(round(val / SNAP_GRID) * SNAP_GRID, 1)

def _cell_key(x, y):
    return f"{_snap(x)},{_snap(y)}"

class RateLimiter:
    def __init__(self):
        self.windows = {}
        self.lock = threading.Lock()

    def is_allowed(self, key, limit, window_secs):
        now = time.time()
        with self.lock:
            times = [t for t in self.windows.get(key, []) if now - t < window_secs]
            if len(times) >= limit:
                return False
            times.append(now)
            self.windows[key] = times
            return True

limiter = RateLimiter()

def require_api_key(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        key = request.headers.get("X-API-Key", "")
        if not key or key not in Config.API_KEYS:
            return jsonify({"error": "Unauthorized"}), 401
        request.api_role = Config.API_KEYS[key]
        request.api_key  = key
        return f(*args, **kwargs)
    return wrapper

def require_role(*roles):
    def decorator(f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            if getattr(request, "api_role", None) not in roles:
                return jsonify({"error": "Forbidden"}), 403
            return f(*args, **kwargs)
        return wrapper
    return decorator

def rate_limit(limit=60, window=60):
    def decorator(f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            key = getattr(request, "api_key", request.remote_addr)
            if not limiter.is_allowed(key, limit, window):
                return jsonify({"error": "Rate limit exceeded"}), 429
            return f(*args, **kwargs)
        return wrapper
    return decorator

# ── Static pages ───────────────────────────────────────────────────────────

@app.route("/")
def root():
    return send_from_directory("static", "index.html")

@app.route("/pi.html")
def pi_dashboard():
    return send_from_directory("static", "pi.html")

@app.route("/health")
def health():
    return jsonify({
        "status": "ok",
        "version": Config.VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "sse_clients": len(broker.clients),
    })

# ── Vehicle position ───────────────────────────────────────────────────────

@app.route("/api/vehicle", methods=["POST"])
@require_api_key
@require_role("device", "admin")
def update_vehicle():
    data = request.get_json(force=True, silent=True) or {}
    vid  = data.get("vehicle_id", "unknown")
    with _vehicle_lock:
        _vehicle_positions[vid] = data
    broker.publish("vehicle", data)
    return jsonify({"status": "ok"}), 200

@app.route("/api/vehicle", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def get_vehicle():
    with _vehicle_lock:
        if not _vehicle_positions:
            return jsonify({}), 200
        latest = max(_vehicle_positions.values(), key=lambda v: v.get("ts", 0))
        return jsonify(latest)

@app.route("/api/vehicles", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def get_all_vehicles():
    with _vehicle_lock:
        return jsonify(list(_vehicle_positions.values()))

@app.route("/api/vehicles/batch", methods=["POST"])
@require_api_key
@require_role("device", "admin")
def batch_update_vehicles():
    """
    Receive all 30 car positions in a single request.
    Stores each one and fires a single SSE event with the full fleet snapshot
    instead of 30 individual SSE events — cuts dashboard render load by 30x.
    """
    data     = request.get_json(force=True, silent=True) or {}
    vehicles = data.get("vehicles", [])
    if not vehicles:
        return jsonify({"error": "no vehicles"}), 400

    with _vehicle_lock:
        for v in vehicles:
            vid = v.get("vehicle_id", "unknown")
            _vehicle_positions[vid] = v

    # Single SSE event with full fleet snapshot
    broker.publish("fleet_snapshot", vehicles)
    return jsonify({"status": "ok", "count": len(vehicles)}), 200

# ── Road network ───────────────────────────────────────────────────────────

@app.route("/api/road_network", methods=["POST"])
@require_api_key
@require_role("device", "admin")
def set_road_network():
    data = request.get_json(force=True, silent=True) or {}
    with _road_lock:
        _road_network["segments"]  = data.get("segments",  [])
        _road_network["sidewalks"] = data.get("sidewalks", [])
        _road_network["map"]       = data.get("map", "")
    broker.publish("road_network", {
        "map":   _road_network["map"],
        "count": len(_road_network["segments"]),
    })
    print(f"[Road] {len(_road_network['segments'])} segments, map={_road_network['map']}")
    return jsonify({"status": "ok", "segments": len(_road_network["segments"])}), 200

@app.route("/api/road_network", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def get_road_network():
    """Return stored CARLA road waypoints to the dashboard."""
    with _road_lock:
        return jsonify(_road_network)

# ── Detections ─────────────────────────────────────────────────────────────

@app.route("/api/detections", methods=["POST"])
@require_api_key
@require_role("device", "admin")
@rate_limit(limit=Config.DEVICE_RATE_LIMIT, window=60)
def ingest_detection():
    if request.content_type and "multipart" in request.content_type:
        data       = request.form.to_dict()
        image_file = request.files.get("image")
    else:
        data       = request.get_json(force=True, silent=True) or {}
        image_file = None

    required = ("latitude", "longitude", "confidence", "severity")
    missing  = [f for f in required if f not in data]
    if missing:
        return jsonify({"error": f"Missing fields: {missing}"}), 400

    try:
        lat  = float(data["latitude"])
        lon  = float(data["longitude"])
        conf = float(data["confidence"])
        sev  = str(data["severity"]).upper()
        assert -90  <= lat  <= 90
        assert -180 <= lon  <= 180
        assert 0    <= conf <= 1
        assert sev in ("LOW", "MEDIUM", "HIGH")
    except (ValueError, AssertionError) as e:
        return jsonify({"error": str(e)}), 400

    image_path = None
    if image_file and image_file.filename:
        ext = image_file.filename.rsplit(".", 1)[-1].lower()
        if ext not in ("jpg", "jpeg", "png"):
            return jsonify({"error": "Image must be JPEG or PNG"}), 400
        fname = f"{uuid.uuid4().hex}.{ext}"
        os.makedirs(Config.IMAGE_DIR, exist_ok=True)
        image_file.save(os.path.join(Config.IMAGE_DIR, fname))
        image_path = f"/static/detections/{fname}"

    det_id = data.get("id") or f"DET-{uuid.uuid4().hex[:8].upper()}"
    ts     = data.get("timestamp") or datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    record = {
        "id":          det_id,
        "timestamp":   ts,
        "latitude":    lat,
        "longitude":   lon,
        "confidence":  round(conf, 4),
        "severity":    sev,
        "source":      data.get("source", "live"),
        "vehicle_id":  data.get("vehicle_id", ""),
        "frame_id":    int(data.get("frame_id", 0)),
        "bbox_w":      int(data.get("bbox_w", 0)),
        "bbox_h":      int(data.get("bbox_h", 0)),
        "image_path":  image_path or data.get("image_path", ""),
        "ingested_at": datetime.now(timezone.utc).isoformat(),
    }
    # Store raw CARLA coords as extra fields (not in DB schema, passed through SSE only)
    sse_record = dict(record)
    sse_record["carla_x"] = data.get("carla_x", None)
    sse_record["carla_y"] = data.get("carla_y", None)

    save_detection(record)
    broker.publish("detection", sse_record)
    return jsonify({"status": "ok", "id": det_id}), 201

@app.route("/api/detections", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
@rate_limit(limit=120, window=60)
def query_detections():
    limit    = min(int(request.args.get("limit", 100)), 500)
    offset   = int(request.args.get("offset", 0))
    severity = request.args.get("severity", "").upper().split(",")
    source   = request.args.get("source", None)
    since    = request.args.get("since", None)
    until    = request.args.get("until", None)
    rows, total = get_detections(
        limit=limit, offset=offset,
        severity=severity if any(severity) else None,
        source=source, since=since, until=until,
    )
    return jsonify({"detections": rows, "total": total, "limit": limit, "offset": offset})

@app.route("/api/detections/<det_id>", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def get_detection(det_id):
    rows, _ = get_detections(limit=1, det_id=det_id)
    if not rows:
        return jsonify({"error": "Not found"}), 404
    return jsonify(rows[0])

@app.route("/api/stats", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def stats():
    return jsonify(get_stats())

# ── Pothole consensus voting ───────────────────────────────────────────────

@app.route("/api/pothole_vote", methods=["POST"])
@require_api_key
@require_role("device", "admin")
def pothole_vote():
    data = request.get_json(force=True, silent=True) or {}
    vid  = data.get("vehicle_id", "unknown")
    vote = data.get("vote", "")
    try:
        cx = float(data["carla_x"])
        cy = float(data["carla_y"])
    except (KeyError, ValueError):
        return jsonify({"error": "carla_x and carla_y required"}), 400
    if vote not in ("pothole", "clear"):
        return jsonify({"error": "vote must be 'pothole' or 'clear'"}), 400

    key = _cell_key(cx, cy)
    sx, sy = _snap(cx), _snap(cy)

    with _vote_lock:
        if key not in _vote_store:
            _vote_store[key] = {
                "pothole":   set(),
                "clear":     set(),
                "confirmed": False,
                "resolved":  False,
                "carla_x":   sx,
                "carla_y":   sy,
            }
        cell = _vote_store[key]
        cell[vote].add(vid)
        pothole_count = len(cell["pothole"])
        clear_count   = len(cell["clear"])

    if vote == "pothole" and pothole_count >= POTHOLE_THRESHOLD and not _vote_store[key]["confirmed"]:
        with _vote_lock:
            _vote_store[key]["confirmed"] = True
            _vote_store[key]["resolved"]  = False
        broker.publish("pothole_confirmed", {
            "carla_x":  sx, "carla_y": sy,
            "votes":    pothole_count, "cell_key": key,
        })
        print(f"[Vote] POTHOLE CONFIRMED at ({sx},{sy}) -- {pothole_count} votes")

    elif (vote == "clear" and clear_count >= CLEAR_THRESHOLD
          and _vote_store[key]["confirmed"] and not _vote_store[key]["resolved"]):
        with _vote_lock:
            _vote_store[key]["resolved"]  = True
            _vote_store[key]["confirmed"] = False
            _vote_store[key]["pothole"]   = set()
            _vote_store[key]["clear"]     = set()
        broker.publish("pothole_resolved", {
            "carla_x": sx, "carla_y": sy, "cell_key": key,
        })
        print(f"[Vote] POTHOLE RESOLVED at ({sx},{sy}) -- {clear_count} clear votes")

    broker.publish("vote_update", {
        "carla_x":       sx,
        "carla_y":       sy,
        "cell_key":      key,
        "pothole_votes": pothole_count,
        "clear_votes":   clear_count,
        "confirmed":     _vote_store[key]["confirmed"],
        "resolved":      _vote_store[key]["resolved"],
        "vehicle_id":    vid,
        "vote":          vote,
    })
    return jsonify({"status": "ok", "cell": key,
                    "pothole_votes": pothole_count, "clear_votes": clear_count}), 200


@app.route("/api/pothole_zones", methods=["GET"])
@require_api_key
@require_role("dashboard", "admin", "device")
def get_pothole_zones():
    with _vote_lock:
        result = [
            {"cell_key": k, "carla_x": c["carla_x"], "carla_y": c["carla_y"],
             "pothole_votes": len(c["pothole"]), "clear_votes": len(c["clear"]),
             "confirmed": c["confirmed"], "resolved": c["resolved"]}
            for k, c in _vote_store.items()
        ]
    return jsonify(result)


# ── SSE stream ─────────────────────────────────────────────────────────────

@app.route("/api/stream")
def sse_stream():
    key = request.args.get("key", request.headers.get("X-API-Key", ""))
    if not key or key not in Config.API_KEYS:
        abort(401)
    if Config.API_KEYS[key] not in ("dashboard", "admin"):
        abort(403)

    def stream():
        q = broker.subscribe()
        try:
            yield f"event: connected\ndata: {json.dumps({'status': 'ok'})}\n\n"
            # Send road network immediately if already loaded
            with _road_lock:
                if _road_network["segments"]:
                    yield (f"event: road_network\ndata: "
                           f"{json.dumps({'map': _road_network['map'], 'segments': _road_network['segments'], 'sidewalks': _road_network['sidewalks']})}\n\n")
            # Send current vote state so dashboard recovers after refresh
            with _vote_lock:
                zones_snapshot = [
                    {"cell_key": k, "carla_x": c["carla_x"], "carla_y": c["carla_y"],
                     "pothole_votes": len(c["pothole"]), "clear_votes": len(c["clear"]),
                     "confirmed": c["confirmed"], "resolved": c["resolved"]}
                    for k, c in _vote_store.items()
                ]
            if zones_snapshot:
                yield (f"event: zones_snapshot\ndata: {json.dumps(zones_snapshot)}\n\n")
            while True:
                try:
                    msg = q.get(timeout=20)
                    yield msg
                except queue.Empty:
                    yield f"event: heartbeat\ndata: {json.dumps({'ts': time.time()})}\n\n"
        except GeneratorExit:
            pass
        finally:
            broker.unsubscribe(q)

    return Response(stream(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

@app.route("/static/detections/<filename>")
def serve_image(filename):
    return send_from_directory(Config.IMAGE_DIR, filename)

if __name__ == "__main__":
    os.makedirs(Config.IMAGE_DIR, exist_ok=True)
    init_db()
    print(f"[RoadSense] Starting on port {Config.PORT}")
    app.run(host="0.0.0.0", port=Config.PORT, threaded=True, debug=False)