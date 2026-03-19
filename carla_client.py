"""
RoadSense — CARLA Simulation Client
Connects to a running CARLA server, drives the ego vehicle,
runs the pothole detection model on camera frames, and POSTs
detections to the RoadSense backend API.

Requirements:
  pip install carla requests numpy opencv-python-headless

Usage:
  python carla_client.py --host 127.0.0.1 --port 2000 \
      --api-url http://localhost:5000 \
      --api-key dev-device-key-carla-sim \
      --model-path ../models/pothole_model.tflite
"""

import argparse
import time
import math
import queue
import threading
import requests
import numpy as np

# Optional CARLA import — graceful mock if not installed
try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    print("[WARN] CARLA package not found. Running in mock/simulation mode.")

# Optional TFLite import
try:
    import tflite_runtime.interpreter as tflite
    TFLITE_AVAILABLE = True
except ImportError:
    try:
        import tensorflow as tf
        tflite = tf.lite
        TFLITE_AVAILABLE = True
    except ImportError:
        TFLITE_AVAILABLE = False
        print("[WARN] TFLite not found. Using mock detections.")

import cv2


class PotholeDetector:
    """
    Wraps the TFLite pothole detection model.
    Falls back to random mock detections if model unavailable.
    """
    def __init__(self, model_path: str, conf_threshold: float = 0.45):
        self.threshold = conf_threshold
        self.interpreter = None
        if TFLITE_AVAILABLE and model_path:
            try:
                interpreter = tflite.Interpreter(model_path=model_path)
                interpreter.allocate_tensors()
                self.interpreter = interpreter
                self.input_details  = interpreter.get_input_details()
                self.output_details = interpreter.get_output_details()
                self.input_h = self.input_details[0]["shape"][1]
                self.input_w = self.input_details[0]["shape"][2]
                print(f"[Detector] Loaded model: {model_path}")
            except Exception as e:
                print(f"[Detector] Failed to load model: {e}. Using mock.")

    def detect(self, frame: np.ndarray) -> list[dict]:
        """
        Returns list of detections: [{confidence, severity, bbox_w, bbox_h}]
        """
        if self.interpreter is None:
            return self._mock_detect(frame)

        h, w = frame.shape[:2]
        resized = cv2.resize(frame, (self.input_w, self.input_h))
        input_data = np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)

        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()

        # Typical YOLO output: boxes, scores, classes, num_detections
        # Adjust indices to match your specific model's output layout
        boxes   = self.interpreter.get_tensor(self.output_details[0]["index"])[0]
        scores  = self.interpreter.get_tensor(self.output_details[1]["index"])[0]
        # classes = self.interpreter.get_tensor(self.output_details[2]["index"])[0]

        detections = []
        for i, score in enumerate(scores):
            if score < self.threshold:
                continue
            box = boxes[i]  # [ymin, xmin, ymax, xmax] normalized
            ymin, xmin, ymax, xmax = box
            bbox_w = int((xmax - xmin) * w)
            bbox_h = int((ymax - ymin) * h)
            severity = self._classify_severity(bbox_w * bbox_h, w * h)
            detections.append({
                "confidence": float(round(score, 4)),
                "severity": severity,
                "bbox_w": bbox_w,
                "bbox_h": bbox_h,
            })
        return detections

    def _classify_severity(self, bbox_area: int, frame_area: int) -> str:
        ratio = bbox_area / frame_area
        if ratio > 0.05:
            return "HIGH"
        elif ratio > 0.02:
            return "MEDIUM"
        return "LOW"

    def _mock_detect(self, frame: np.ndarray) -> list[dict]:
        """Random mock — fires ~20% of the time."""
        import random
        if random.random() > 0.2:
            return []
        conf = round(random.uniform(0.45, 0.97), 3)
        r = random.random()
        sev = "HIGH" if r < 0.25 else ("MEDIUM" if r < 0.6 else "LOW")
        return [{"confidence": conf, "severity": sev,
                 "bbox_w": random.randint(40, 140), "bbox_h": random.randint(30, 100)}]


class RoadSenseClient:
    """HTTP client that posts detections to the RoadSense API."""
    def __init__(self, api_url: str, api_key: str):
        self.base = api_url.rstrip("/")
        self.session = requests.Session()
        self.session.headers.update({
            "X-API-Key": api_key,
            "User-Agent": "RoadSense-CARLA/1.0",
        })

    def post_detection(self, payload: dict) -> bool:
        try:
            r = self.session.post(f"{self.base}/api/detections",
                                  json=payload, timeout=5)
            if r.status_code == 201:
                print(f"[API] ✓ {payload['id']} {payload['severity']} {payload['confidence']}")
                return True
            print(f"[API] ✗ {r.status_code} — {r.text[:120]}")
            return False
        except requests.RequestException as e:
            print(f"[API] Connection error: {e}")
            return False

    def health(self) -> bool:
        try:
            r = self.session.get(f"{self.base}/health", timeout=3)
            return r.status_code == 200
        except Exception:
            return False


# ── CARLA vehicle controller ───────────────────────────────────────────────

class CarlaRunner:
    """
    Spawns an ego vehicle in CARLA with autopilot, attaches a
    front-facing RGB camera, and pipes frames to the detector.
    """
    def __init__(self, host: str, port: int, detector: PotholeDetector,
                 client: RoadSenseClient, frame_skip: int = 5):
        self.host = host
        self.port = port
        self.detector = detector
        self.api_client = client
        self.frame_skip = frame_skip
        self._frame_q: queue.Queue = queue.Queue(maxsize=4)
        self._running = False
        self._frame_counter = 0

    def run(self):
        if not CARLA_AVAILABLE:
            print("[CARLA] Package not available — running mock loop instead.")
            self._mock_loop()
            return

        carla_client = carla.Client(self.host, self.port)
        carla_client.set_timeout(10.0)
        world = carla_client.get_world()
        bp_lib = world.get_blueprint_library()

        # Spawn vehicle
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        spawn_tf = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_tf)
        vehicle.set_autopilot(True)
        print(f"[CARLA] Spawned {vehicle.type_id} at {spawn_tf.location}")

        # Attach RGB camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "640")
        cam_bp.set_attribute("image_size_y", "480")
        cam_bp.set_attribute("fov", "90")
        cam_tf = carla.Transform(carla.Location(x=2.0, z=1.4))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        camera.listen(self._on_frame)

        self._running = True
        # Start detection worker thread
        worker = threading.Thread(target=self._detection_worker,
                                  args=(vehicle,), daemon=True)
        worker.start()

        try:
            print("[CARLA] Running. Ctrl-C to stop.")
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            camera.stop()
            camera.destroy()
            vehicle.destroy()
            print("[CARLA] Cleaned up actors.")

    def _on_frame(self, image):
        """CARLA camera callback — convert and queue frame."""
        self._frame_counter += 1
        if self._frame_counter % self.frame_skip != 0:
            return
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3]
        frame_copy = arr.copy()
        try:
            self._frame_q.put_nowait((image.frame, frame_copy))
        except queue.Full:
            pass

    def _detection_worker(self, vehicle):
        """Worker thread: pulls frames, runs detector, posts results."""
        while self._running:
            try:
                frame_id, frame = self._frame_q.get(timeout=1)
            except queue.Empty:
                continue

            detections = self.detector.detect(frame)
            if not detections:
                continue

            tf = vehicle.get_transform()
            loc = tf.location

            # Convert CARLA world coords to approximate lat/lon
            # This uses a simple linear projection anchored to Chennai
            # Replace with actual GNSS sensor data in production
            lat, lon = _carla_to_latlon(loc.x, loc.y)

            for det in detections:
                import uuid as _uuid
                payload = {
                    "id": f"CARLA-{_uuid.uuid4().hex[:8].upper()}",
                    "timestamp": _now_str(),
                    "latitude": lat,
                    "longitude": lon,
                    "confidence": det["confidence"],
                    "severity": det["severity"],
                    "source": "carla",
                    "vehicle_id": str(vehicle.id),
                    "frame_id": frame_id,
                    "bbox_w": det["bbox_w"],
                    "bbox_h": det["bbox_h"],
                }
                self.api_client.post_detection(payload)

    def _mock_loop(self):
        """Runs when CARLA is not available — sends mock detections."""
        import random, uuid as _uuid
        import math
        print("[Mock] Sending simulated CARLA detections every 0.8s")
        cx, cy = 0.0, 0.0
        angle = 0.0
        self._running = True
        try:
            while self._running:
                time.sleep(0.8)
                angle += 0.05
                cx += math.cos(angle) * 2
                cy += math.sin(angle) * 2
                lat, lon = _carla_to_latlon(cx, cy)

                if random.random() > 0.3:
                    continue

                r = random.random()
                sev = "HIGH" if r < 0.25 else ("MEDIUM" if r < 0.6 else "LOW")
                payload = {
                    "id": f"CARLA-{_uuid.uuid4().hex[:8].upper()}",
                    "timestamp": _now_str(),
                    "latitude": lat,
                    "longitude": lon,
                    "confidence": round(random.uniform(0.45, 0.97), 3),
                    "severity": sev,
                    "source": "carla",
                    "vehicle_id": "CARLA-EGO-01",
                    "frame_id": int(time.time() * 10) % 100000,
                    "bbox_w": random.randint(40, 140),
                    "bbox_h": random.randint(30, 100),
                }
                self.api_client.post_detection(payload)
        except KeyboardInterrupt:
            self._running = False


def _carla_to_latlon(x: float, y: float,
                     origin_lat: float = 13.0827,
                     origin_lon: float = 80.2707) -> tuple[float, float]:
    """Convert CARLA world XY (meters) to approximate lat/lon."""
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    lat = round(origin_lat + y / meters_per_deg_lat, 6)
    lon = round(origin_lon + x / meters_per_deg_lon, 6)
    return lat, lon


def _now_str() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


# ── Entry point ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RoadSense CARLA Client")
    parser.add_argument("--host",       default="127.0.0.1")
    parser.add_argument("--port",       type=int, default=2000)
    parser.add_argument("--api-url",    default="http://localhost:5000")
    parser.add_argument("--api-key",    default="dev-device-key-carla-sim")
    parser.add_argument("--model-path", default="")
    parser.add_argument("--frame-skip", type=int, default=5,
                        help="Process every Nth CARLA frame")
    args = parser.parse_args()

    api_client = RoadSenseClient(args.api_url, args.api_key)
    if not api_client.health():
        print(f"[WARN] Backend at {args.api_url} not reachable. Will retry on each post.")

    detector = PotholeDetector(args.model_path)
    runner = CarlaRunner(args.host, args.port, detector, api_client, args.frame_skip)
    runner.run()
