"""
RoadSense — Raspberry Pi Edge Client
Runs on the Pi alongside the TFLite inference loop.
Call post_detection() after each confirmed detection.

Usage:
  from edge_client import EdgeClient
  client = EdgeClient()
  client.post_detection(lat, lon, conf, sev, frame, bbox_w, bbox_h, image_path)
"""

import os
import time
import json
import uuid
import queue
import threading
import requests
from datetime import datetime
from pathlib import Path


class EdgeClient:
    """
    Non-blocking HTTP client for the Pi.
    Detections are queued and sent by a background thread
    so inference never blocks waiting for the network.
    """
    def __init__(
        self,
        api_url: str = None,
        api_key: str = None,
        vehicle_id: str = None,
        retry_limit: int = 3,
        queue_size: int = 200,
        offline_log: str = "offline_queue.jsonl",
    ):
        self.base = (api_url or os.getenv("ROADSENSE_API_URL", "https://visionx-production-425c.up.railway.app")).rstrip("/")
        self.api_key = api_key or os.getenv("ROADSENSE_API_KEY", "")
        self.vehicle_id = vehicle_id or os.getenv("VEHICLE_ID", _get_pi_serial())
        self.retry_limit = retry_limit
        self.offline_log = Path(offline_log)

        self._q: queue.Queue = queue.Queue(maxsize=queue_size)
        self._session = requests.Session()
        self._session.headers.update({
            "X-API-Key": self.api_key,
            "User-Agent": f"RoadSense-Pi/{self.vehicle_id}",
        })

        self._worker = threading.Thread(target=self._send_loop, daemon=True)
        self._worker.start()
        print(f"[EdgeClient] Ready. API: {self.base}  Vehicle: {self.vehicle_id}")

    def post_detection(
        self,
        latitude: float,
        longitude: float,
        confidence: float,
        severity: str,
        frame_id: int = 0,
        bbox_w: int = 0,
        bbox_h: int = 0,
        image_path: str = "",
    ):
        """
        Enqueue a detection for async dispatch.
        Non-blocking — returns immediately.
        """
        record = {
            "id": f"PI-{uuid.uuid4().hex[:8].upper()}",
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "latitude": round(float(latitude), 6),
            "longitude": round(float(longitude), 6),
            "confidence": round(float(confidence), 4),
            "severity": severity.upper(),
            "source": "live",
            "vehicle_id": self.vehicle_id,
            "frame_id": frame_id,
            "bbox_w": bbox_w,
            "bbox_h": bbox_h,
            "image_path": image_path,
        }
        try:
            self._q.put_nowait(record)
        except queue.Full:
            # Queue full means the Pi is offline and we're accumulating
            # Write to offline log instead
            self._write_offline(record)

    def _send_loop(self):
        while True:
            record = self._q.get()
            self._send_with_retry(record)

    def _send_with_retry(self, record: dict, attempt: int = 0):
        try:
            r = self._session.post(
                f"{self.base}/api/detections",
                json=record,
                timeout=8,
            )
            if r.status_code == 201:
                return True
            if r.status_code == 429:
                # Rate limited — back off
                retry_after = int(r.headers.get("Retry-After", 10))
                time.sleep(retry_after)
                if attempt < self.retry_limit:
                    return self._send_with_retry(record, attempt + 1)
            print(f"[EdgeClient] HTTP {r.status_code}: {r.text[:80]}")
            return False
        except requests.ConnectionError:
            if attempt < self.retry_limit:
                time.sleep(2 ** attempt)  # Exponential backoff: 1s, 2s, 4s
                return self._send_with_retry(record, attempt + 1)
            # Give up — log for later replay
            self._write_offline(record)
            return False
        except Exception as e:
            print(f"[EdgeClient] Error: {e}")
            self._write_offline(record)
            return False

    def _write_offline(self, record: dict):
        """Persist failed records to JSONL for later replay."""
        try:
            with open(self.offline_log, "a") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            print(f"[EdgeClient] Could not write offline log: {e}")

    def replay_offline_log(self):
        """
        Call this on startup to send any detections that were
        buffered while the Pi was offline.
        """
        if not self.offline_log.exists():
            return
        lines = self.offline_log.read_text().strip().splitlines()
        if not lines:
            return
        print(f"[EdgeClient] Replaying {len(lines)} offline detections...")
        self.offline_log.unlink()  # Clear before replaying to avoid duplicates
        for line in lines:
            try:
                record = json.loads(line)
                record["source"] = "replay"
                self._q.put(record)
            except json.JSONDecodeError:
                pass


def _get_pi_serial() -> str:
    """Read Raspberry Pi CPU serial as a unique device ID."""
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("Serial"):
                    return "PI-" + line.split(":")[1].strip()[-8:]
    except Exception:
        pass
    return "PI-UNKNOWN"


# ── Quick test ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import random
    client = EdgeClient(api_url="https://visionx-production-425c.up.railway.app",
                        api_key="dev-device-key-raspberry-pi")
    client.replay_offline_log()

    print("Sending 5 test detections...")
    for i in range(5):
        r = random.random()
        sev = "HIGH" if r < 0.25 else ("MEDIUM" if r < 0.6 else "LOW")
        client.post_detection(
            latitude=13.0827 + (random.random() - 0.5) * 0.02,
            longitude=80.2707 + (random.random() - 0.5) * 0.02,
            confidence=round(random.uniform(0.45, 0.97), 3),
            severity=sev,
            frame_id=i * 100,
            bbox_w=random.randint(40, 140),
            bbox_h=random.randint(30, 100),
        )
        time.sleep(1)
    time.sleep(2)
    print("Done.")
