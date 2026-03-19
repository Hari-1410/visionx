# RoadSense Backend

Edge AI pothole detection — Flask API server with SSE streaming.

## File Structure

```
roadsense_backend/
├── app.py              # Flask API — auth, routes, SSE broker
├── db.py               # SQLite layer — init, save, query
├── config.py           # API keys, rate limits (loads from .env)
├── edge_client.py      # Raspberry Pi push client (non-blocking)
├── carla_client.py     # CARLA simulation client
├── requirements.txt
├── .env.example        # Copy to .env and fill in
└── static/
    ├── index.html      # Dashboard (served by Flask)
    └── detections/     # Auto-created — detection images saved here
```

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Configure
cp .env.example .env
# Edit .env — at minimum set API_KEYS

# 3. Run
python app.py
# → http://localhost:5000
```

Open `http://localhost:5000/static/index.html` in a browser.

## API Keys

Generate keys:
```bash
python -c "import secrets; print(secrets.token_hex(32))"
```

Set in `.env`:
```
API_KEYS=abc123:device,def456:dashboard,ghi789:admin
```

Roles:
- `device` — can POST detections only
- `dashboard` — can GET detections, stats, and subscribe to SSE
- `admin` — all of the above

## API Endpoints

| Method | Path | Role | Description |
|--------|------|------|-------------|
| POST | `/api/detections` | device | Ingest a detection (JSON or multipart) |
| GET  | `/api/detections` | dashboard | Query detections with filters |
| GET  | `/api/detections/<id>` | dashboard | Single detection |
| GET  | `/api/stats` | dashboard | Summary stats + hourly chart data |
| GET  | `/api/stream?key=<key>` | dashboard | SSE live stream |
| GET  | `/health` | public | Health check |

All requests (except `/health`) require `X-API-Key` header.
SSE endpoint accepts key via `?key=` query param (browsers can't set headers on EventSource).

## Sending a Detection (cURL)

```bash
curl -X POST http://localhost:5000/api/detections \
  -H "X-API-Key: your-device-key" \
  -H "Content-Type: application/json" \
  -d '{
    "timestamp": "2026-03-14 14:32:18",
    "latitude": 13.0827,
    "longitude": 80.2707,
    "confidence": 0.87,
    "severity": "HIGH",
    "source": "live",
    "vehicle_id": "PI-4B-001"
  }'
```

With image:
```bash
curl -X POST http://localhost:5000/api/detections \
  -H "X-API-Key: your-device-key" \
  -F "latitude=13.0827" \
  -F "longitude=80.2707" \
  -F "confidence=0.87" \
  -F "severity=HIGH" \
  -F "source=live" \
  -F "image=@/path/to/frame.jpg"
```

## Raspberry Pi Integration

```python
from edge_client import EdgeClient

client = EdgeClient(
    api_url="http://your-server:5000",
    api_key="your-device-key",
    vehicle_id="PI-BUS-07",
)
client.replay_offline_log()   # Send buffered detections from last offline period

# In your inference loop:
for frame in camera_stream():
    detections = model.detect(frame)
    for det in detections:
        lat, lon = gps.get_location()
        client.post_detection(lat, lon, det.confidence, det.severity,
                              frame_id=frame.id, bbox_w=det.w, bbox_h=det.h)
```

The `EdgeClient` is non-blocking — detections are sent in a background thread.
If the server is unreachable, they're written to `offline_queue.jsonl` and replayed on next startup.

## CARLA Integration

```bash
# Start CARLA server first (separate terminal):
./CarlaUE4.sh -quality-level=Low

# Run the CARLA client:
python carla_client.py \
    --host 127.0.0.1 --port 2000 \
    --api-url http://localhost:5000 \
    --api-key your-device-key \
    --model-path ../models/pothole.tflite \
    --frame-skip 5

# Without CARLA installed (mock mode):
python carla_client.py --api-key your-device-key
```

## Production Deployment

```bash
# Run with gunicorn (multi-threaded for SSE)
gunicorn app:app \
    --workers 1 \
    --threads 8 \
    --worker-class gthread \
    --bind 0.0.0.0:5000

# Important: use --workers 1 so all SSE clients share the same broker instance.
# Use threads for concurrency instead.
```

Nginx config for SSE proxying:
```nginx
location /api/stream {
    proxy_pass         http://127.0.0.1:5000;
    proxy_http_version 1.1;
    proxy_set_header   Connection '';
    proxy_buffering    off;
    proxy_cache        off;
    chunked_transfer_encoding on;
}

location / {
    proxy_pass http://127.0.0.1:5000;
}
```

## Upgrading to PostgreSQL

In `db.py`, replace:
```python
import sqlite3
conn = sqlite3.connect(Config.DB_PATH)
```
With:
```python
import psycopg2, psycopg2.extras
conn = psycopg2.connect(Config.DB_URL)
conn.cursor_factory = psycopg2.extras.RealDictCursor
```
Also change `?` placeholders to `%s` throughout `db.py`.
