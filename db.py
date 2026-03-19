"""
RoadSense — Database layer
Uses SQLite for simplicity. Swap get_db_conn() for PostgreSQL
by replacing sqlite3 with psycopg2 and adjusting placeholders (? → %s).
"""

import sqlite3
import json
from datetime import datetime, date, timezone
from config import Config


def get_db_conn() -> sqlite3.Connection:
    conn = sqlite3.connect(Config.DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")   # Better concurrent write performance
    conn.execute("PRAGMA synchronous=NORMAL")
    return conn


def init_db():
    """Create tables if they don't exist."""
    with get_db_conn() as conn:
        conn.execute("""
            CREATE TABLE IF NOT EXISTS detections (
                id          TEXT PRIMARY KEY,
                timestamp   TEXT NOT NULL,
                latitude    REAL NOT NULL,
                longitude   REAL NOT NULL,
                confidence  REAL NOT NULL,
                severity    TEXT NOT NULL CHECK(severity IN ('LOW','MEDIUM','HIGH')),
                source      TEXT NOT NULL DEFAULT 'live',
                vehicle_id  TEXT DEFAULT '',
                frame_id    INTEGER DEFAULT 0,
                bbox_w      INTEGER DEFAULT 0,
                bbox_h      INTEGER DEFAULT 0,
                image_path  TEXT DEFAULT '',
                ingested_at TEXT NOT NULL
            )
        """)
        conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON detections(timestamp DESC)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_severity  ON detections(severity)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_source    ON detections(source)")
        conn.commit()
    print(f"[DB] Initialised at {Config.DB_PATH}")


def save_detection(record: dict):
    with get_db_conn() as conn:
        conn.execute("""
            INSERT OR REPLACE INTO detections
              (id, timestamp, latitude, longitude, confidence, severity,
               source, vehicle_id, frame_id, bbox_w, bbox_h, image_path, ingested_at)
            VALUES
              (:id, :timestamp, :latitude, :longitude, :confidence, :severity,
               :source, :vehicle_id, :frame_id, :bbox_w, :bbox_h, :image_path, :ingested_at)
        """, record)
        conn.commit()


def get_detections(
    limit: int = 100,
    offset: int = 0,
    severity: list | None = None,
    source: str | None = None,
    since: str | None = None,
    until: str | None = None,
    det_id: str | None = None,
) -> tuple[list[dict], int]:
    """
    Returns (rows, total_count) matching the given filters.
    """
    conditions = []
    params: list = []

    if det_id:
        conditions.append("id = ?")
        params.append(det_id)
    if severity:
        placeholders = ",".join("?" * len(severity))
        conditions.append(f"severity IN ({placeholders})")
        params.extend([s.upper() for s in severity])
    if source:
        conditions.append("source = ?")
        params.append(source)
    if since:
        conditions.append("timestamp >= ?")
        params.append(since)
    if until:
        conditions.append("timestamp <= ?")
        params.append(until)

    where = ("WHERE " + " AND ".join(conditions)) if conditions else ""

    with get_db_conn() as conn:
        total = conn.execute(f"SELECT COUNT(*) FROM detections {where}", params).fetchone()[0]
        rows = conn.execute(
            f"SELECT * FROM detections {where} ORDER BY timestamp DESC LIMIT ? OFFSET ?",
            params + [limit, offset]
        ).fetchall()

    return [dict(r) for r in rows], total


def get_stats() -> dict:
    today = date.today().isoformat()
    with get_db_conn() as conn:
        total = conn.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
        high  = conn.execute("SELECT COUNT(*) FROM detections WHERE severity='HIGH'").fetchone()[0]
        med   = conn.execute("SELECT COUNT(*) FROM detections WHERE severity='MEDIUM'").fetchone()[0]
        low   = conn.execute("SELECT COUNT(*) FROM detections WHERE severity='LOW'").fetchone()[0]
        today_count = conn.execute(
            "SELECT COUNT(*) FROM detections WHERE timestamp LIKE ?", (f"{today}%",)
        ).fetchone()[0]
        avg_conf_row = conn.execute("SELECT AVG(confidence) FROM detections").fetchone()[0]
        avg_conf = round(avg_conf_row or 0.0, 4)

        # Detections per hour (last 24h) for time chart
        hourly = conn.execute("""
            SELECT substr(timestamp, 1, 13) AS hour, COUNT(*) AS cnt
            FROM detections
            WHERE timestamp >= datetime('now', '-24 hours')
            GROUP BY hour
            ORDER BY hour
        """).fetchall()

    return {
        "total": total,
        "high": high,
        "medium": med,
        "low": low,
        "today": today_count,
        "avg_confidence": avg_conf,
        "hourly": [{"hour": r["hour"], "count": r["cnt"]} for r in hourly],
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
