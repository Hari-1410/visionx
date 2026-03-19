import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    VERSION = "1.0.0"
    PORT = int(os.getenv("PORT", 5000))
    DEBUG = os.getenv("DEBUG", "false").lower() == "true"
    ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "*").split(",")
    DB_PATH = os.getenv("DB_PATH", "roadsense.db")
    IMAGE_DIR = os.getenv("IMAGE_DIR", "static/detections")
    API_KEYS: dict[str, str] = {}

    @classmethod
    def _load_keys(cls):
        raw = os.getenv("API_KEYS", "")
        if raw:
            for pair in raw.split(","):
                pair = pair.strip()
                if ":" in pair:
                    k, r = pair.split(":", 1)
                    cls.API_KEYS[k.strip()] = r.strip()

        if not cls.API_KEYS:
            cls.API_KEYS = {
                "dev-device-key-raspberry-pi": "device",
                "dev-device-key-carla-sim": "device",
                "dev-dashboard-key-browser": "dashboard",
                "dev-admin-key-local": "admin",
            }
            print("[WARNING] Using dev API keys.")

    DEVICE_RATE_LIMIT = int(os.getenv("DEVICE_RATE_LIMIT", 120))
    DASHBOARD_RATE_LIMIT = int(os.getenv("DASHBOARD_RATE_LIMIT", 120))

Config._load_keys()