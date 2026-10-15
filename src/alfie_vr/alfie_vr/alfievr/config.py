"""
Configuration for the VR teleoperation web/websocket servers.

Loads alfie_vr/config.yaml (next to this package, in source or install) with
fallback to defaults. Relative paths in the config (SSL files) resolve against
the alfie_vr package directory, never the process working directory.
"""

import copy
import os
import yaml
from dataclasses import dataclass
from pathlib import Path

# alfie_vr/ package directory (holds config.yaml, cert.pem, key.pem, web-ui/)
PACKAGE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = PACKAGE_DIR / "config.yaml"

DEFAULT_CONFIG = {
    "network": {
        "https_port": 8443,
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
    },
    "ssl": {
        "certfile": "cert.pem",
        "keyfile": "key.pem"
    },
    "robot": {
        "vr_to_robot_scale": 1.0,
    },
}


def load_config(config_path: Path = CONFIG_PATH) -> dict:
    """Load configuration from YAML file with fallback to defaults."""
    config = copy.deepcopy(DEFAULT_CONFIG)
    if config_path.exists():
        try:
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)
            if yaml_config:
                _deep_merge(config, yaml_config)
        except Exception as e:
            print(f"Warning: Could not load config from {config_path}: {e}")
    else:
        print(f"Config file {config_path} not found, using defaults")
    return config


def _deep_merge(base: dict, update: dict):
    """Deep merge update dict into base dict."""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def _resolve(path: str) -> str:
    p = Path(os.path.expanduser(path))
    return str(p if p.is_absolute() else PACKAGE_DIR / p)


_config_data = load_config()


@dataclass
class AlfieVRConfig:
    """Network/SSL settings for the VR servers."""
    https_port: int = _config_data["network"]["https_port"]
    websocket_port: int = _config_data["network"]["websocket_port"]
    host_ip: str = _config_data["network"]["host_ip"]
    certfile: str = _resolve(_config_data["ssl"]["certfile"])
    keyfile: str = _resolve(_config_data["ssl"]["keyfile"])
    enable_vr: bool = True
    vr_to_robot_scale: float = _config_data["robot"]["vr_to_robot_scale"]
    web_root: str = str(PACKAGE_DIR / "web-ui")

    @property
    def ssl_files_exist(self) -> bool:
        return os.path.exists(self.certfile) and os.path.exists(self.keyfile)

    def ensure_ssl_certificates(self) -> bool:
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
