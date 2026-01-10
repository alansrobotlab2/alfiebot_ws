"""
Model manager for offline ASR model storage.
Handles downloading, caching, and loading of ASR and VAD models.
"""

import os
import shutil
import socket
from pathlib import Path


# Model configuration
ASR_REPO_ID = "istupakov/parakeet-tdt-0.6b-v2-onnx"
ASR_QUANTIZATION = "int8"
ASR_REQUIRED_FILES = [
    "config.json",
    "vocab.txt",
]
ASR_MODEL_PATTERNS = [
    "encoder-model*int8.onnx",
    "decoder_joint-model*int8.onnx",
]

VAD_MODEL_NAME = "silero_vad.onnx"


def get_models_dir() -> Path:
    """Get the models directory path in the package share directory."""
    # Use a persistent location that survives rebuilds
    # Store in user's home directory under .alfie_asr/models
    models_dir = Path.home() / ".alfie_asr" / "models"
    models_dir.mkdir(parents=True, exist_ok=True)
    return models_dir


def get_asr_model_dir() -> Path:
    """Get the ASR model directory path."""
    asr_dir = get_models_dir() / "parakeet-tdt-0.6b-v2-onnx"
    asr_dir.mkdir(parents=True, exist_ok=True)
    return asr_dir


def get_vad_model_path() -> Path:
    """Get the VAD model file path."""
    return get_models_dir() / VAD_MODEL_NAME


def check_internet_connection(host: str = "huggingface.co", port: int = 443, timeout: float = 3.0) -> bool:
    """Check if internet connection is available."""
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except (socket.error, socket.timeout):
        return False


def check_asr_model_exists() -> bool:
    """Check if all required ASR model files exist."""
    asr_dir = get_asr_model_dir()

    # Check required files
    for filename in ASR_REQUIRED_FILES:
        if not (asr_dir / filename).exists():
            return False

    # Check model files (using glob patterns)
    import glob
    for pattern in ASR_MODEL_PATTERNS:
        matches = list(asr_dir.glob(pattern))
        if not matches:
            return False

    return True


def check_vad_model_exists() -> bool:
    """Check if VAD model file exists."""
    return get_vad_model_path().exists()


def download_asr_model() -> Path:
    """Download ASR model from HuggingFace Hub."""
    from huggingface_hub import snapshot_download

    asr_dir = get_asr_model_dir()

    print(f"Downloading Parakeet ASR model to {asr_dir}...")

    # Download the model files
    snapshot_download(
        repo_id=ASR_REPO_ID,
        local_dir=str(asr_dir),
        allow_patterns=[
            "config.json",
            "vocab.txt",
            "*int8.onnx",
            "*int8.onnx.data",
        ],
    )

    print("ASR model download complete.")
    return asr_dir


def copy_vad_model() -> Path:
    """Copy bundled Silero VAD model to models directory."""
    vad_path = get_vad_model_path()

    if vad_path.exists():
        return vad_path

    print(f"Copying Silero VAD model to {vad_path}...")

    # Find the bundled model in the silero_vad package
    try:
        import importlib.resources as pkg_resources
        import silero_vad

        # Get the package data directory
        if hasattr(pkg_resources, 'files'):
            # Python 3.9+
            data_path = pkg_resources.files(silero_vad) / "data" / VAD_MODEL_NAME
            with pkg_resources.as_file(data_path) as source_path:
                shutil.copy2(source_path, vad_path)
        else:
            # Fallback for older Python
            with pkg_resources.path(silero_vad, "data") as data_dir:
                source_path = data_dir / VAD_MODEL_NAME
                shutil.copy2(source_path, vad_path)
    except Exception as e:
        # Fallback: try to find silero_vad package location directly
        import silero_vad
        package_dir = Path(silero_vad.__file__).parent
        source_path = package_dir / "data" / VAD_MODEL_NAME
        if source_path.exists():
            shutil.copy2(source_path, vad_path)
        else:
            raise RuntimeError(f"Could not find bundled Silero VAD model: {e}")

    print("VAD model copy complete.")
    return vad_path


class ModelNotAvailableError(Exception):
    """Raised when models are not available and cannot be downloaded."""
    pass


def ensure_models() -> tuple[Path, Path]:
    """
    Ensure all required models are available locally.

    Returns:
        Tuple of (asr_model_dir, vad_model_path)

    Raises:
        ModelNotAvailableError: If models are not available and cannot be downloaded
    """
    asr_available = check_asr_model_exists()
    vad_available = check_vad_model_exists()

    # If both models exist, return paths
    if asr_available and vad_available:
        print("All models found locally.")
        return get_asr_model_dir(), get_vad_model_path()

    # Try to download/copy missing models
    if not vad_available:
        try:
            copy_vad_model()
            vad_available = True
        except Exception as e:
            raise ModelNotAvailableError(
                f"VAD model not available and could not be copied from package: {e}"
            )

    if not asr_available:
        # Check internet connection
        if not check_internet_connection():
            raise ModelNotAvailableError(
                "ASR model not available locally and no internet connection. "
                "Please connect to the internet to download the model, or manually "
                f"place model files in: {get_asr_model_dir()}"
            )

        try:
            download_asr_model()
        except Exception as e:
            raise ModelNotAvailableError(
                f"Failed to download ASR model: {e}"
            )

    return get_asr_model_dir(), get_vad_model_path()
