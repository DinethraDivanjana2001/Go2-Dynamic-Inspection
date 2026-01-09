"""File storage operations for job data and images."""

import shutil
from pathlib import Path
from typing import BinaryIO

from fastapi import UploadFile

from app.settings import settings


def create_job_directory(job_id: str) -> Path:
    """
    Create a directory for storing job files.
    
    Args:
        job_id: Unique job identifier
        
    Returns:
        Path to the created job directory
    """
    job_dir = settings.get_job_dir(job_id)
    job_dir.mkdir(parents=True, exist_ok=True)
    return job_dir


async def save_roi_image(job_id: str, file: UploadFile) -> str:
    """
    Save uploaded ROI image to job directory.
    
    Args:
        job_id: Unique job identifier
        file: Uploaded file from FastAPI
        
    Returns:
        Relative path to saved file (relative to storage_root)
    """
    # Create job directory
    job_dir = create_job_directory(job_id)
    
    # Determine file extension
    original_filename = file.filename or "roi.jpg"
    extension = Path(original_filename).suffix or ".jpg"
    
    # Save file
    roi_filename = f"roi{extension}"
    roi_path = job_dir / roi_filename
    
    with roi_path.open("wb") as buffer:
        shutil.copyfileobj(file.file, buffer)
    
    # Return relative path
    return f"{job_id}/{roi_filename}"


def get_roi_path(job_id: str, roi_filename: str) -> Path:
    """
    Get the absolute path to a ROI image.
    
    Args:
        job_id: Unique job identifier
        roi_filename: Relative filename (e.g., "job_id/roi.jpg")
        
    Returns:
        Absolute path to the ROI image
    """
    return settings.storage_root / roi_filename


def roi_exists(job_id: str, roi_filename: str) -> bool:
    """
    Check if ROI image exists.
    
    Args:
        job_id: Unique job identifier
        roi_filename: Relative filename
        
    Returns:
        True if file exists, False otherwise
    """
    roi_path = get_roi_path(job_id, roi_filename)
    return roi_path.exists() and roi_path.is_file()


def ensure_storage_directories() -> None:
    """Ensure all required storage directories exist."""
    settings.storage_root.mkdir(parents=True, exist_ok=True)
