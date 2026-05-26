"""Database models for the visual inspection server."""

from datetime import datetime
from typing import Optional

from sqlalchemy import JSON, Column, DateTime, Float, String, Text
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class Job(Base):
    """Job model representing a visual inspection task."""

    __tablename__ = "jobs"

    # Primary identifier
    job_id = Column(String(36), primary_key=True, index=True)

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Job metadata
    object_type = Column(String(50), nullable=False, index=True)
    status = Column(String(20), nullable=False, index=True, default="RECEIVED")
    # Status values: RECEIVED, QUEUED, RUNNING, DONE, FAILED

    # File storage
    roi_filename = Column(String(255), nullable=True)

    # Results
    result_json = Column(Text, nullable=True)  # JSON string
    error_message = Column(Text, nullable=True)

    # Quality metrics (optional)
    blur_score = Column(Float, nullable=True)
    brightness_mean = Column(Float, nullable=True)
    detection_conf = Column(Float, nullable=True)

    # Additional metadata
    metadata_json = Column(Text, nullable=True)  # JSON string

    def __repr__(self) -> str:
        return f"<Job(job_id={self.job_id}, object_type={self.object_type}, status={self.status})>"

    def to_dict(self) -> dict:
        """Convert job to dictionary representation."""
        return {
            "job_id": self.job_id,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
            "object_type": self.object_type,
            "status": self.status,
            "roi_filename": self.roi_filename,
            "result_json": self.result_json,
            "error_message": self.error_message,
            "blur_score": self.blur_score,
            "brightness_mean": self.brightness_mean,
            "detection_conf": self.detection_conf,
            "metadata_json": self.metadata_json,
        }
