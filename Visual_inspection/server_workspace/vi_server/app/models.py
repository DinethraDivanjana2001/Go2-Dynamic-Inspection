"""Database models for the visual inspection server."""

from datetime import datetime
from typing import Optional

from sqlalchemy import Boolean, Column, DateTime, Float, ForeignKey, Integer, String, Text
from sqlalchemy.orm import declarative_base, relationship

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
    
    # Location reference (optional - for known location inspections)
    inspection_point_id = Column(Integer, ForeignKey("inspection_points.id"), nullable=True, index=True)

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
            "inspection_point_id": self.inspection_point_id,
        }


class Area(Base):
    """Area/Floor model for organizing inspection locations."""
    
    __tablename__ = "areas"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    name = Column(String(100), nullable=False, unique=True)  # e.g., "Floor 1", "Area A"
    description = Column(Text, nullable=True)
    floor_number = Column(Integer, nullable=True)
    
    # Map image for 2D coordinate system
    map_image_path = Column(String(255), nullable=True)
    
    # Coordinate system metadata
    map_width = Column(Float, nullable=True)  # Physical width in meters
    map_height = Column(Float, nullable=True)  # Physical height in meters
    origin_x = Column(Float, default=0.0)  # Origin offset
    origin_y = Column(Float, default=0.0)
    
    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationship
    inspection_points = relationship("InspectionPoint", back_populates="area", cascade="all, delete-orphan")
    
    def __repr__(self) -> str:
        return f"<Area(id={self.id}, name={self.name})>"
    
    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "floor_number": self.floor_number,
            "map_image_path": self.map_image_path,
            "map_width": self.map_width,
            "map_height": self.map_height,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }


class InspectionPoint(Base):
    """Known inspection point with saved coordinates."""
    
    __tablename__ = "inspection_points"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    area_id = Column(Integer, ForeignKey("areas.id"), nullable=False, index=True)
    
    # Point identification
    name = Column(String(100), nullable=False)  # e.g., "Gauge 1", "Fire Extinguisher 2"
    object_type = Column(String(50), nullable=False)  # gauge, fire_extinguisher, etc.
    
    # Coordinates (in robot's coordinate frame)
    x = Column(Float, nullable=False)
    y = Column(Float, nullable=False)
    z = Column(Float, default=0.0)  # Height if needed
    orientation = Column(Float, default=0.0)  # Yaw angle in radians
    
    # Status
    is_active = Column(Boolean, default=True)
    
    # Additional metadata
    notes = Column(Text, nullable=True)
    last_inspected_at = Column(DateTime, nullable=True)
    
    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationship
    area = relationship("Area", back_populates="inspection_points")
    
    def __repr__(self) -> str:
        return f"<InspectionPoint(id={self.id}, name={self.name}, area_id={self.area_id})>"
    
    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "area_id": self.area_id,
            "name": self.name,
            "object_type": self.object_type,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "orientation": self.orientation,
            "is_active": self.is_active,
            "notes": self.notes,
            "last_inspected_at": self.last_inspected_at.isoformat() if self.last_inspected_at else None,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }
