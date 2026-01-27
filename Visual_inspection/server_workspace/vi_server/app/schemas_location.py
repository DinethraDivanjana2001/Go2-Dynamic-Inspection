"""Pydantic schemas for location management API."""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


# Area Schemas
class AreaCreate(BaseModel):
    """Schema for creating a new area."""
    name: str = Field(..., min_length=1, max_length=100)
    description: Optional[str] = None
    floor_number: Optional[int] = None
    map_width: Optional[float] = None
    map_height: Optional[float] = None
    origin_x: float = 0.0
    origin_y: float = 0.0


class AreaUpdate(BaseModel):
    """Schema for updating an area."""
    name: Optional[str] = Field(None, min_length=1, max_length=100)
    description: Optional[str] = None
    floor_number: Optional[int] = None
    map_width: Optional[float] = None
    map_height: Optional[float] = None
    origin_x: Optional[float] = None
    origin_y: Optional[float] = None


class AreaResponse(BaseModel):
    """Schema for area response."""
    id: int
    name: str
    description: Optional[str]
    floor_number: Optional[int]
    map_image_path: Optional[str]
    map_width: Optional[float]
    map_height: Optional[float]
    origin_x: float
    origin_y: float
    created_at: str
    updated_at: Optional[str]

    class Config:
        from_attributes = True


# Inspection Point Schemas
class InspectionPointCreate(BaseModel):
    """Schema for creating a new inspection point."""
    name: str = Field(..., min_length=1, max_length=100)
    object_type: str = Field(..., min_length=1, max_length=50)
    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0
    notes: Optional[str] = None


class InspectionPointUpdate(BaseModel):
    """Schema for updating an inspection point."""
    name: Optional[str] = Field(None, min_length=1, max_length=100)
    object_type: Optional[str] = Field(None, min_length=1, max_length=50)
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    orientation: Optional[float] = None
    is_active: Optional[bool] = None
    notes: Optional[str] = None


class InspectionPointResponse(BaseModel):
    """Schema for inspection point response."""
    id: int
    area_id: int
    name: str
    object_type: str
    x: float
    y: float
    z: float
    orientation: float
    is_active: bool
    notes: Optional[str]
    last_inspected_at: Optional[str]
    created_at: str
    updated_at: Optional[str]

    class Config:
        from_attributes = True


# Coordinate Request/Response Schemas
class CoordinateRequest(BaseModel):
    """Schema for requesting robot's current coordinates."""
    message: str = "Please navigate to the desired location and send coordinates"


class CoordinateResponse(BaseModel):
    """Schema for robot sending its current coordinates."""
    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0
    timestamp: Optional[str] = None


class SaveCoordinatesRequest(BaseModel):
    """Schema for saving coordinates to an inspection point."""
    point_name: str
    object_type: str
    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0
    notes: Optional[str] = None


# Inspection Execution Schemas
class KnownLocationInspectionRequest(BaseModel):
    """Schema for inspecting a known location."""
    area_id: int
    point_id: int


class AdhocInspectionRequest(BaseModel):
    """Schema for ad-hoc inspection at arbitrary coordinates."""
    area_id: int
    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0
    object_type: Optional[str] = "unknown"  # Can be specified or auto-detected


class InspectionResponse(BaseModel):
    """Schema for inspection execution response."""
    job_id: str
    status: str
    message: str
    coordinates: dict
