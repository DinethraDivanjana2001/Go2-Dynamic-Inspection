"""Pydantic schemas for API request/response validation."""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field


class JobCreate(BaseModel):
    """Schema for creating a new job."""
    object_type: str = Field(..., description="Type of object to inspect")
    metadata_json: Optional[str] = Field(None, description="Additional metadata as JSON string")


class JobAcceptedResponse(BaseModel):
    """Response schema for job creation acknowledgment."""
    accepted: bool = Field(True, description="Whether the job was accepted")
    job_id: str = Field(..., description="Unique job identifier")
    status: str = Field(..., description="Current job status")


class JobResponse(BaseModel):
    """Schema for detailed job information."""
    job_id: str
    created_at: datetime
    updated_at: datetime
    object_type: str
    status: str
    roi_filename: Optional[str] = None
    result_json: Optional[str] = None
    error_message: Optional[str] = None
    blur_score: Optional[float] = None
    brightness_mean: Optional[float] = None
    detection_conf: Optional[float] = None
    metadata_json: Optional[str] = None

    class Config:
        from_attributes = True


class JobSummary(BaseModel):
    """Schema for job list summary."""
    job_id: str
    created_at: datetime
    object_type: str
    status: str
    error_message: Optional[str] = None

    class Config:
        from_attributes = True


class JobListResponse(BaseModel):
    """Schema for paginated job list."""
    total: int = Field(..., description="Total number of jobs matching filters")
    limit: int = Field(..., description="Number of jobs per page")
    offset: int = Field(..., description="Offset for pagination")
    jobs: List[JobSummary] = Field(..., description="List of job summaries")


class HealthResponse(BaseModel):
    """Schema for health check response."""
    status: str = Field("ok", description="Health status")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Current server time")
    version: str = Field("0.1.0", description="API version")
