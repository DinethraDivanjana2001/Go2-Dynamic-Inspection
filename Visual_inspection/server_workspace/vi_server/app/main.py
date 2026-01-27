"""Main FastAPI application with routes and lifecycle management."""

import json
import uuid
from datetime import datetime
from typing import List, Optional

from fastapi import Depends, FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.db import get_db, init_db
from app.models import Job
from app.queue_worker import job_queue
from app.routes_location import router as location_router
from app.schemas import (
    HealthResponse,
    JobAcceptedResponse,
    JobListResponse,
    JobResponse,
    JobSummary,
)
from app.settings import settings
from app.storage import ensure_storage_directories, get_roi_path, roi_exists, save_roi_image
from app.utils.image_utils import validate_image
from app.utils.logging import get_logger, setup_logging

# Setup logging
setup_logging()
logger = get_logger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Visual Inspection Server",
    description="Backend server for robot visual inspection system",
    version="0.1.0",
)

# Add CORS middleware to allow browser UI to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for local development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include location management routes
app.include_router(location_router)


@app.on_event("startup")
async def startup_event():
    """Initialize application on startup."""
    logger.info("Starting Visual Inspection Server...")
    
    # Ensure storage directories exist
    ensure_storage_directories()
    logger.info(f"Storage root: {settings.storage_root}")
    
    # Initialize database
    await init_db()
    logger.info("Database initialized")
    
    # Start background worker
    await job_queue.start_worker()
    logger.info("Background worker started")
    
    logger.info(f"Server ready on {settings.server_host}:{settings.server_port}")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on application shutdown."""
    logger.info("Shutting down Visual Inspection Server...")
    
    # Stop background worker
    await job_queue.stop_worker()
    logger.info("Background worker stopped")


@app.get("/api/v1/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    
    Returns server status and current timestamp.
    """
    return HealthResponse(
        status="ok",
        timestamp=datetime.utcnow(),
        version="0.1.0"
    )


@app.post("/api/v1/jobs", response_model=JobAcceptedResponse)
async def create_job(
    file: UploadFile = File(..., description="ROI image file"),
    object_type: str = Form(..., description="Object type (gauge/door/fire_extinguisher/unknown)"),
    metadata_json: Optional[str] = Form(None, description="Additional metadata as JSON string"),
    db: AsyncSession = Depends(get_db),
):
    """
    Create a new inspection job.
    
    Accepts an ROI image and metadata, stores the image, creates a job record,
    and enqueues it for processing. Returns immediately with job_id.
    
    Args:
        file: Uploaded ROI image (JPEG or PNG)
        object_type: Type of object to inspect
        metadata_json: Optional JSON string with additional metadata
        db: Database session
        
    Returns:
        Job acceptance response with job_id and status
    """
    logger.info(f"Received job creation request: object_type={object_type}")
    
    # Validate object type
    if object_type not in settings.allowed_object_types:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid object_type. Allowed values: {', '.join(settings.allowed_object_types)}"
        )
    
    # Validate image
    await validate_image(file, max_size_mb=settings.max_upload_size_mb)
    
    # Validate metadata JSON if provided
    if metadata_json:
        try:
            json.loads(metadata_json)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid metadata_json format")
    
    # Generate job ID
    job_id = str(uuid.uuid4())
    logger.info(f"Created job_id: {job_id}")
    
    # Save ROI image
    try:
        roi_filename = await save_roi_image(job_id, file)
        logger.info(f"Saved ROI image: {roi_filename}")
    except Exception as e:
        logger.error(f"Failed to save ROI image: {e}")
        raise HTTPException(status_code=500, detail="Failed to save image")
    
    # Create job record
    job = Job(
        job_id=job_id,
        object_type=object_type,
        status="QUEUED",
        roi_filename=roi_filename,
        metadata_json=metadata_json,
    )
    
    db.add(job)
    await db.commit()
    logger.info(f"Job {job_id} created in database with status QUEUED")
    
    # Enqueue job for processing
    try:
        await job_queue.enqueue(job_id)
        logger.info(f"Job {job_id} enqueued for processing")
    except Exception as e:
        logger.error(f"Failed to enqueue job: {e}")
        # Job is still in database, can be retried manually
    
    return JobAcceptedResponse(
        accepted=True,
        job_id=job_id,
        status="QUEUED"
    )


@app.get("/api/v1/jobs", response_model=JobListResponse)
async def list_jobs(
    limit: int = 20,
    offset: int = 0,
    status: Optional[str] = None,
    object_type: Optional[str] = None,
    db: AsyncSession = Depends(get_db),
):
    """
    List jobs with pagination and filtering.
    
    Args:
        limit: Number of jobs to return (max 100)
        offset: Offset for pagination
        status: Filter by status (optional)
        object_type: Filter by object type (optional)
        db: Database session
        
    Returns:
        Paginated list of jobs
    """
    # Limit maximum page size
    limit = min(limit, 100)
    
    # Build query
    query = select(Job)
    
    # Apply filters
    if status:
        query = query.where(Job.status == status)
    if object_type:
        query = query.where(Job.object_type == object_type)
    
    # Get total count
    count_query = select(func.count()).select_from(query.subquery())
    total_result = await db.execute(count_query)
    total = total_result.scalar() or 0
    
    # Apply pagination and ordering
    query = query.order_by(Job.created_at.desc()).offset(offset).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    jobs = result.scalars().all()
    
    # Convert to summary format
    job_summaries = [
        JobSummary(
            job_id=job.job_id,
            created_at=job.created_at,
            object_type=job.object_type,
            status=job.status,
            error_message=job.error_message,
        )
        for job in jobs
    ]
    
    return JobListResponse(
        total=total,
        limit=limit,
        offset=offset,
        jobs=job_summaries
    )


@app.get("/api/v1/jobs/{job_id}", response_model=JobResponse)
async def get_job(
    job_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Get detailed information about a specific job.
    
    Args:
        job_id: Unique job identifier
        db: Database session
        
    Returns:
        Complete job details including results
    """
    result = await db.execute(select(Job).where(Job.job_id == job_id))
    job = result.scalar_one_or_none()
    
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    
    return JobResponse.model_validate(job)


@app.get("/api/v1/jobs/{job_id}/roi")
async def get_job_roi(
    job_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Retrieve the ROI image for a specific job.
    
    Args:
        job_id: Unique job identifier
        db: Database session
        
    Returns:
        ROI image file
    """
    # Get job from database
    result = await db.execute(select(Job).where(Job.job_id == job_id))
    job = result.scalar_one_or_none()
    
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    
    if not job.roi_filename:
        raise HTTPException(status_code=404, detail="ROI image not available")
    
    # Get image path
    roi_path = get_roi_path(job_id, job.roi_filename)
    
    if not roi_exists(job_id, job.roi_filename):
        raise HTTPException(status_code=404, detail="ROI image file not found")
    
    # Determine media type
    media_type = "image/jpeg"
    if roi_path.suffix.lower() in [".png"]:
        media_type = "image/png"
    
    return FileResponse(
        path=str(roi_path),
        media_type=media_type,
        filename=f"roi_{job_id}{roi_path.suffix}"
    )


if __name__ == "__main__":
    import uvicorn
    
    uvicorn.run(
        "app.main:app",
        host=settings.server_host,
        port=settings.server_port,
        reload=True,
    )
