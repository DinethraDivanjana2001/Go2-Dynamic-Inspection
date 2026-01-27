"""API routes for location management."""

import logging
from datetime import datetime
from typing import List

from fastapi import APIRouter, Depends, HTTPException, UploadFile, File
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from sqlalchemy.orm import selectinload

from app.db import get_db
from app.models import Area, InspectionPoint, Job
from app.schemas_location import (
    AreaCreate,
    AreaResponse,
    AreaUpdate,
    CoordinateRequest,
    CoordinateResponse,
    InspectionPointCreate,
    InspectionPointResponse,
    InspectionPointUpdate,
    SaveCoordinatesRequest,
    KnownLocationInspectionRequest,
    AdhocInspectionRequest,
    InspectionResponse,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/v1", tags=["locations"])


# Area Management Endpoints

@router.post("/areas", response_model=AreaResponse, status_code=201)
async def create_area(
    area_data: AreaCreate,
    db: AsyncSession = Depends(get_db)
):
    """Create a new area/floor."""
    # Check if area with same name exists
    result = await db.execute(select(Area).where(Area.name == area_data.name))
    existing = result.scalar_one_or_none()
    if existing:
        raise HTTPException(status_code=400, detail=f"Area with name '{area_data.name}' already exists")
    
    area = Area(**area_data.model_dump())
    db.add(area)
    await db.commit()
    await db.refresh(area)
    
    logger.info(f"Created area: {area.name} (ID: {area.id})")
    return area.to_dict()


@router.get("/areas", response_model=List[AreaResponse])
async def list_areas(db: AsyncSession = Depends(get_db)):
    """List all areas."""
    result = await db.execute(select(Area).order_by(Area.floor_number, Area.name))
    areas = result.scalars().all()
    return [area.to_dict() for area in areas]


@router.get("/areas/{area_id}", response_model=AreaResponse)
async def get_area(area_id: int, db: AsyncSession = Depends(get_db)):
    """Get area by ID."""
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    return area.to_dict()


@router.put("/areas/{area_id}", response_model=AreaResponse)
async def update_area(
    area_id: int,
    area_data: AreaUpdate,
    db: AsyncSession = Depends(get_db)
):
    """Update an area."""
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    
    # Update fields
    for field, value in area_data.model_dump(exclude_unset=True).items():
        setattr(area, field, value)
    
    await db.commit()
    await db.refresh(area)
    
    logger.info(f"Updated area: {area.name} (ID: {area.id})")
    return area.to_dict()


@router.delete("/areas/{area_id}", status_code=204)
async def delete_area(area_id: int, db: AsyncSession = Depends(get_db)):
    """Delete an area and all its inspection points."""
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    
    await db.delete(area)
    await db.commit()
    
    logger.info(f"Deleted area: {area.name} (ID: {area.id})")


# Inspection Point Management Endpoints

@router.post("/areas/{area_id}/points", response_model=InspectionPointResponse, status_code=201)
async def create_inspection_point(
    area_id: int,
    point_data: InspectionPointCreate,
    db: AsyncSession = Depends(get_db)
):
    """Create a new inspection point in an area."""
    # Verify area exists
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    
    point = InspectionPoint(area_id=area_id, **point_data.model_dump())
    db.add(point)
    await db.commit()
    await db.refresh(point)
    
    logger.info(f"Created inspection point: {point.name} in area {area.name}")
    return point.to_dict()


@router.get("/areas/{area_id}/points", response_model=List[InspectionPointResponse])
async def list_inspection_points(
    area_id: int,
    active_only: bool = True,
    db: AsyncSession = Depends(get_db)
):
    """List all inspection points in an area."""
    query = select(InspectionPoint).where(InspectionPoint.area_id == area_id)
    if active_only:
        query = query.where(InspectionPoint.is_active == True)
    query = query.order_by(InspectionPoint.name)
    
    result = await db.execute(query)
    points = result.scalars().all()
    return [point.to_dict() for point in points]


@router.get("/areas/{area_id}/points/{point_id}", response_model=InspectionPointResponse)
async def get_inspection_point(
    area_id: int,
    point_id: int,
    db: AsyncSession = Depends(get_db)
):
    """Get inspection point by ID."""
    result = await db.execute(
        select(InspectionPoint).where(
            InspectionPoint.id == point_id,
            InspectionPoint.area_id == area_id
        )
    )
    point = result.scalar_one_or_none()
    if not point:
        raise HTTPException(status_code=404, detail=f"Inspection point {point_id} not found in area {area_id}")
    return point.to_dict()


@router.put("/areas/{area_id}/points/{point_id}", response_model=InspectionPointResponse)
async def update_inspection_point(
    area_id: int,
    point_id: int,
    point_data: InspectionPointUpdate,
    db: AsyncSession = Depends(get_db)
):
    """Update an inspection point."""
    result = await db.execute(
        select(InspectionPoint).where(
            InspectionPoint.id == point_id,
            InspectionPoint.area_id == area_id
        )
    )
    point = result.scalar_one_or_none()
    if not point:
        raise HTTPException(status_code=404, detail=f"Inspection point {point_id} not found in area {area_id}")
    
    # Update fields
    for field, value in point_data.model_dump(exclude_unset=True).items():
        setattr(point, field, value)
    
    await db.commit()
    await db.refresh(point)
    
    logger.info(f"Updated inspection point: {point.name}")
    return point.to_dict()


@router.delete("/areas/{area_id}/points/{point_id}", status_code=204)
async def delete_inspection_point(
    area_id: int,
    point_id: int,
    db: AsyncSession = Depends(get_db)
):
    """Delete an inspection point."""
    result = await db.execute(
        select(InspectionPoint).where(
            InspectionPoint.id == point_id,
            InspectionPoint.area_id == area_id
        )
    )
    point = result.scalar_one_or_none()
    if not point:
        raise HTTPException(status_code=404, detail=f"Inspection point {point_id} not found in area {area_id}")
    
    await db.delete(point)
    await db.commit()
    
    logger.info(f"Deleted inspection point: {point.name}")


# Coordinate Setup Endpoints

@router.post("/areas/{area_id}/request-coordinates", response_model=CoordinateRequest)
async def request_coordinates(area_id: int, db: AsyncSession = Depends(get_db)):
    """
    Request robot to send its current coordinates.
    This endpoint returns a message that can be sent to the robot.
    The robot should then call the save-coordinates endpoint.
    """
    # Verify area exists
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    
    logger.info(f"Coordinate request for area: {area.name}")
    return CoordinateRequest(
        message=f"Please navigate to the desired inspection point in {area.name} and send coordinates"
    )


@router.post("/areas/{area_id}/save-coordinates", response_model=InspectionPointResponse, status_code=201)
async def save_coordinates(
    area_id: int,
    coord_data: SaveCoordinatesRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Save coordinates received from robot as a new inspection point.
    This is called by the robot after navigating to the desired location.
    """
    # Verify area exists
    result = await db.execute(select(Area).where(Area.id == area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {area_id} not found")
    
    # Create inspection point
    point = InspectionPoint(
        area_id=area_id,
        name=coord_data.point_name,
        object_type=coord_data.object_type,
        x=coord_data.x,
        y=coord_data.y,
        z=coord_data.z,
        orientation=coord_data.orientation,
        notes=coord_data.notes,
    )
    db.add(point)
    await db.commit()
    await db.refresh(point)
    
    logger.info(f"Saved coordinates for: {point.name} at ({point.x}, {point.y})")
    return point.to_dict()


# Inspection Execution Endpoints

@router.post("/inspections/known", response_model=InspectionResponse)
async def inspect_known_location(
    request: KnownLocationInspectionRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Trigger inspection at a known location.
    Returns coordinates that should be sent to the robot for navigation.
    """
    # Get inspection point with area information
    result = await db.execute(
        select(InspectionPoint).where(
            InspectionPoint.id == request.point_id,
            InspectionPoint.area_id == request.area_id
        )
    )
    point = result.scalar_one_or_none()
    if not point:
        raise HTTPException(
            status_code=404,
            detail=f"Inspection point {request.point_id} not found in area {request.area_id}"
        )
    
    if not point.is_active:
        raise HTTPException(status_code=400, detail=f"Inspection point {point.name} is inactive")
    
    # Get area information
    result = await db.execute(select(Area).where(Area.id == request.area_id))
    area = result.scalar_one_or_none()
    
    logger.info(f"Known location inspection requested: {point.name} at ({point.x}, {point.y})")
    
    return InspectionResponse(
        job_id="pending",  # Will be created when robot sends image
        status="NAVIGATION_REQUESTED",
        message=f"Navigate to {point.name} and capture ROI",
        coordinates={
            "x": point.x,
            "y": point.y,
            "z": point.z,
            "orientation": point.orientation,
            "point_id": point.id,
            "point_name": point.name,
            "object_type": point.object_type,
            # Area information for robot
            "area_id": area.id,
            "area_name": area.name,
            "floor_number": area.floor_number,
        }
    )


@router.post("/inspections/adhoc", response_model=InspectionResponse)
async def inspect_adhoc_location(
    request: AdhocInspectionRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Trigger inspection at an arbitrary location.
    Returns coordinates that should be sent to the robot for navigation.
    """
    # Verify area exists and get full details
    result = await db.execute(select(Area).where(Area.id == request.area_id))
    area = result.scalar_one_or_none()
    if not area:
        raise HTTPException(status_code=404, detail=f"Area {request.area_id} not found")
    
    logger.info(f"Ad-hoc inspection requested in {area.name} at ({request.x}, {request.y})")
    
    return InspectionResponse(
        job_id="pending",  # Will be created when robot sends image
        status="NAVIGATION_REQUESTED",
        message=f"Navigate to coordinates in {area.name} and capture ROI",
        coordinates={
            "x": request.x,
            "y": request.y,
            "z": request.z,
            "orientation": request.orientation,
            # Area information for robot
            "area_id": request.area_id,
            "area_name": area.name,
            "floor_number": area.floor_number,
            "description": area.description,
            "object_type": request.object_type,
        }
    )
