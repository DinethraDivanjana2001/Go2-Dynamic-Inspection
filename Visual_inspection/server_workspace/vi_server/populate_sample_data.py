"""
Populate database with sample location data for testing.

This script creates sample areas and inspection points for testing
without needing a robot.
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sqlalchemy import select
from app.db import AsyncSessionLocal
from app.models import Area, InspectionPoint


async def populate_sample_data():
    """Populate database with sample areas and inspection points."""
    
    print("=" * 70)
    print("POPULATING DATABASE WITH SAMPLE DATA")
    print("=" * 70)
    
    async with AsyncSessionLocal() as db:
        # Check if data already exists
        result = await db.execute(select(Area))
        existing_areas = result.scalars().all()
        
        if existing_areas:
            print(f"\n⚠ Found {len(existing_areas)} existing area(s)")
            response = input("Do you want to add more sample data? (y/n): ")
            if response.lower() != 'y':
                print("Cancelled.")
                return
        
        # Create sample areas
        print("\n1. Creating sample areas...")
        
        areas_data = [
            {
                "name": "Floor 1 - Main Hall",
                "description": "Main production floor",
                "floor_number": 1,
                "map_width": 30.0,
                "map_height": 20.0,
                "origin_x": 0.0,
                "origin_y": 0.0
            },
            {
                "name": "Floor 2 - Storage",
                "description": "Storage and maintenance area",
                "floor_number": 2,
                "map_width": 25.0,
                "map_height": 15.0,
                "origin_x": 0.0,
                "origin_y": 0.0
            },
            {
                "name": "Basement - Utilities",
                "description": "Utility systems and equipment",
                "floor_number": -1,
                "map_width": 20.0,
                "map_height": 12.0,
                "origin_x": 0.0,
                "origin_y": 0.0
            }
        ]
        
        created_areas = []
        for area_data in areas_data:
            area = Area(**area_data)
            db.add(area)
            await db.flush()  # Get the ID
            created_areas.append(area)
            print(f"   ✓ Created: {area.name} (ID: {area.id})")
        
        # Create sample inspection points for Floor 1
        print("\n2. Creating sample inspection points for Floor 1...")
        
        floor1 = created_areas[0]
        
        points_data = [
            {
                "area_id": floor1.id,
                "name": "Pressure Gauge 1",
                "object_type": "gauge",
                "x": 5.5,
                "y": 3.2,
                "z": 1.5,
                "orientation": 0.0,
                "notes": "Main pressure line gauge"
            },
            {
                "area_id": floor1.id,
                "name": "Pressure Gauge 2",
                "object_type": "gauge",
                "x": 12.8,
                "y": 7.5,
                "z": 1.5,
                "orientation": 1.57,
                "notes": "Secondary pressure gauge"
            },
            {
                "area_id": floor1.id,
                "name": "Fire Extinguisher 1",
                "object_type": "fire_extinguisher",
                "x": 2.0,
                "y": 1.5,
                "z": 0.0,
                "orientation": 0.0,
                "notes": "Near entrance"
            },
            {
                "area_id": floor1.id,
                "name": "Fire Extinguisher 2",
                "object_type": "fire_extinguisher",
                "x": 18.5,
                "y": 10.2,
                "z": 0.0,
                "orientation": 3.14,
                "notes": "Near emergency exit"
            },
            {
                "area_id": floor1.id,
                "name": "Emergency Exit 1",
                "object_type": "emergency_exit",
                "x": 20.0,
                "y": 10.0,
                "z": 0.0,
                "orientation": 1.57,
                "notes": "Main emergency exit"
            },
            {
                "area_id": floor1.id,
                "name": "Main Door",
                "object_type": "door",
                "x": 1.0,
                "y": 5.0,
                "z": 0.0,
                "orientation": 0.0,
                "notes": "Main entrance door"
            },
            {
                "area_id": floor1.id,
                "name": "Hydraulic Cylinder 1",
                "object_type": "main_cylinder",
                "x": 15.0,
                "y": 5.0,
                "z": 0.0,
                "orientation": 0.0,
                "notes": "Main hydraulic system"
            }
        ]
        
        for point_data in points_data:
            point = InspectionPoint(**point_data)
            db.add(point)
            print(f"   ✓ Created: {point.name} at ({point.x}, {point.y})")
        
        # Create sample inspection points for Floor 2
        print("\n3. Creating sample inspection points for Floor 2...")
        
        floor2 = created_areas[1]
        
        floor2_points = [
            {
                "area_id": floor2.id,
                "name": "Storage Door 1",
                "object_type": "door",
                "x": 5.0,
                "y": 2.0,
                "z": 0.0,
                "orientation": 0.0,
                "notes": "Storage room A"
            },
            {
                "area_id": floor2.id,
                "name": "Fire Extinguisher 3",
                "object_type": "fire_extinguisher",
                "x": 10.0,
                "y": 7.5,
                "z": 0.0,
                "orientation": 0.0,
                "notes": "Central location"
            },
            {
                "area_id": floor2.id,
                "name": "Emergency Exit 2",
                "object_type": "emergency_exit",
                "x": 20.0,
                "y": 7.5,
                "z": 0.0,
                "orientation": 1.57,
                "notes": "Floor 2 emergency exit"
            }
        ]
        
        for point_data in floor2_points:
            point = InspectionPoint(**point_data)
            db.add(point)
            print(f"   ✓ Created: {point.name} at ({point.x}, {point.y})")
        
        # Commit all changes
        await db.commit()
        
        print("\n" + "=" * 70)
        print("SAMPLE DATA CREATED SUCCESSFULLY!")
        print("=" * 70)
        print(f"\nCreated:")
        print(f"  - {len(created_areas)} areas")
        print(f"  - {len(points_data) + len(floor2_points)} inspection points")
        print("\nYou can now:")
        print("  1. Start the server")
        print("  2. Open location_ui.html")
        print("  3. Select an area to see the inspection points")
        print("=" * 70)


if __name__ == "__main__":
    import sys
    from app.db import engine
    
    try:
        asyncio.run(populate_sample_data())
    finally:
        # Cleanup
        asyncio.run(engine.dispose())
        print("\nScript finished. Press Ctrl+C if needed.")
        sys.exit(0)
