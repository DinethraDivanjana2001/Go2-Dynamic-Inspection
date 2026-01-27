"""
Database migration script to add location management tables.

Run this script to update the database schema with new tables:
- areas
- inspection_points

Usage:
    python migrate_add_locations.py
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from sqlalchemy import text
from app.db import engine, Base
from app.models import Area, InspectionPoint, Job


async def migrate():
    """Run database migration."""
    print("Starting database migration...")
    
    try:
        # Create all tables (will only create new ones)
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        
        print("Migration completed successfully!")
        print("\nNew tables created:")
        print("  - areas")
        print("  - inspection_points")
        print("\nExisting tables updated:")
        print("  - jobs (added inspection_point_id column)")
    finally:
        # Properly dispose of the engine
        await engine.dispose()


if __name__ == "__main__":
    asyncio.run(migrate())
    print("\nMigration script finished. You can now start the server.")
