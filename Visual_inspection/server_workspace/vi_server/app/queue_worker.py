"""Background job queue and worker implementation."""

import asyncio
import json
import logging
from datetime import datetime
from typing import Optional

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.db import AsyncSessionLocal
from app.models import Job
from app.pipelines.gauge_pipeline import run_gauge_pipeline
from app.pipelines.vlm_stub import run_vlm_pipeline
from app.settings import settings
from app.storage import get_roi_path

logger = logging.getLogger(__name__)


class JobQueue:
    """Async job queue for background processing."""

    def __init__(self, maxsize: int = 1000):
        """Initialize job queue."""
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=maxsize)
        self.worker_task: Optional[asyncio.Task] = None
        self.is_running = False

    async def enqueue(self, job_id: str) -> None:
        """
        Add a job to the processing queue.
        
        Args:
            job_id: Unique job identifier
        """
        await self.queue.put(job_id)
        logger.info(f"Job {job_id} enqueued. Queue size: {self.queue.qsize()}")

    async def start_worker(self) -> None:
        """Start the background worker task."""
        if self.is_running:
            logger.warning("Worker already running")
            return

        self.is_running = True
        self.worker_task = asyncio.create_task(self._worker_loop())
        logger.info("Background worker started")

    async def stop_worker(self) -> None:
        """Stop the background worker task gracefully."""
        if not self.is_running:
            return

        self.is_running = False
        
        if self.worker_task:
            self.worker_task.cancel()
            try:
                await self.worker_task
            except asyncio.CancelledError:
                pass

        logger.info("Background worker stopped")

    async def _worker_loop(self) -> None:
        """Main worker loop that processes jobs from the queue."""
        logger.info("Worker loop started")

        while self.is_running:
            try:
                # Get job from queue with timeout
                try:
                    job_id = await asyncio.wait_for(
                        self.queue.get(),
                        timeout=settings.worker_poll_interval
                    )
                except asyncio.TimeoutError:
                    continue

                logger.info(f"Processing job: {job_id}")

                # Process the job
                await self._process_job(job_id)

                # Mark task as done
                self.queue.task_done()

            except asyncio.CancelledError:
                logger.info("Worker loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error in worker loop: {e}", exc_info=True)
                # Continue processing other jobs
                continue

    async def _process_job(self, job_id: str) -> None:
        """
        Process a single job.
        
        Args:
            job_id: Unique job identifier
        """
        async with AsyncSessionLocal() as db:
            try:
                # Fetch job from database
                result = await db.execute(select(Job).where(Job.job_id == job_id))
                job = result.scalar_one_or_none()

                if not job:
                    logger.error(f"Job {job_id} not found in database")
                    return

                # Update status to RUNNING
                job.status = "RUNNING"
                job.updated_at = datetime.utcnow()
                await db.commit()
                logger.info(f"Job {job_id} status: RUNNING")

                # Get ROI image path
                if not job.roi_filename:
                    raise ValueError("ROI filename not set")

                roi_path = get_roi_path(job_id, job.roi_filename)
                roi_path_str = str(roi_path.absolute())

                # Parse metadata if available
                metadata = {}
                if job.metadata_json:
                    try:
                        import json
                        metadata = json.loads(job.metadata_json)
                    except json.JSONDecodeError:
                        logger.warning(f"Invalid metadata_json for job {job_id}")

                # Route to appropriate pipeline based on object_type
                result_data = await self._route_pipeline(job.object_type, roi_path_str, metadata)

                # Store result
                job.result_json = json.dumps(result_data)
                job.status = "DONE"
                job.updated_at = datetime.utcnow()
                await db.commit()

                logger.info(f"Job {job_id} completed successfully")

            except Exception as e:
                logger.error(f"Error processing job {job_id}: {e}", exc_info=True)

                # Update job status to FAILED
                try:
                    job.status = "FAILED"
                    job.error_message = str(e)
                    job.updated_at = datetime.utcnow()
                    await db.commit()
                except Exception as db_error:
                    logger.error(f"Failed to update job status: {db_error}")

    async def _route_pipeline(self, object_type: str, roi_path: str, metadata: dict = None) -> dict:
        """
        Route job to appropriate pipeline based on object type.
        
        Args:
            object_type: Type of object to process
            roi_path: Absolute path to ROI image
            metadata: Optional metadata dictionary from job
            
        Returns:
            Pipeline result dictionary
        """
        logger.info(f"Routing to pipeline for object_type: {object_type}")

        # Run pipeline in thread pool to avoid blocking
        loop = asyncio.get_event_loop()

        if object_type == "gauge":
            # Gauge reading pipeline
            result = await loop.run_in_executor(None, run_gauge_pipeline, roi_path)
        else:
            # VLM-based reasoning pipeline for all other types
            from app.pipelines.vlm import run_vlm_task
            
            result = await loop.run_in_executor(
                None, 
                run_vlm_task, 
                object_type, 
                roi_path, 
                metadata
            )

        return result


# Global job queue instance
job_queue = JobQueue(maxsize=settings.max_queue_size)
