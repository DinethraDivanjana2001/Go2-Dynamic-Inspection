"""Application settings and configuration management."""

from pathlib import Path
from typing import List

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Database
    database_url: str = Field(
        default="sqlite+aiosqlite:///./data/vi_server.db",
        description="Database connection URL"
    )

    # Storage
    storage_root: Path = Field(
        default=Path("./data/jobs"),
        description="Root directory for job file storage"
    )
    max_upload_size_mb: int = Field(
        default=10,
        description="Maximum upload size in megabytes"
    )

    # Server
    server_host: str = Field(
        default="0.0.0.0",
        description="Server bind host"
    )
    server_port: int = Field(
        default=8000,
        description="Server bind port"
    )
    log_level: str = Field(
        default="INFO",
        description="Logging level"
    )

    # Object Types
    allowed_object_types: List[str] = Field(
        default=["gauge", "door", "fire_extinguisher", "unknown", "emergency_exit", "main_cylinder"],
        description="Allowed object types for processing"
    )

    # Job Processing
    max_queue_size: int = Field(
        default=1000,
        description="Maximum job queue size"
    )
    worker_poll_interval: float = Field(
        default=0.1,
        description="Worker polling interval in seconds"
    )

    # VLM Configuration
    vlm_provider: str = Field(
        default="stub",
        description="VLM provider: 'stub' (offline testing) or 'openai'"
    )
    vlm_api_key: str = Field(
        default="",
        description="API key for VLM provider (required for openai)"
    )
    vlm_model: str = Field(
        default="gpt-4o",
        description="VLM model name (e.g., gpt-4o, gpt-4-vision-preview)"
    )


    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    def get_job_dir(self, job_id: str) -> Path:
        """Get the directory path for a specific job."""
        return self.storage_root / job_id

    def get_roi_path(self, job_id: str, filename: str = "roi.jpg") -> Path:
        """Get the ROI image path for a specific job."""
        return self.get_job_dir(job_id) / filename


# Global settings instance
settings = Settings()
