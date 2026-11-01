"""Create logger and write to file + console."""
from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional


def configure_logging(
    log_dir: Path,
    level: str = "INFO",
    console: bool = True,
    name: str = "adas",
) -> logging.Logger:
    """Configure root + named logger with file + console handlers."""
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger(name)
    if getattr(logger, "_adas_configured", False):
        return logger
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler = RotatingFileHandler(
        log_dir / "adas.log", maxBytes=2_000_000, backupCount=3, encoding="utf-8"
    )
    file_handler.setFormatter(fmt)
    logger.addHandler(file_handler)
    if console:
        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setFormatter(fmt)
        logger.addHandler(stream_handler)
    logger.propagate = False
    logger._adas_configured = True  # type: ignore[attr-defined]
    return logger


def get_logger(name: Optional[str] = None) -> logging.Logger:
    return logging.getLogger(name or "adas")


__all__ = ["configure_logging", "get_logger"]
