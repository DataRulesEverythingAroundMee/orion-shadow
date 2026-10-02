"""Orion Shadow - Simulation server and engine for Trillium Orion Gimbal systems."""

import logging
from typing import Optional

LOGGER_NAME = "orion_shadow"

def setup_logging(log_level: str = "warning", stream: Optional[object] = None) -> logging.Logger:
    """Configures and returns the top-level logger for orion_shadow."""
    logger = logging.getLogger(LOGGER_NAME)
    numeric_level = getattr(logging, log_level.upper(), logging.WARNING)
    logger.setLevel(numeric_level)

    if not logger.handlers:
        handler = logging.StreamHandler(stream)
        handler.setLevel(numeric_level)
        formatter = logging.Formatter(
            "%(asctime)s [%(levelname)-7s] [%(name)s] %(message)s",
            datefmt="%H:%M:%S"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    else:
        for handler in logger.handlers:
            handler.setLevel(numeric_level)

    logger.propagate = False
    return logger
