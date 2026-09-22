#!/usr/bin/env python3
"""
config/system.py — Real configuration loader.

Replaces the mock loader used during Phase 1 transition.
Parses config/system.yaml and returns the dictionary.
"""

import logging
import os
import yaml

logger = logging.getLogger(__name__)


def load_config(path: str = None) -> dict:
    """Load the system configuration from YAML."""
    if path is None:
        # Default to config/system.yaml relative to this file
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(base_dir, 'config', 'system.yaml')
        
    if not os.path.exists(path):
        logger.warning(f"Configuration file not found at {path}. Using empty config.")
        return {}
        
    try:
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
            return config if config else {}
    except Exception as e:
        logger.error(f"Failed to load configuration from {path}: {e}")
        return {}
