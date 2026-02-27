"""
Constants for OpenVLA Client

This module imports shared constants and defines OpenVLA client-specific configuration.
"""

from typing import Final
import sys
from pathlib import Path

# Add parent directory to path for shared_constants import
sys.path.insert(0, str(Path(__file__).parent.parent))

# Import all shared constants
from shared_constants import *  # noqa: F401, F403

# ============================================================================
# OpenVLA Client Specific Configuration
# ============================================================================

IK_SOLVER_TOLERANCE: Final[float] = 1e-6
"""Tolerance for IK solver convergence (relaxed for faster performance)"""
