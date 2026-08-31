"""Backward-compatibility import path for the WildBridge Safety Computer client.

The real implementation lives in wildbridge_groundstation.safety. This module only
exists so historical import paths (`from djiInterfaceSafety import DJIInterfaceSafety`)
keep working; new code should import from wildbridge_groundstation.safety.
"""

from wildbridge_groundstation.safety import (
    EP_RELEASE_SAFETY_CONTROL,
    SAFETY_TOKEN,
    SAFETY_TOKEN_HEADER,
    DJIInterfaceSafety,
)

__all__ = [
    "EP_RELEASE_SAFETY_CONTROL",
    "SAFETY_TOKEN",
    "SAFETY_TOKEN_HEADER",
    "DJIInterfaceSafety",
]
