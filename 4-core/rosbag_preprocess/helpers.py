#!/usr/bin/env python3
###############################################################################################################
"""
artificial-agent
11-07-2024
"""
"""
helpers.py
"""
###############################################################################################################


###############################################################################################################
# Core Imports
from typing import Union, Dict
###############################################################################################################


###############################################################################################################
def format_value(value: Union[int, float], precision: int) -> Union[str, int, float]:
    """Formats a value according to the specified precision.

    Args:
        value: The value to format, which can be an integer or float.
        precision: The number of decimal places to format the value to.

    Returns:
        Formatted value as a string if precision is specified; otherwise, returns the original value.
    """
    if isinstance(value, float):
        return f"{value:.{precision}f}"
    return value  # Return as-is if it's an integer or no formatting is needed


###############################################################################################################

# EOF