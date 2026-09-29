"""Transport-independent RoboCup fleet coordination (schema v1)."""
from .core import Coordinator
from .protocol import CoordinationError

__all__ = ['Coordinator', 'CoordinationError']
