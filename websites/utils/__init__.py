"""Shared configuration, styling and database helpers for the ERS dashboard.

This package replaces the former single ``utils.py`` god-module. Pages still use
``import utils as ers`` and call ``ers.<name>``; every public name is re-exported
here from a focused submodule:

    config     - paths, constants, autorefresh()
    styles     - ERS_CSS and HTML snippet builders
    db         - connection + staff/devices schema
    incidents  - incident queries, counts, retention purge
    staff      - staff lookup and CRUD
    devices    - device lookup and CRUD
    battery    - client battery readings
    system     - status, control flags, admin recipients, server commands
    mapping    - map scale, gateways, room zones (localisation)
    heartbeat  - device online/offline status
"""

from .config import *      # noqa: F401,F403
from .styles import *      # noqa: F401,F403
from .db import *          # noqa: F401,F403
from .incidents import *   # noqa: F401,F403
from .staff import *       # noqa: F401,F403
from .devices import *     # noqa: F401,F403
from .battery import *     # noqa: F401,F403
from .system import *      # noqa: F401,F403
from .mapping import *     # noqa: F401,F403
from .heartbeat import *   # noqa: F401,F403

from . import config, styles, db, incidents, staff, devices, battery, system, mapping, heartbeat

__all__ = [
    *config.__all__,
    *styles.__all__,
    *db.__all__,
    *incidents.__all__,
    *staff.__all__,
    *devices.__all__,
    *battery.__all__,
    *system.__all__,
    *mapping.__all__,
    *heartbeat.__all__,
]
