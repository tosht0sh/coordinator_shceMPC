"""Public package exports for motion planning.

This module keeps the old laptop simulation import style working:

    from pkg_motion_plan import GlobalPathCoordinator, LocalTrajPlanner

while avoiding eager imports of laptop-only modules during bot startup.
The bot runtime should still prefer direct submodule imports.
"""

from importlib import import_module

__all__ = ["GlobalPathCoordinator", "LocalTrajPlanner"]


def __getattr__(name):
    if name == "GlobalPathCoordinator":
        return import_module(".global_path_coordinate", __name__).GlobalPathCoordinator
    if name == "LocalTrajPlanner":
        return import_module(".local_traj_plan", __name__).LocalTrajPlanner
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
