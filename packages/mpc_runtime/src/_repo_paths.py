"""Repo-relative path helpers for the transitional MPC split.

These utilities let the new bot runtime package import the existing TrajPlan code
without copying the whole source tree into `packages/mpc_runtime` yet.
"""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
TRAJPLAN_ROOT = REPO_ROOT / "assets" / "TrajPlan-ScheMPC-copy"
TRAJPLAN_SRC = TRAJPLAN_ROOT / "src"
TRAJPLAN_CONFIG = TRAJPLAN_ROOT / "config"
MPC_RUNTIME_SRC = REPO_ROOT / "packages" / "mpc_runtime" / "src"


def ensure_trajplan_imports() -> None:
    for path in (TRAJPLAN_SRC, MPC_RUNTIME_SRC):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


def ensure_mpc_runtime_imports() -> None:
    path_str = str(MPC_RUNTIME_SRC)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)
