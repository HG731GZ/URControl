from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Union

import pinocchio as pin


PathLike = Union[str, Path]


@lru_cache(maxsize=8)
def _build_model_from_mjcf_cached(mjcf_path: str):
    return pin.buildModelFromMJCF(mjcf_path)


@lru_cache(maxsize=8)
def _build_models_from_mjcf_cached(mjcf_path: str):
    return pin.shortcuts.buildModelsFromMJCF(mjcf_path)


def build_model_from_mjcf(mjcf_path: PathLike):
    """Build only the Pinocchio kinematic model once per process."""
    path = str(Path(mjcf_path).resolve())
    return _build_model_from_mjcf_cached(path).copy()


def build_models_from_mjcf(mjcf_path: PathLike):
    """Build Pinocchio MJCF model and geometry once per process."""
    path = str(Path(mjcf_path).resolve())
    model, constraint_models, collision_model, visual_model = (
        _build_models_from_mjcf_cached(path)
    )
    return (
        model.copy(),
        constraint_models.copy(),
        collision_model.clone(),
        visual_model.clone(),
    )


def clear_model_cache() -> None:
    _build_model_from_mjcf_cached.cache_clear()
    _build_models_from_mjcf_cached.cache_clear()
