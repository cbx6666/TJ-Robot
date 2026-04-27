import os
from pathlib import Path

from robot_navigation.utils.map_save_utils import resolve_map_basename, resolve_map_save_directory


def test_resolve_map_basename_fallback():
    base = resolve_map_basename("")
    assert base.startswith("coverage_map_")


def test_resolve_map_basename_strips_path():
    base = resolve_map_basename("/tmp/a/b/demo")
    assert base == "demo"


def test_resolve_map_save_directory_explicit(tmp_path: Path):
    out = resolve_map_save_directory(str(tmp_path / "maps"))
    assert out.exists()
    assert out.name == "maps"


def test_resolve_map_save_directory_from_env(tmp_path: Path):
    env_path = tmp_path / "from_env"
    old = os.environ.get("TJ_ROBOT_SAVED_MAPS_DIR")
    os.environ["TJ_ROBOT_SAVED_MAPS_DIR"] = str(env_path)
    try:
        out = resolve_map_save_directory("")
    finally:
        if old is None:
            os.environ.pop("TJ_ROBOT_SAVED_MAPS_DIR", None)
        else:
            os.environ["TJ_ROBOT_SAVED_MAPS_DIR"] = old
    assert out == env_path.resolve()
