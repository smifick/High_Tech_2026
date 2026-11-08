"""Пути к ресурсам проекта, не зависящие от рабочей директории."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PROJECT_ROOT / "scene" / "field.xml"
