"""Сохранение результатов сканирования."""
import json
from pathlib import Path

def save_grid(cells, output_file):
    path = Path(output_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cells, ensure_ascii=False, indent=4), encoding="utf-8")
