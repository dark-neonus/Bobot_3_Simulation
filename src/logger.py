from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

import pandas as pd


@dataclass
class SimulationLogger:
    rows: List[Dict[str, Any]] = field(default_factory=list)

    def log(self, row: Dict[str, Any]) -> None:
        self.rows.append(dict(row))

    def to_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame(self.rows)

    def save_csv(self, file_path: str | Path) -> Path:
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        df = self.to_dataframe()
        df.to_csv(path, index=False)
        return path
