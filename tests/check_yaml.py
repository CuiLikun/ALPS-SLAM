"""Optional YAML syntax check: python3 tests/check_yaml.py (requires PyYAML)."""
from pathlib import Path
import yaml


root = Path(__file__).resolve().parents[1]
paths = list((root / "alps_bringup/config").glob("*.yaml"))
paths += list((root / "alps_bringup/config").glob("*.rviz"))
paths += [root / "dependencies.repos", root / ".github/workflows/checks.yml"]
for path in paths:
    with path.open(encoding="utf-8") as stream:
        assert isinstance(yaml.safe_load(stream), dict), path
print("Parsed {} YAML configuration files".format(len(paths)))
