import json
from pathlib import Path
CONFIG=json.loads((Path(__file__).resolve().parents[1]/'config/workstation.json').read_text())
PERCEPTION=json.loads((Path(__file__).resolve().parents[1]/'config/perception.json').read_text())
