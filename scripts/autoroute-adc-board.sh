#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
board_dir="$repo_root/hardware/kicad/adc_board"
placed_pcb="hardware/kicad/adc_board/adc_board_8ch_unrouted.kicad_pcb"
dsn_file="hardware/kicad/adc_board/adc_board_8ch_unrouted.dsn"
ses_file="hardware/kicad/adc_board/adc_board_8ch_autorouted.ses"
final_pcb="hardware/kicad/adc_board/adc_board_8ch.kicad_pcb"
freerouting_data="hardware/kicad/adc_board/freerouting-data"

mkdir -p "$freerouting_data"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$repo_root:/work" \
  -w /work \
  kicad/kicad:9.0 \
  /bin/bash -lc \
  "python3 hardware/kicad/build_adc_board_layout.py --output $placed_pcb --dsn-output $dsn_file"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$repo_root:/work" \
  -w /work \
  ghcr.io/freerouting/freerouting:latest \
  java -jar /app/freerouting-executable.jar \
  -de "$dsn_file" \
  -do "$ses_file" \
  --gui.enabled=false \
  -mp 12 \
  --router.optimizer.improvement_threshold=5.0 \
  --user_data_path="$freerouting_data" \
  --logging.console.level=INFO \
  --logging.file.enabled=false

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$repo_root:/work" \
  -w /work \
  kicad/kicad:9.0 \
  /bin/bash -lc \
  "python3 - <<'PY'
import pcbnew

board = pcbnew.LoadBoard('$placed_pcb')
if not pcbnew.ImportSpecctraSES(board, '$ses_file'):
    raise SystemExit('failed to import Freerouting SES')
pcbnew.SaveBoard('$final_pcb', board)
print('Wrote $final_pcb')
PY"

./scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/adc_board/adc_board_8ch-drc.json \
  --exit-code-violations \
  hardware/kicad/adc_board/adc_board_8ch.kicad_pcb
