#!/usr/bin/env bash
set -euo pipefail

image="kicad/kicad:9.0"
workdir="$PWD"

usage() {
  cat <<'EOF'
Usage: scripts/kicad-cli.sh [--workdir DIR] <kicad-cli arguments...>

Runs kicad-cli inside the pinned KiCad Docker image.

Options:
  --workdir DIR  Mount DIR at /work instead of the current directory
  -h, --help     Show this help text
EOF
}

while (($#)); do
  case "$1" in
    --workdir)
      shift
      if (($# == 0)); then
        echo "error: --workdir requires a directory" >&2
        exit 64
      fi
      workdir="$1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "error: unknown wrapper option: $1" >&2
      usage >&2
      exit 64
      ;;
    *)
      break
      ;;
  esac
done

if (($# == 0)); then
  usage >&2
  exit 64
fi

if [[ ! -d "$workdir" ]]; then
  echo "error: workdir does not exist: $workdir" >&2
  exit 66
fi

workdir="$(cd "$workdir" && pwd)"

exec docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$workdir:/work" \
  -w /work \
  "$image" \
  kicad-cli "$@"
