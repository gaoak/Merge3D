#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if ! command -v NekMesh >/dev/null 2>&1 && type module >/dev/null 2>&1; then
  module load nektar++
fi
layers="${1:-$(./build/mesh --layers)}"
if [[ ! "$layers" =~ ^[0-9]+$ ]] || (( layers < 2 )); then
  echo "Extrusion layer count must be an integer >= 2." >&2
  exit 1
fi
[[ -s InputMesh/inFoil.xml ]] || { echo "Missing InputMesh/inFoil.xml" >&2; exit 1; }
# Use the user's current circle mesh. Always re-extrude to avoid stale inputs.
NekMesh -m "extrude:layers=${layers}:length=1" InputMesh/inFoil.xml \
  InputMesh/pipeStraight.xml:xml:uncompress -f
