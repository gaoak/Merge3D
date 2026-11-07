#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if ! command -v NekMesh >/dev/null 2>&1 && type module >/dev/null 2>&1; then
  module load nektar++
fi
command -v NekMesh >/dev/null 2>&1 || {
  echo "NekMesh is required; load the Nektar++ environment first." >&2
  exit 1
}

# Re-extrude the current 2D input on every run to avoid using a stale mesh.
bash InputMesh/extrude.sh
cmake -S . -B build
cmake --build build --target mesh
./build/mesh "$@" --yes --lshape --output lshape_3d.xml
NekMesh -m jac:list lshape_3d.xml lshape_3dcomp.xml -f
echo "L-shape mesh: lshape_3d.xml, lshape_3dcomp.xml"
