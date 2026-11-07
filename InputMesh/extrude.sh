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

# Use unit extrusion for the layer-index mapping in ../mesh.cpp.
NekMesh -m extrude:layers=32:length=1 \
  lshape_mesh.xml lshape_3d_uniform.xml:xml:uncompress -f
