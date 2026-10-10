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

for input in 3DoutUcomp.xml 3DoutUcomp2.xml 3DinUcomp.xml; do
  if [[ ! -f InputMesh/$input ]]; then
    (cd InputMesh && bash extrude.sh)
    break
  fi
done
cmake -S . -B build
cmake --build build
# Deformation dimensions are configured separately in paramsDeformed.h.
./build/mesh "$@" --deformed --output deformed.xml
NekMesh -m jac:list deformed.xml deformedcomp.xml -f
echo "Deformed mesh: deformed.xml, deformedcomp.xml"
