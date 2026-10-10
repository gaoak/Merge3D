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
# Rectangle is the default, and this script explicitly selects it as well.
./build/mesh "$@" --rectangular --output test.xml
NekMesh test.xml testcomp.xml -f
echo "Rectangular mesh: test.xml, testcomp.xml"
