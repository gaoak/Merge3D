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
cmake -S . -B build
cmake --build build --parallel
bash InputMesh/extrude.sh
./build/mesh
mkdir -p verification
# Publish the compressed mesh only after the actual high-order check succeeds.
NekMesh -m jac:list pipe.xml verification/pipecomp.checked.xml -f \
  > verification/pipe_jacobian.log 2>&1
cat verification/pipe_jacobian.log
python3 checks/verify_pipe_mesh.py --jacobian-log verification/pipe_jacobian.log
mv verification/pipecomp.checked.xml pipecomp.xml
echo "Bent pipe: pipe.xml, pipecomp.xml; section stations: axis.csv"
