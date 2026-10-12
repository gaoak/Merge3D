#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! command -v NekMesh >/dev/null 2>&1 && type module >/dev/null 2>&1; then
  module load nektar++
fi
labels=(wall inlet outlet)
for id in 0 1 2; do
  name="pipe_${labels[id]}"
  NekMesh -m "extract:surf=${id}" pipecomp.xml "${name}.xml" -f
  FieldConvert "${name}.xml" "${name}.plt" -f
done
