#!/usr/bin/env bash
# Generate the mavpilot API documentation from docstrings with pdoc.
#
# Usage:
#   scripts/build_docs.sh            # build into docs/api/
#   OUT=site scripts/build_docs.sh   # build into a custom directory
#
# Requires the dev extra: pip install -e ".[dev]"  (or: pip install pdoc)
set -euo pipefail

OUT="${OUT:-docs/api}"

cd "$(dirname "$0")/.."

echo "Building API docs into ${OUT}/ ..."
python -m pdoc \
    --docformat google \
    --logo-link "https://github.com/Onikore/mavpilot" \
    -o "${OUT}" \
    mavpilot

echo "Done. Open ${OUT}/index.html"
