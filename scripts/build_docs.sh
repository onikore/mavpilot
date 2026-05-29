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

# `mavpilot` exposes the public API via __all__ (DroneController + data types),
# which keeps the internal mavpilot.core.* modules out of the rendered site.
# mavpilot.utils is public (per the README) but not re-exported, so include it
# explicitly to give the coordinate helpers their own page.
echo "Building API docs into ${OUT}/ ..."
python -m pdoc \
    --docformat google \
    --logo-link "https://github.com/Onikore/mavpilot" \
    -o "${OUT}" \
    mavpilot mavpilot.utils

echo "Done. Open ${OUT}/index.html"
