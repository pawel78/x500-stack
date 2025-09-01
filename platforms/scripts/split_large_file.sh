#!/usr/bin/env bash
# Usage: ./split_large_file.sh <file> [chunk_size_MB]
# Example: ./split_large_file.sh fmu_logs/2022-05-15-06-00-09.bag 1900

set -euo pipefail

FILE="$1"
SIZE_MB="${2:-1900}"   # default 1900 MB
BASENAME="$(basename "$FILE")"
DIR="$(dirname "$FILE")"
PREFIX="$DIR/$BASENAME.part-"

echo "Splitting $FILE into ${SIZE_MB}MB chunks..."
split -b "${SIZE_MB}m" -a 3 -d "$FILE" "$PREFIX"

echo "Done. Created:"
ls -lh "$PREFIX"*

