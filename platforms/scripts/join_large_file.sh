#!/usr/bin/env bash
# Usage: ./join_large_file.sh <first_part_file> [output_file]
# Example: ./join_large_file.sh fmu_logs/2022-05-15-06-00-09.bag.part-000

set -euo pipefail

FIRST="$1"
OUT="${2:-${FIRST%.part-*}}"

DIR="$(dirname "$FIRST")"
BASENAME="$(basename "$FIRST")"
PREFIX="${BASENAME%.part-*}"

echo "Reassembling parts starting with $FIRST into $OUT ..."
cat "$DIR/$PREFIX".part-* > "$OUT"

echo "Done. Output written to: $OUT"
ls -lh "$OUT"

