#!/bin/sh
# Pack rade.wasm.gz into the `model` partition blob (src/trxd/src/rade.rs):
#   "RAD1" | u32 LE length | SHA-256 of the gzip stream | gzip stream
#   pack-rade.sh <rade.wasm.gz> <model.bin>
set -eu
in=$1 out=$2
len=$(wc -c < "$in")
le32() { printf "\\$(printf %03o $(($1 & 255)))\\$(printf %03o $(($1 >> 8 & 255)))\\$(printf %03o $(($1 >> 16 & 255)))\\$(printf %03o $(($1 >> 24 & 255)))"; }
{ printf RAD1; le32 "$len"; sha256sum < "$in" | cut -c1-64 | xxd -r -p; cat "$in"; } > "$out"
[ "$(wc -c < "$out")" -eq $((len + 40)) ] || { echo "pack-rade.sh: bad blob size" >&2; exit 1; }
