#!/bin/sh
# Regenerate package/trxd/deepcw-model.bin (the DeepCW model for the `model`
# flash partition) from sdroxide's model.onnx: weights rounded to bfloat16
# precision, xz -9e, DCW1 header with the SHA-256 of the result
# (src/trxd/src/model.rs). DeepCW's WPM x SNR accuracy matrix is unchanged.
#
#   tools/pack-deepcw-model.sh [path/to/sdroxide/crates/sdroxide-deepcw/assets/model.onnx]
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
IN="${1:-$HERE/../sdroxide-vhf/crates/sdroxide-deepcw/assets/model.onnx}"
(cd "$HERE/src/trxd" && cargo run --release --quiet -- --pack-model "$IN" "$HERE/package/trxd/deepcw-model.bin")
