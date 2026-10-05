#!/bin/sh
# Build rade.wasm (RADE V2 for the SQTRX page) in Docker: rade_c's V2
# transmitter/receiver, opus's LPCNet features and FARGAN, int8 weights only.
#
#   src/rade-web/build.sh [out-dir]       (default: src/rade-web/out)
#
# Sources are fetched into src/rade-web/cache (git-ignored) at pinned revs.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$(mkdir -p "${1:-$HERE/out}" && cd "${1:-$HERE/out}" && pwd)
RADE_C_REV=cc17222acc597339199bdfd7253c3e6cf147f953
OPUS_REV=940d4e5af64351ca8ba8390df3f555484c567fbb
IMAGE=rade-web:1

docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -q -t "$IMAGE" --label owner=claude "$HERE"
mkdir -p "$HERE/cache"
docker run --rm --name tezuka-rade-web --label owner=claude --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -v "$HERE:/w" -v "$OUT:/out" -w /w/cache "$IMAGE" sh -euc "
RADE_C_REV=$RADE_C_REV
OPUS_REV=$OPUS_REV
if [ ! -d rade_c ]; then git clone -q https://github.com/freedv/rade_c.git; fi
(cd rade_c && git cat-file -e \$RADE_C_REV 2>/dev/null || git fetch -q; git checkout -q \$RADE_C_REV)
if [ ! -f opus-\$OPUS_REV/.built ]; then
    rm -rf opus-\$OPUS_REV
    curl -sSL -o opus.zip https://github.com/xiph/opus/archive/\$OPUS_REV.zip
    unzip -q opus.zip && rm opus.zip
    cd opus-\$OPUS_REV
    patch -s dnn/nnet.h < ../rade_c/src/opus-nnet.h.diff
    patch -s dnn/nnet.c < ../rade_c/src/opus-nnet.c.diff
    ./autogen.sh >/dev/null 2>&1
    emconfigure ./configure --host=wasm32-unknown-emscripten --disable-shared --disable-doc \
        --disable-extra-programs --enable-osce --enable-dred --disable-rtcd --disable-intrinsics \
        CFLAGS='-O3 -msimd128 -msse4.1 -DDISABLE_DEBUG_FLOAT' >/dev/null
    emmake make -j\$(nproc) >/dev/null
    touch .built
    cd ..
fi
O=\$PWD/opus-\$OPUS_REV
S=\$PWD/rade_c/src
emcc -O3 -msimd128 -msse4.1 -DDISABLE_DEBUG_FLOAT -DHAVE_CONFIG_H \
    -I\$O -I\$O/include -I\$O/dnn -I\$O/celt -I\$O/silk -I\$S \
    /w/rade_web.c \$S/rade_tx_v2.c \$S/rade_rx_v2.c \$S/rade_v2_ofdm.c \
    \$S/rade_enc_v2.c \$S/rade_dec_v2.c \$S/rade_sync.c \
    \$S/rade_enc_v2_data.c \$S/rade_dec_v2_data.c \$S/rade_sync_data.c \
    \$S/rade_bpf.c \$S/rade_dsp.c \
    \$O/.libs/libopus.a \
    --no-entry -sSTANDALONE_WASM -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=33554432 \
    -sFILESYSTEM=0 -o /out/rade.wasm
"
gzip -9nc "$OUT/rade.wasm" > "$OUT/rade.wasm.gz"
ls -l "$OUT/rade.wasm" "$OUT/rade.wasm.gz"
