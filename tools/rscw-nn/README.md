# Rain-scatter CW keying detector: training

The network in trxd's `src/rsnn.rs` (weights `src/trxd/rsnn.bin`). See
docs/RSCW.md. CPU training (PyTorch) in Docker:

    docker build -t rscw-torch:1 .
    docker run --rm --shm-size=8g -e HOP=128 -e C2D=8 -v $PWD:/w -w /w rscw-torch:1 python train.py model 14000
    docker run --rm -e HOP=128 -e C2D=8 -v $PWD:/w -w /w rscw-torch:1 python export.py model.pt ../../src/trxd/rsnn.bin testvec.bin

then `RSNN_VEC=testvec.bin cargo test --release rsnn_matches_torch -- --ignored`
checks trxd's forward pass against PyTorch. evalnn.py writes per-frame LLRs
of a directory of recordings for trxd's `rscw_viterbi` / `rscw_oracle`
tests (RSCW_LLRDIR, RSCW_LLRHOP).

Environment: HOP (samples between frames, 64 or 128), C2D (2-D channels),
SNR_LO / SNR_HI (training SNR range, dB in 2.5 kHz), FSK (share of FSK
beacons, 0), NOISEMOD (share with a fluctuating noise level), INIT / LR
(fine-tuning), THREADS / WORKERS. avg.py averages checkpoints.

The shipped rsnn.bin: HOP=128 C2D=8; the average of a 14000-step run's
checkpoints at 2000 and 4000 steps (SNR -14..14) and of that run's 4000
fine-tuned 1000 steps (LR 5e-4, SNR -16..12, NOISEMOD 0.4).
