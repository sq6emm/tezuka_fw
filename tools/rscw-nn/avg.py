"""Average checkpoints' weights: avg.py OUT.pt IN1.pt IN2.pt ..."""
import sys, torch
sds = [torch.load(p) for p in sys.argv[2:]]
out = {k: sum(sd[k] for sd in sds) / len(sds) for k in sds[0]}
torch.save(out, sys.argv[1])
print('averaged', len(sds))
