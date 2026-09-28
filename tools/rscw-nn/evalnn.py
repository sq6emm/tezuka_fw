"""Per-frame LLRs of every real recording by a checkpoint: evalnn.py CKPT OUTDIR"""
import sys, os, numpy as np, torch, scipy.io.wavfile as wf
import rsgen
from train import Net
torch.set_num_threads(8)
ck, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
net = Net(); net.load_state_dict(torch.load(ck)); net.eval()
prior = float(os.environ.get('PRIOR', '0.45'))
wd = os.environ.get('WAVDIR', '/data/claude/rscw/wav')
for f in sorted(os.listdir(wd)):
    fs, x = wf.read(os.path.join(wd, f))
    assert fs == 12000, (f, fs)
    x = x.astype(np.float32) / 32768.0
    if x.ndim > 1: x = x.mean(1)
    feat = rsgen.features(x)
    with torch.no_grad():
        lg = net(torch.from_numpy(feat)[None])[0].numpy()
    llr = (lg - np.log(prior / (1 - prior))).astype(np.float32)
    llr.tofile(os.path.join(out, f + '.f32'))
print('done', ck)
