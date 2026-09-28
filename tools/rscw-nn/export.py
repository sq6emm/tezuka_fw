"""Weights for trxd's forward pass (rsnn.rs), and a test vector.
export.py CKPT OUT.bin [TESTVEC.bin]

OUT.bin: b'RSN2', u32 hop, u32 2-D channels, u32 count of f32 that follow, then the tensors in order:
c1.w c1.b c2.w c2.b c3.w c3.b att.w att.b inp.w inp.b t0.w t0.b .. t3.w t3.b out.w out.b
(PyTorch layouts: conv2d (out, in, kt, kf), conv1d (out, in, k)).
TESTVEC.bin: u32 T, then T*61 features, then T logits (f32 LE)."""
import sys, struct, numpy as np, torch
import rsgen
from train import Net

net = Net(); net.load_state_dict(torch.load(sys.argv[1])); net.eval()
names = ['c1', 'c2', 'c3', 'att', 'inp', 't.0', 't.1', 't.2', 't.3', 'out']
sd = net.state_dict()
parts = []
for n in names:
    parts += [sd[n + '.weight'].numpy().ravel(), sd[n + '.bias'].numpy().ravel()]
flat = np.concatenate(parts).astype('<f4')
with open(sys.argv[2], 'wb') as f:
    f.write(b'RSN2' + struct.pack('<III', rsgen.HOP, net.c1.out_channels, len(flat)) + flat.tobytes())
print('weights', len(flat))
if len(sys.argv) > 3:
    x, _ = rsgen.example(np.random.default_rng(42), secs=3.0)
    feat = rsgen.features(x)
    with torch.no_grad():
        lg = net(torch.from_numpy(feat)[None])[0].numpy()
    with open(sys.argv[3], 'wb') as f:
        f.write(struct.pack('<I', len(feat)) + feat.astype('<f4').tobytes() + lg.astype('<f4').tobytes()
                + x.astype('<f4').tobytes())
    print('testvec T', len(feat), 'samples', len(x))
