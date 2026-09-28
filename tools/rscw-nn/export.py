"""Weights for trxd's forward pass (rsnn.rs), and a test vector.
export.py CKPT OUT.bin [TESTVEC.bin]

OUT.bin: b'RSN3', u32 hop, u32 2-D channels, u32 temporal channels, u32
layers, u32 dilation each, u32 count of f32 that follow, then the tensors in
order: c1.w c1.b c2.w c2.b c3.w c3.b att.w att.b inp.w inp.b t0.w t0.b ..
out.w out.b (the shipped network, C1D=32 DIL=1,2,4,8, also as b'RSN2':
u32 hop, u32 2-D channels, count, tensors).
(PyTorch layouts: conv2d (out, in, kt, kf), conv1d (out, in, k)).
TESTVEC.bin: u32 T, then T*61 features, then T logits (f32 LE)."""
import sys, struct, numpy as np, torch
import rsgen
from train import Net

net = Net(); net.load_state_dict(torch.load(sys.argv[1])); net.eval()
dils = [c.dilation[0] for c in net.t]
names = ['c1', 'c2', 'c3', 'att', 'inp'] + [f't.{i}' for i in range(len(dils))] + ['out']
sd = net.state_dict()
parts = []
for n in names:
    parts += [sd[n + '.weight'].numpy().ravel(), sd[n + '.bias'].numpy().ravel()]
flat = np.concatenate(parts).astype('<f4')
with open(sys.argv[2], 'wb') as f:
    c1d = net.inp.out_channels
    if c1d == 32 and dils == [1, 2, 4, 8]:
        f.write(b'RSN2' + struct.pack('<III', rsgen.HOP, net.c1.out_channels, len(flat)) + flat.tobytes())
    else:
        f.write(b'RSN3' + struct.pack('<IIII', rsgen.HOP, net.c1.out_channels, c1d, len(dils))
                + struct.pack(f'<{len(dils)}I', *dils) + struct.pack('<I', len(flat)) + flat.tobytes())
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
