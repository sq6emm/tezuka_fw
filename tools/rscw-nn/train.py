"""Train the keying detector on synthetic rain scatter (rsgen.py).
python train.py OUT_PREFIX STEPS"""
import os, sys, time, math, numpy as np, torch, torch.nn as nn, torch.nn.functional as F
import rsgen

torch.set_num_threads(int(os.environ.get('THREADS', '24')))


class Net(nn.Module):
    # C1D: temporal channels; DIL: the temporal convolutions' dilations
    # (context: 4 x sum of them frames either side). The shipped network:
    # C2D=8, C1D=32, DIL=1,2,4,8 (about 23 k weights, for the A9).
    def __init__(self, c2d=int(os.environ.get('C2D', '12')), c1d=int(os.environ.get('C1D', '32')),
                 dil=tuple(int(x) for x in os.environ.get('DIL', '1,2,4,8').split(','))):
        super().__init__()
        self.c1 = nn.Conv2d(1, c2d, (3, 5), padding=(1, 2))
        self.c2 = nn.Conv2d(c2d, c2d, (3, 5), padding=(1, 2), stride=(1, 2))
        self.c3 = nn.Conv2d(c2d, c2d, (3, 3), padding=(1, 1))
        self.att = nn.Conv2d(c2d, 1, 1)
        self.inp = nn.Conv1d(2 * c2d, c1d, 1)
        self.t = nn.ModuleList([nn.Conv1d(c1d, c1d, 5, padding=2 * d, dilation=d) for d in dil])
        self.out = nn.Conv1d(c1d, 1, 1)

    def forward(self, x):                       # x: (B, T, F)
        h = F.relu(self.c1(x.unsqueeze(1)))      # (B, C, T, F): kernels (time, freq)
        h = F.relu(self.c2(h))
        h = F.relu(self.c3(h))
        a = torch.softmax(self.att(h), dim=3)    # attention over frequency
        pooled = torch.cat([(h * a).sum(3), h.amax(3)], 1)   # (B, 2C, T)
        z = F.relu(self.inp(pooled))
        for c in self.t:
            z = z + F.relu(c(z))
        return self.out(z).squeeze(1)            # (B, T) logits


class Synth(torch.utils.data.IterableDataset):
    def __iter__(self):
        wi = torch.utils.data.get_worker_info()
        r = np.random.default_rng((wi.id if wi else 0) * 7919 + int(time.time() * 1e3) % 100000)
        while True:
            x, k = rsgen.example(r)
            f = rsgen.features(x)
            y = rsgen.labels(k, len(f))
            yield torch.from_numpy(f), torch.from_numpy(y.astype(np.float32))


def main():
    out, steps = sys.argv[1], int(sys.argv[2])
    net = Net()
    import os
    if os.environ.get('INIT'):
        net.load_state_dict(torch.load(os.environ['INIT']))
        print('from', os.environ['INIT'])
    print('params', sum(p.numel() for p in net.parameters()), flush=True)
    dl = torch.utils.data.DataLoader(Synth(), batch_size=32, num_workers=int(os.environ.get('WORKERS', '16')), prefetch_factor=4)
    opt = torch.optim.AdamW(net.parameters(), lr=2e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=float(os.environ.get('LR', '2e-3')), total_steps=steps, pct_start=0.05)
    t0, run = time.time(), None
    for step, (x, y) in enumerate(dl):
        if step >= steps:
            break
        logit = net(x)
        loss = F.binary_cross_entropy_with_logits(logit, y)
        opt.zero_grad()
        loss.backward()
        opt.step()
        sched.step()
        run = loss.item() if run is None else 0.98 * run + 0.02 * loss.item()
        if step % 200 == 0:
            acc = ((logit > 0).float() == y).float().mean().item()
            print(f'step {step} loss {run:.4f} acc {acc:.3f} {time.time() - t0:.0f}s', flush=True)
        if step % 2000 == 0 or step == steps - 1:
            torch.save(net.state_dict(), out + '.pt')
    torch.save(net.state_dict(), out + '.pt')


if __name__ == '__main__':
    main()
