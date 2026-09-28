//! T2 frame building (gr-dtv dvbt2_framemapper_cc) and frequency
//! interleaving (dvbt2_freqinterleaver_cc), 2K SISO.

use super::{l1, Cell, Params, BPSK0, C_P2, L1_PRE_CELLS, N_P2, ZERO};

/// Frame builder: L1-pre and L1-post spread over the P2 symbols, then the
/// data cells, dummy cells (BPSK from a PRBS) and unmodulated cells up to
/// the frame closing symbol's active count.
pub struct FrameMapper {
    p: Params,
    pre: Vec<Cell>,
    /// L1-post for each frame index of the super-frame (nothing else in
    /// it changes).
    post: Vec<Vec<Cell>>,
    dummy: Vec<Cell>,
    frame_idx: usize,
}

impl FrameMapper {
    pub fn new(p: Params) -> FrameMapper {
        let (_, n_fc, c_fc) = p.data_cells();
        let stream = p.cells() * p.fec_blocks;
        let n_dummy = p
            .frame_cells()
            .checked_sub(stream + L1_PRE_CELLS + l1::post_cells() + (n_fc - c_fc))
            .expect("too many FEC blocks for the T2 frame");
        let mut sr: u32 = 0x4A80;
        let dummy = (0..n_dummy)
            .map(|_| {
                let b = (sr ^ (sr >> 1)) & 1;
                sr >>= 1;
                if b != 0 {
                    sr |= 0x4000;
                }
                BPSK0 + b as Cell
            })
            .collect();
        let post = (0..p.t2_frames).map(|i| l1::post(&p, i)).collect();
        FrameMapper { p, pre: l1::pre(&p), post, dummy, frame_idx: 0 }
    }

    /// One T2 frame's cells from its interleaved data cells.
    pub fn frame(&mut self, data: &[Cell]) -> Vec<Cell> {
        let p = &self.p;
        let post = &self.post[self.frame_idx];
        self.frame_idx = (self.frame_idx + 1) % p.t2_frames;
        // L1-pre and L1-post each spread over the P2 symbols (every N_P2-th
        // cell to a symbol), then data and dummy cells into the P2 symbols'
        // rest and the data symbols in order; unmodulated cells last.
        let mut out = vec![ZERO; p.frame_cells()];
        let npost = post.len();
        let (pre_per, post_per) = (L1_PRE_CELLS / N_P2, npost / N_P2);
        for n in 0..N_P2 {
            for j in 0..pre_per {
                out[n * C_P2 + j] = self.pre[n + j * N_P2];
            }
            for j in 0..post_per {
                out[n * C_P2 + pre_per + j] = post[n + j * N_P2];
            }
        }
        let mut src: [&[Cell]; 2] = [data, &self.dummy];
        let mut put = |mut d: &mut [Cell]| {
            for s in src.iter_mut() {
                let n = d.len().min(s.len());
                d[..n].copy_from_slice(&s[..n]);
                *s = &s[n..];
                d = &mut std::mem::take(&mut d)[n..];
            }
        };
        for n in 0..N_P2 {
            put(&mut out[n * C_P2 + pre_per + post_per..(n + 1) * C_P2]);
        }
        put(&mut out[N_P2 * C_P2..]);
        debug_assert!(src.iter().all(|s| s.is_empty()));
        out
    }

    #[cfg(test)]
    pub fn frame_ref(&mut self, data: &[Cell]) -> Vec<Cell> {
        let p = &self.p;
        let (_, n_fc, c_fc) = p.data_cells();
        let post = &self.post[self.frame_idx];
        self.frame_idx = (self.frame_idx + 1) % p.t2_frames;
        let mut z = Vec::with_capacity(p.frame_cells());
        z.extend_from_slice(&self.pre);
        z.extend_from_slice(post);
        z.extend_from_slice(data);
        z.extend_from_slice(&self.dummy);
        z.extend(std::iter::repeat_n(ZERO, n_fc - c_fc));
        assert_eq!(z.len(), p.frame_cells());
        // L1-pre and L1-post each spread over the P2 symbols (every N_P2-th
        // cell to a symbol), then the P2 symbols' rest and the data symbols
        // in order.
        let mut out = vec![ZERO; z.len()];
        let npost = post.len();
        let (pre_per, post_per) = (L1_PRE_CELLS / N_P2, npost / N_P2);
        for n in 0..N_P2 {
            for j in 0..pre_per {
                out[n * C_P2 + j] = z[n + j * N_P2];
            }
            for j in 0..post_per {
                out[n * C_P2 + pre_per + j] = z[L1_PRE_CELLS + n + j * N_P2];
            }
        }
        let mut read = L1_PRE_CELLS + npost;
        for n in 0..N_P2 {
            for j in pre_per + post_per..C_P2 {
                out[n * C_P2 + j] = z[read];
                read += 1;
            }
        }
        let mut index = N_P2 * C_P2;
        while read < z.len() {
            out[index] = z[read];
            index += 1;
            read += 1;
        }
        out
    }
}

impl FrameMapper {
    /// [`Self::frame`]'s layout for any cell type: L1-pre and L1-post each
    /// spread over the P2 symbols, then data, dummy and `zero` cells (for
    /// FastFrame's labels).
    pub fn layout<T: Copy>(&self, pre: &[T], post: &[T], data: &[T], dummy: &[T], zero: T) -> Vec<T> {
        let mut z = Vec::with_capacity(self.p.frame_cells());
        z.extend_from_slice(pre);
        z.extend_from_slice(post);
        z.extend_from_slice(data);
        z.extend_from_slice(dummy);
        z.resize(self.p.frame_cells(), zero);
        let mut out = vec![zero; z.len()];
        let npost = post.len();
        let (pre_per, post_per) = (L1_PRE_CELLS / N_P2, npost / N_P2);
        for n in 0..N_P2 {
            for j in 0..pre_per {
                out[n * C_P2 + j] = z[n + j * N_P2];
            }
            for j in 0..post_per {
                out[n * C_P2 + pre_per + j] = z[L1_PRE_CELLS + n + j * N_P2];
            }
        }
        let mut read = L1_PRE_CELLS + npost;
        let mut rest = (0..N_P2).flat_map(|n| n * C_P2 + pre_per + post_per..(n + 1) * C_P2).chain(N_P2 * C_P2..z.len());
        while read < z.len() {
            out[rest.next().unwrap()] = z[read];
            read += 1;
        }
        out
    }

    /// The dummy cells (after the data).
    pub fn dummy_cells(&self) -> &[Cell] {
        &self.dummy
    }

    /// The reverse (receive): a frame's cells to (L1-pre, L1-post, data).
    pub fn unmap<T: Copy>(&self, out: &[T]) -> (Vec<T>, Vec<T>, Vec<T>) {
        let npost = l1::post_cells();
        let (pre_per, post_per) = (L1_PRE_CELLS / N_P2, npost / N_P2);
        let mut pre = vec![out[0]; L1_PRE_CELLS];
        let mut post = vec![out[0]; npost];
        for n in 0..N_P2 {
            for j in 0..pre_per {
                pre[n + j * N_P2] = out[n * C_P2 + j];
            }
            for j in 0..post_per {
                post[n + j * N_P2] = out[n * C_P2 + pre_per + j];
            }
        }
        let mut data = Vec::with_capacity(out.len());
        for n in 0..N_P2 {
            data.extend_from_slice(&out[n * C_P2 + pre_per + post_per..(n + 1) * C_P2]);
        }
        data.extend_from_slice(&out[N_P2 * C_P2..]);
        data.truncate(self.p.cells() * self.p.fec_blocks);
        (pre, post, data)
    }

    /// The L1-pre cells as sent.
    pub fn pre_cells(&self) -> &[Cell] {
        &self.pre
    }
}

/// Frequency interleaver permutations (even and odd symbols) for `cells`
/// active cells, 2K.
fn freq_permutation(cells: usize) -> [Vec<usize>; 2] {
    const PN_DEGREE: u32 = 10;
    const MAX_STATES: u32 = 2048;
    const EVEN: [u32; 10] = [4, 3, 9, 6, 2, 8, 1, 5, 7, 0];
    const ODD: [u32; 10] = [6, 9, 4, 8, 5, 1, 0, 7, 2, 3];
    let (mut he, mut ho) = (Vec::with_capacity(cells), Vec::with_capacity(cells));
    let mut lfsr: u32 = 0;
    for i in 0..MAX_STATES {
        if i == 0 || i == 1 {
            lfsr = 0;
        } else if i == 2 {
            lfsr = 1;
        } else {
            let r = (lfsr ^ (lfsr >> 3)) & 1;
            lfsr &= 0x3ff;
            lfsr >>= 1;
            lfsr |= r << (PN_DEGREE - 1);
        }
        let (mut e, mut o) = (0u32, 0u32);
        for n in 0..PN_DEGREE as usize {
            e |= ((lfsr >> n) & 1) << EVEN[n];
            o |= ((lfsr >> n) & 1) << ODD[n];
        }
        e += (i % 2) * (MAX_STATES / 2);
        o += (i % 2) * (MAX_STATES / 2);
        if (e as usize) < cells {
            he.push(e as usize);
        }
        if (o as usize) < cells {
            ho.push(o as usize);
        }
    }
    [he, ho]
}

pub struct FreqInterleaver {
    p2: [Vec<usize>; 2],
    data: [Vec<usize>; 2],
    fc: Option<[Vec<usize>; 2]>,
    data_symbols: usize,
}

impl FreqInterleaver {
    pub fn new(p: &Params) -> FreqInterleaver {
        let (c_data, n_fc, _) = p.data_cells();
        FreqInterleaver {
            p2: freq_permutation(C_P2),
            data: freq_permutation(c_data),
            fc: (n_fc != 0).then(|| freq_permutation(n_fc)),
            data_symbols: if n_fc != 0 { p.data_symbols - 1 } else { p.data_symbols },
        }
    }

    /// The reverse (receive): symbols' cells back to the frame's order.
    pub fn unframe<T: Copy>(&self, syms: &[Vec<T>]) -> Vec<T> {
        let mut out = Vec::new();
        for (symbol, s) in syms.iter().enumerate() {
            let h: &[usize] = if symbol < N_P2 {
                &self.p2[symbol % 2]
            } else if self.fc.is_some() && symbol == syms.len() - 1 {
                &self.fc.as_ref().unwrap()[symbol % 2]
            } else {
                &self.data[symbol % 2]
            };
            let mut cells = vec![s[0]; h.len()];
            for (j, &k) in h.iter().enumerate() {
                cells[k] = s[j];
            }
            out.extend(cells);
        }
        out
    }

    /// One frame's cells, symbol by symbol: out[j] = in[H[j]].
    pub fn frame<T: Copy>(&self, cells: &[T]) -> Vec<Vec<T>> {
        let mut syms = Vec::new();
        let mut at = 0;
        let mut symbol = 0;
        let one = |h: &[usize], at: &mut usize, symbol: &mut usize| {
            let s: Vec<T> = h.iter().map(|&k| cells[*at + k]).collect();
            *at += h.len();
            *symbol += 1;
            s
        };
        for _ in 0..N_P2 {
            let h = &self.p2[symbol % 2];
            syms.push(one(h, &mut at, &mut symbol));
        }
        for _ in 0..self.data_symbols {
            let h = &self.data[symbol % 2];
            syms.push(one(h, &mut at, &mut symbol));
        }
        if let Some(fc) = &self.fc {
            let h = &fc[symbol % 2];
            syms.push(one(h, &mut at, &mut symbol));
        }
        assert_eq!(at, cells.len());
        syms
    }
}
