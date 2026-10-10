"""相位调度：给每个逻辑门分配级数（= 被哪个时钟相位驱动）。

约定
- 主输入 (PI) 在第 0 级产生（balance_pis=True 时；否则 PI 可以在任意相位到达）。
- 第 L 级的单元读取第 L-1 级产生的信号，在第 L 级产生输出。
- 主输出 (PO) 全部在同一级 D 取出（balance_pos=True 时）。

调度只决定逻辑门的级数；buffer/splitter 由 insert.py 根据级数差补出来。
可选算法：
- asap  ：每个门尽量早。原版想做的就是这个，但它用 DFS"首次访问即定级"取到的不是最长路径。
- alap  ：在不增加总深度 D 的前提下每个门尽量晚。离输出近、离输入远的门受益。
- lp    ：把"buffer 数"近似成线性代价后解线性规划（差分约束 ⇒ 全幺模 ⇒ 解天然是整数）。
- best  ：上面几种都做，再用精确代价做局部搜索，取 buffer+splitter 最少者。（默认）
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from . import trees
from .netlist import CONSTS, Netlist, Sink


@dataclass
class Config:
    max_fanout: Optional[int] = None    # 限制 splitter 最大扇出（None = 库里能用的最大）
    balance_pis: bool = True            # 主输入是否都在第 0 相位到达
    balance_pos: bool = True            # 主输出是否对齐到同一相位
    branch_pis: bool = False            # 主输入能否不经 splitter 直接驱动多个负载
    method: str = "best"                # asap / alap / lp / best
    refine: bool = True                 # 用精确代价做局部搜索
    depth: Optional[int] = None         # 指定输出相位 D（None = 最小可行深度）


@dataclass
class Schedule:
    levels: Dict[str, int]                      # 逻辑门 -> 级
    pi_levels: Dict[str, Optional[int]]         # 主输入 -> 级（None = 自由）
    depth: int                                  # 主输出相位 D
    method: str = ""
    cost: int = -1                              # 精确的 buffer+splitter 数
    notes: List[str] = field(default_factory=list)


class _Graph:
    """调度用的精简图：只关心逻辑门、网线的驱动与负载。"""

    def __init__(self, nl: Netlist, cfg: Config):
        self.nl, self.cfg = nl, cfg
        self.k = nl.lib.max_splitter_fanout(cfg.max_fanout)
        self.order = nl.topo_order()
        self.drv = nl.drivers()
        self.snk = nl.sinks()
        self.pis = nl.pis
        # 只保留"有负载且不是常量"的网线
        self.nets = [n for n in self.snk if n not in CONSTS]
        self.net_of_out: Dict[str, List[str]] = {}       # 门 -> 它驱动的网线
        for n in self.nets:
            d = self.drv.get(n)
            if d is None:
                raise ValueError(f"网线 {n} 没有驱动")
            if d[0] is not None:
                self.net_of_out.setdefault(d[0], []).append(n)
        self.in_nets: Dict[str, List[str]] = {g: [] for g in self.order}
        for n in self.nets:
            for s in self.snk[n]:
                if s.inst is not None:
                    self.in_nets[s.inst].append(n)

    def is_pi_net(self, n: str) -> bool:
        return self.drv[n][0] is None

    def sd(self, n: str) -> int:
        """保守的 splitter 深度：负载统一推迟这么多级一定可行。"""
        if self.is_pi_net(n) and self.cfg.branch_pis:
            return 0
        return trees.splitter_depth(len(self.snk[n]), self.k)

    def root_cap(self, n: str) -> float:
        return trees.INF if (self.is_pi_net(n) and self.cfg.branch_pis) else 1

    # ---------------------------------------------------------------- 精确代价
    def reqs(self, n: str, levels: Dict[str, int], depth: int) -> List[int]:
        out = []
        for s in self.snk[n]:
            if s.inst is None:
                out.append(depth if self.cfg.balance_pos else None)  # type: ignore[arg-type]
            else:
                out.append(levels[s.inst] - 1)
        if not self.cfg.balance_pos and None in out:
            # 不对齐输出时，主输出就在"树能最早给出拷贝"的地方取
            real = [r for r in out if r is not None]
            root = self.root_level(n, levels)
            base = root if root is not None else (min(real) if real else 0)
            out = [r if r is not None else base + self.sd(n) for r in out]
        return out

    def root_level(self, n: str, levels: Dict[str, int]) -> Optional[int]:
        d = self.drv[n][0]
        if d is None:
            return 0 if self.cfg.balance_pis else None
        return levels[d]

    def net_cost(self, n: str, levels: Dict[str, int], depth: int) -> int:
        return trees.cost(self.reqs(n, levels, depth), self.k, self.root_level(n, levels), self.root_cap(n))

    def total_cost(self, levels: Dict[str, int], depth: int) -> int:
        tot = 0
        for n in self.nets:
            c = self.net_cost(n, levels, depth)
            if c < 0:
                return -1
            tot += c
        return tot


# ---------------------------------------------------------------------- 算法
def _asap(g: _Graph) -> Tuple[Dict[str, int], int]:
    lv: Dict[str, int] = {}
    for gate in g.order:
        L = 1
        for n in g.in_nets[gate]:
            d = g.drv[n][0]
            base = lv[d] if d is not None else 0
            L = max(L, base + 1 + g.sd(n))
        lv[gate] = L
    D = 0
    for po in g.nl.pos:
        n = g.nl.po_drivers.get(po, po)
        if n in CONSTS:
            continue
        d = g.drv[n][0]
        D = max(D, (lv[d] if d is not None else 0) + g.sd(n))
    return lv, D


def _alap(g: _Graph, D: int, asap: Dict[str, int]) -> Dict[str, int]:
    lv: Dict[str, int] = {}
    for gate in reversed(g.order):
        L = None
        for n in g.net_of_out.get(gate, []):
            for s in g.snk[n]:
                if s.inst is None:
                    lim = D - g.sd(n) if g.cfg.balance_pos else asap[gate]
                else:
                    lim = lv[s.inst] - 1 - g.sd(n)
                L = lim if L is None else min(L, lim)
        lv[gate] = asap[gate] if L is None else max(L, asap[gate])
    return lv


def _lp(g: _Graph, D: int) -> Optional[Dict[str, int]]:
    """线性规划：min Σ_net (M_net - L_driver)，M_net ≥ 每个负载要求的相位。
    约束全是 x_i - x_j ≤ c 的差分形式，系数矩阵全幺模，所以单纯形给出的顶点解就是整数解。
    代价是"每根网线上最长那条 buffer 链的长度"，扇出 >1 时它是真实单元数的下界（近似）。"""
    try:
        import numpy as np
        from scipy.optimize import linprog
        from scipy.sparse import coo_matrix
    except ImportError:
        return None
    gates = g.order
    gi = {x: i for i, x in enumerate(gates)}
    nvar = len(gates)
    mi: Dict[str, int] = {}
    for n in g.nets:
        mi[n] = nvar
        nvar += 1
    pfree: Dict[str, int] = {}
    if not g.cfg.balance_pis:
        for pi in g.pis:
            if pi in g.snk:
                pfree[pi] = nvar
                nvar += 1
    rows, cols, vals, b = [], [], [], []
    obj = np.zeros(nvar)

    def var_of_driver(n: str):
        d = g.drv[n][0]
        if d is not None:
            return gi[d], 0.0
        if n in pfree:
            return pfree[n], 0.0
        return None, 0.0  # 固定在 0 级的主输入

    r = 0
    for n in g.nets:
        dv, _ = var_of_driver(n)
        sd = g.sd(n)
        obj[mi[n]] += 1
        if dv is not None:
            obj[dv] -= 1
        for s in g.snk[n]:
            if s.inst is None:
                if not g.cfg.balance_pos:
                    continue
                # L_d ≤ D - sd ; D - M ≤ 0
                if dv is not None:
                    rows += [r]; cols += [dv]; vals += [1.0]; b.append(D - sd); r += 1
                rows += [r]; cols += [mi[n]]; vals += [-1.0]; b.append(-D); r += 1
            else:
                si = gi[s.inst]
                # L_d - L_s ≤ -(1+sd)
                if dv is not None:
                    rows += [r, r]; cols += [dv, si]; vals += [1.0, -1.0]
                else:
                    rows += [r]; cols += [si]; vals += [-1.0]
                b.append(-(1 + sd)); r += 1
                # L_s - 1 - M ≤ 0
                rows += [r, r]; cols += [si, mi[n]]; vals += [1.0, -1.0]; b.append(1.0); r += 1
    A = coo_matrix((vals, (rows, cols)), shape=(r, nvar)).tocsr()
    bounds = [(1, D) for _ in gates] + [(None, None)] * (nvar - len(gates))
    res = linprog(obj, A_ub=A, b_ub=np.array(b), bounds=bounds, method="highs")
    if not res.success:
        return None
    x = res.x
    lv = {gate: int(round(x[gi[gate]])) for gate in gates}
    if max(abs(x[gi[gate]] - lv[gate]) for gate in gates) > 1e-6:
        return None  # 理论上不会发生（全幺模）
    return lv


def _refine(g: _Graph, lv: Dict[str, int], D: int, max_passes: int = 20) -> Dict[str, int]:
    """局部搜索：逐个门尝试上移/下移一级，用精确树代价判断是否更好。
    精确代价允许突破调度里"统一推迟 ceil(log_k n) 级"的保守约束。"""
    lv = dict(lv)
    cost_of: Dict[str, int] = {n: g.net_cost(n, lv, D) for n in g.nets}
    touched: Dict[str, List[str]] = {gate: g.in_nets[gate] + g.net_of_out.get(gate, []) for gate in g.order}
    for _ in range(max_passes):
        improved = False
        for gate in g.order:
            nets = touched[gate]
            old = sum(cost_of[n] for n in nets)
            best_delta, best_L, best_costs = 0, None, None
            for dL in (-1, 1):
                L = lv[gate] + dL
                if L < 1 or L > D:
                    continue
                lv[gate] = L
                costs = {n: g.net_cost(n, lv, D) for n in nets}
                lv[gate] = L - dL
                if any(c < 0 for c in costs.values()):
                    continue
                delta = sum(costs.values()) - old
                if delta < best_delta:
                    best_delta, best_L, best_costs = delta, L, costs
            if best_L is not None:
                lv[gate] = best_L
                cost_of.update(best_costs)  # type: ignore[arg-type]
                improved = True
        if not improved:
            break
    return lv


def schedule(nl: Netlist, cfg: Optional[Config] = None) -> Schedule:
    cfg = cfg or Config()
    g = _Graph(nl, cfg)
    asap, D_min = _asap(g)
    D = D_min if cfg.depth is None else cfg.depth
    notes = []
    if D < D_min:
        raise ValueError(f"指定深度 {D} 小于最小可行深度 {D_min}")
    if D > D_min:
        notes.append(f"输出相位被指定为 {D}（最小可行为 {D_min}）")

    cands: List[Tuple[str, Dict[str, int]]] = []
    methods = [cfg.method] if cfg.method != "best" else ["asap", "alap", "lp"]
    for m in methods:
        if m == "asap":
            cands.append(("asap", asap))
        elif m == "alap":
            cands.append(("alap", _alap(g, D, asap)))
        elif m == "lp":
            lv = _lp(g, D)
            if lv is None:
                notes.append("LP 不可用（未装 scipy 或求解失败），跳过")
            else:
                cands.append(("lp", lv))
        else:
            raise ValueError(f"未知调度方法 {m}")
    if cfg.refine:
        cands += [(f"{m}+refine", _refine(g, lv, D)) for m, lv in list(cands)]

    best = None
    tried = []
    for m, lv in cands:
        c = g.total_cost(lv, D)
        tried.append((m, c))
        if c >= 0 and (best is None or c < best[2]):
            best = (m, lv, c)
    if best is None:
        raise RuntimeError(f"所有调度都不可行：{tried}（这是 bug，请报告）")
    notes.append("候选：" + "，".join(f"{m}={c}" for m, c in tried))
    m, lv, c = best
    pi_levels: Dict[str, Optional[int]] = {pi: (0 if cfg.balance_pis else None) for pi in nl.pis}
    return Schedule(lv, pi_levels, D, m, c, notes)


def graph_for(nl: Netlist, cfg: Config) -> _Graph:
    return _Graph(nl, cfg)
