"""单根网线上的 buffer/splitter 树：可行性判定与最少单元数。

问题：一根网线的驱动在第 L0 相位产生信号；它有若干负载，第 i 个负载要求在第 r_i 相位拿到一份拷贝
（门在第 L 级 ⇒ 它要的拷贝产生于 L-1；主输出要求拷贝产生于输出相位 D）。
每级可以放 buffer（1 进 1 出）或 k 路 splitter（1 进 ≤k 出），每个单元占 1 级、1 份输入拷贝。
求：能不能做到？最少要几个单元？

贪心（从最深的负载往驱动方向倒推）：
    c(t) = 第 t 级需要存在的拷贝数 = 本级直接被负载拿走的 s(t) + 下一级单元的输入数 m(t+1)
    第 t 级至少要 g(t) = ceil(c(t)/k) 个单元（每个最多产 k 份），于是 m(t) = g(t)。
    一直推到驱动所在的 L0：可行 ⇔ c(L0) ≤ 1（驱动只有一个输出）。
每一级都取了下界 ceil(c/k)，而 c(t) 越小上一层只会越小，所以这个贪心对"固定负载相位"是最优的。
这也解释了原版"先分裂再各自补 buffer"为什么浪费：它让 c(t) 在很多层都大于 1。

可行条件的另一个等价写法是 Kraft 不等式 Σ k^-(r_i-L0) ≤ 1（负载都在 L0 之后时），
所以"扇出 n 的网线，所有负载统一推迟 ceil(log_k n) 级"只是充分条件——调度里用它做保守约束，
精确判定留给这里。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

INF = float("inf")


def splitter_depth(n: int, k: int) -> int:
    """做出 n 份拷贝至少需要几级（扇出 1 时为 0）。"""
    d, cap = 0, 1
    while cap < n:
        cap *= k
        d += 1
    return d


def plan(reqs: List[int], k: int, root: Optional[int], root_cap: float = 1
         ) -> Tuple[Optional[int], Dict[int, int], int]:
    """贪心规划一棵树。

    reqs: 每个负载要求的拷贝相位 r_i
    root: 驱动相位；None 表示驱动相位可自由选择（如"不要求主输入对齐"时的主输入）
    root_cap: 驱动本身能直接给出几份拷贝（通常 1；允许主输入无限分支时为 INF）
    返回 (root, c, cost)：c[t] 为第 t 级的拷贝数，cost 为单元数；不可行时 cost = -1。
    """
    if not reqs:
        return root, {}, 0
    lo, hi = min(reqs), max(reqs)
    if root is not None and lo < root:
        return root, {}, -1
    s: Dict[int, int] = {}
    for r in reqs:
        s[r] = s.get(r, 0) + 1
    c: Dict[int, int] = {}
    m = 0
    t = hi
    cost = 0
    while True:
        c[t] = s.get(t, 0) + m
        if root is not None and t == root:
            return (root, c, cost) if c[t] <= root_cap else (root, c, -1)
        if root is None and t <= lo and c[t] <= root_cap:
            return t, c, cost
        g = math.ceil(c[t] / k)
        cost += g
        m = g
        t -= 1
        if root is None and t < lo - 64:  # 防御：不应发生
            return None, c, -1


def cost(reqs: List[int], k: int, root: Optional[int], root_cap: float = 1) -> int:
    return plan(reqs, k, root, root_cap)[2]


def split_sizes(copies: int, k: int) -> List[int]:
    """把 copies 份拷贝尽量均匀地分给 ceil(copies/k) 个单元（如 5 份、k=4 → [3, 2]）。"""
    g = math.ceil(copies / k)
    base, rem = divmod(copies, g)
    return [base + 1] * rem + [base] * (g - rem)
