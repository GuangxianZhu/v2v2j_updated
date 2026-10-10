"""两种仿真。

1. comb_eval：零延迟组合求值，用于功能等价性检查（插 buffer/splitter 不应改变逻辑功能）。
2. phase_sim：相位精确的"波形流水线"仿真，用来回答：插完以后，电路能不能每个时钟周期吃一组新输入？

phase_sim 的物理模型（教学用的离散化，不是电路级仿真）：
- 时间以"相位步"为单位，4 相时钟一个周期 = 4 步。
- 第 L 级单元在 t ≡ L (mod 4) 的步被激励，读取输入、算出输出，并在 t、t+1 两步保持输出；
  t+2、t+3 步它没有励磁，输出电流为 0（对下游来说是 X）。
- 因此第 L 级单元在 t 时刻读来源 S（第 Ls 级）时：
    (L - Ls) mod 4 == 1  → 读到 S 在 t-1 步算出的值。若 L - Ls == 1 正好是同一波数据；
                            若 L - Ls == 5、9…，读到的是**更新的**一波数据（短路径"抢跑"）→ 静默错误
    其余情况            → S 此刻没有输出或正在同时切换 → X
- 主输入在第 0 级：第 k 组输入在 t = 4k 步出现、保持两步。
这就是为什么相位不平衡要么得到 X，要么（差 4 的倍数时）静默地混入别的数据——后者更危险。
"""
from __future__ import annotations

import random
from typing import Dict, List, Optional, Sequence

from .netlist import CONST0, CONST1, Netlist


def comb_eval(nl: Netlist, vec: Dict[str, int], order: Optional[List[str]] = None) -> Dict[str, int]:
    lib = nl.lib
    val: Dict[str, int] = {CONST0: 0, CONST1: 1}
    val.update(vec)
    for name in order or nl.topo_order():
        inst = nl.insts[name]
        cd = lib[inst.cell]
        res = cd.evaluate({p: val[inst.conns[p]] for p in cd.inputs})
        for o, b in res.items():
            if o in inst.conns:
                val[inst.conns[o]] = b
    return {po: val[nl.po_drivers.get(po, po)] for po in nl.pos}


def vectors(pis: Sequence[str], max_exhaustive: int = 14, n_random: int = 2048, seed: int = 1):
    if len(pis) <= max_exhaustive:
        for i in range(1 << len(pis)):
            yield {p: (i >> j) & 1 for j, p in enumerate(pis)}
    else:
        rng = random.Random(seed)
        for _ in range(n_random):
            yield {p: rng.getrandbits(1) for p in pis}


def equivalent(a: Netlist, b: Netlist, **kw) -> Optional[Dict[str, int]]:
    """返回 None 表示（在所测向量上）等价，否则返回一个反例输入。"""
    if sorted(a.pis) != sorted(b.pis) or sorted(a.pos) != sorted(b.pos):
        raise ValueError("两个网表的端口不一致")
    oa, ob = a.topo_order(), b.topo_order()
    for v in vectors(a.pis, **kw):
        if comb_eval(a, v, oa) != comb_eval(b, v, ob):
            return v
    return None


def phase_sim(nl: Netlist, stream: List[Dict[str, int]], levels: Optional[Dict[str, int]] = None,
              po_level: Optional[int] = None, phases: int = 4):
    """相位精确仿真。返回 (每波的输出列表, 输出相位 D)。输出值为 0/1/None(X)。

    levels：单元 -> 级；缺省取 inst.level（插入结果自带）。
    输出端口的读取时刻：第 k 波在 t = 4k + D + 1 步读取（假设输出端口后面接一个第 D+1 级的负载）。
    """
    lib = nl.lib
    lv = {n: (levels[n] if levels else nl.insts[n].level) for n in nl.insts}
    if any(v is None for v in lv.values()):
        raise ValueError("phase_sim 需要每个单元都有级数")
    D = po_level if po_level is not None else max(lv.values(), default=0)
    order = sorted(nl.insts, key=lambda n: lv[n])
    drv = nl.drivers()
    src_level: Dict[str, int] = {}
    for net, (inst, _pin) in drv.items():
        src_level[net] = lv[inst] if inst is not None else 0
    # 每根网线保存 (最近一次被激励的步, 值)
    hist: Dict[str, Dict[int, Optional[int]]] = {net: {} for net in drv}
    T = phases * (len(stream) + 1) + D + 2
    results: List[Dict[str, Optional[int]]] = []

    def read(net: str, t: int, reader_level: int) -> Optional[int]:
        if net == CONST0:
            return 0
        if net == CONST1:
            return 1
        Ls = src_level[net]
        if (reader_level - Ls) % phases != 1:
            return None
        return hist[net].get(t - 1)

    for t in range(T):
        # 主输入：第 0 级，t ≡ 0 (mod 4) 时换下一组
        if t % phases == 0:
            k = t // phases
            vec = stream[k] if k < len(stream) else None
            for pi in nl.pis:
                hist[pi][t] = vec[pi] if vec else None
        for pi in nl.pis:  # 保持一步
            if (t - 1) in hist[pi] and t % phases == 1:
                hist[pi][t] = hist[pi][t - 1]
        for name in order:
            L = lv[name]
            inst = nl.insts[name]
            cd = lib[inst.cell]
            outs = cd.outputs
            if t % phases == L % phases:
                ins = {p: read(inst.conns[p], t, L) for p in cd.inputs}
                res = cd.evaluate3(ins)
                for o in outs:
                    if o in inst.conns:
                        hist[inst.conns[o]][t] = res[o]
            elif t % phases == (L + 1) % phases:
                for o in outs:
                    net = inst.conns.get(o)
                    if net is not None and (t - 1) in hist[net]:
                        hist[net][t] = hist[net][t - 1]
    for k in range(len(stream)):
        t = phases * k + D + 1
        row = {}
        for po in nl.pos:
            row[po] = read(nl.po_drivers.get(po, po), t, D + 1)
        results.append(row)
    return results, D


def stream_check(nl: Netlist, golden: Netlist, n: int = 64, seed: int = 7, levels=None, po_level=None):
    """连续灌入 n 组随机输入，对比相位仿真结果与零延迟参考。返回 (错误数, X 数, 总位数)。"""
    rng = random.Random(seed)
    stream = [{p: rng.getrandbits(1) for p in nl.pis} for _ in range(n)]
    got, _ = phase_sim(nl, stream, levels=levels, po_level=po_level)
    order = golden.topo_order()
    wrong = xs = total = 0
    for vec, row in zip(stream, got):
        ref = comb_eval(golden, vec, order)
        for po, v in row.items():
            total += 1
            if v is None:
                xs += 1
            elif v != ref[po]:
                wrong += 1
    return wrong, xs, total
