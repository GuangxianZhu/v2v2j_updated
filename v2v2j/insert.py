"""按调度结果插入 buffer / splitter，生成满足 AQFP 约束的新网表。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from . import trees
from .netlist import CONSTS, Inst, Netlist, Sink, remove_buffers_and_splitters, sanitize
from .schedule import Config, Schedule, graph_for, schedule


@dataclass
class Result:
    netlist: Netlist
    schedule: Schedule
    warnings: List[str] = field(default_factory=list)
    stats: Dict[str, int] = field(default_factory=dict)


def remove_dangling(nl: Netlist) -> Tuple[Netlist, List[str]]:
    """反复删除输出没人用的逻辑门（AQFP 门的输出必须有负载，悬空门只是白耗面积）。"""
    nl = nl.copy()
    removed: List[str] = []
    while True:
        snk = nl.sinks()
        dead = []
        for inst in nl.insts.values():
            outs = [inst.conns.get(o) for o in nl.lib[inst.cell].outputs]
            if all(o is None or o not in snk for o in outs):
                dead.append(inst.name)
        if not dead:
            return nl, removed
        for d in dead:
            del nl.insts[d]
        removed += dead


def apply_schedule(nl: Netlist, sch: Schedule, cfg: Config) -> Tuple[Netlist, List[str]]:
    lib = nl.lib
    g = graph_for(nl, cfg)
    k = g.k
    warnings: List[str] = []
    out = nl.copy()
    out.po_drivers = {}
    for name, inst in out.insts.items():
        inst.level = sch.levels[name]
        inst.origin = "logic"

    used = set(nl.all_nets()) | set(nl.insts)
    po_set = set(nl.pos)
    producer: Dict[str, Tuple[str, str]] = {}  # 新网线 -> (实例, 输出脚)

    # 1) 逻辑门的输出网线如果恰好是某个主输出的名字，先改成内部名，
    #    主输出名留给"真正送到输出端口的那份拷贝"
    rename_root: Dict[str, str] = {}
    for n in g.nets:
        d, pin = g.drv[n]
        if d is None:
            continue
        if n in po_set:
            new = out.fresh_name(f"{sanitize(n)}__int", used)
            out.insts[d].conns[pin] = new
            rename_root[n] = new
            producer[new] = (d, pin)
        else:
            producer[n] = (d, pin)

    const_sinks = 0
    for n in nl.sinks():
        if n in CONSTS:
            for s in nl.sinks()[n]:
                const_sinks += 1
                if s.inst is None:
                    out.po_drivers[s.pin] = n
    if const_sinks:
        warnings.append(f"有 {const_sinks} 个负载接常量 {CONSTS}；常量不参与相位平衡，"
                        "实际 AQFP 中需要用偏置实现，请确认单元库支持")

    n_buf = n_spl = 0
    for n in g.nets:
        sinks: List[Sink] = g.snk[n]
        reqs = g.reqs(n, sch.levels, sch.depth)
        root = g.root_level(n, sch.levels)
        root, c, cost = trees.plan(reqs, k, root, g.root_cap(n))
        if cost < 0:
            raise RuntimeError(f"网线 {n} 的树不可行（调度 bug）：负载相位 {reqs}，驱动相位 {root}")
        root_net = rename_root.get(n, n)
        base = sanitize(n)
        # 按相位分组负载
        by_r: Dict[int, List[Sink]] = {}
        for s, r in zip(sinks, reqs):
            by_r.setdefault(r, []).append(s)
        hi = max(reqs)
        copies: List[str] = [root_net] * int(min(c[root], 10**9))  # 允许主输入无限分支时 c[root] 可 >1
        assign: List[Tuple[Sink, str]] = []
        for t in range(root, hi + 1):
            here = by_r.get(t, [])
            for s in here:
                assign.append((s, copies.pop(0)))
            if t == hi:
                break
            need = c[t + 1]
            feeders = copies
            sizes = trees.split_sizes(need, k)
            assert len(sizes) == len(feeders), (n, t, sizes, feeders)
            new_copies: List[str] = []
            for j, (src, size) in enumerate(zip(feeders, sizes)):
                cell = lib.splitter(size)
                cd = lib[cell]
                kind = "buffer" if size == 1 else "splitter"
                iname = out.fresh_name(f"{'bfr' if size == 1 else 'spl'}_{base}_L{t + 1}" +
                                       (f"_{j}" if len(sizes) > 1 else ""), used)
                conns = {cd.inputs[0]: src}
                for o in cd.outputs[:size]:
                    net = out.fresh_name(f"{base}__L{t + 1}" + (f"_{len(new_copies)}"), used)
                    conns[o] = net
                    producer[net] = (iname, o)
                    new_copies.append(net)
                out.add(Inst(iname, cell, conns, t + 1, kind, {"net": n}))
                if size == 1:
                    n_buf += 1
                else:
                    n_spl += 1
            copies = new_copies
        # 接上负载
        for s, net in assign:
            if s.inst is not None:
                out.insts[s.inst].conns[s.pin] = net
            else:
                po = s.pin
                if net in producer and net not in po_set:
                    inst, pin = producer.pop(net)
                    out.insts[inst].conns[pin] = po
                    producer[po] = (inst, pin)
                    # 若这份拷贝恰好是逻辑门的根网线，则更新其它引用（不会有：根网线扇出为 1）
                elif net != po:
                    out.po_drivers[po] = net  # 主输入/常量直通到输出：assign po = pi

    # 统计
    stats = {
        "logic": sum(1 for i in out.insts.values() if i.origin == "logic"),
        "buffers": n_buf,
        "splitters": n_spl,
        "depth": sch.depth,
        "jj_cells_total": len(out.insts),
    }
    out.stats = stats  # type: ignore[attr-defined]
    return out, warnings


def insert(nl: Netlist, cfg: Optional[Config] = None) -> Result:
    """完整流程：去掉已有 buffer/splitter → 删除悬空门 → 调度 → 建树。"""
    cfg = cfg or Config()
    warnings: List[str] = []
    nl, removed = remove_buffers_and_splitters(nl)
    if removed:
        warnings.append(f"输入网表里已有 {removed} 个 buffer/splitter，已当作导线去掉后重新插入")
    nl, dead = remove_dangling(nl)
    if dead:
        warnings.append(f"删除了 {len(dead)} 个输出无负载的门：{dead[:8]}{'…' if len(dead) > 8 else ''}")
    sch = schedule(nl, cfg)
    out, w = apply_schedule(nl, sch, cfg)
    warnings += w
    return Result(out, sch, warnings, out.stats)  # type: ignore[attr-defined]
