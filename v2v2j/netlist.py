"""网表数据模型。

模型是"位级"的：每根网线是 1 bit，总线 a[3:0] 被拆成 a[3]..a[0] 四根网线，
端口的位宽信息另外保存在 Port 里，写回 Verilog 时再还原成总线。

术语：
- 驱动 (driver)：给网线提供信号的东西——主输入 (PI)、常量，或某个单元的输出脚；
- 负载 (sink)：读网线的东西——某个单元的输入脚，或主输出 (PO)。
AQFP 的约束就是对 driver/sink 关系的约束，所以这里把它们算得很直接。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

from .library import Library

CONST0 = "1'b0"
CONST1 = "1'b1"
CONSTS = (CONST0, CONST1)


@dataclass
class Port:
    name: str
    direction: str            # "input" / "output"
    bits: List[str]           # 位级网线名，从 LSB 到 MSB
    msb: int = 0
    lsb: int = 0

    @property
    def width(self) -> int:
        return len(self.bits)


@dataclass
class Inst:
    name: str
    cell: str
    conns: Dict[str, str]                  # 端口 -> 网线（只含逻辑端口）
    level: Optional[int] = None
    origin: str = "logic"                  # logic / buffer / splitter（由插入算法生成的标为后两者）
    attrs: Dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class Sink:
    inst: Optional[str]                    # None 表示主输出
    pin: str                               # 单元端口名；主输出时为端口位名


class NetlistError(Exception):
    pass


class Netlist:
    def __init__(self, name: str, lib: Library):
        self.name = name
        self.lib = lib
        self.ports: List[Port] = []
        self.insts: Dict[str, Inst] = {}
        # 输出端口位 -> 驱动它的网线（通常同名；assign po = x 时不同名）
        self.po_drivers: Dict[str, str] = {}

    # ------------------------------------------------------------------ 端口
    @property
    def pis(self) -> List[str]:
        return [b for p in self.ports if p.direction == "input" for b in p.bits]

    @property
    def pos(self) -> List[str]:
        return [b for p in self.ports if p.direction == "output" for b in p.bits]

    # ------------------------------------------------------------------ 拓扑查询
    def drivers(self) -> Dict[str, Tuple[Optional[str], str]]:
        """网线 -> (单元名或 None, 端口名)。None 表示主输入或常量。"""
        drv: Dict[str, Tuple[Optional[str], str]] = {}
        for pi in self.pis:
            drv[pi] = (None, "PI")
        for c in CONSTS:
            drv[c] = (None, "CONST")
        for inst in self.insts.values():
            cd = self.lib[inst.cell]
            for pin in cd.outputs:
                net = inst.conns.get(pin)
                if net is None:
                    continue
                if net in drv:
                    raise NetlistError(f"网线 {net} 有多个驱动：{drv[net]} 和 {inst.name}.{pin}")
                drv[net] = (inst.name, pin)
        return drv

    def sinks(self) -> Dict[str, List[Sink]]:
        snk: Dict[str, List[Sink]] = {}
        for inst in self.insts.values():
            cd = self.lib[inst.cell]
            for pin in cd.inputs:
                net = inst.conns.get(pin)
                if net is None:
                    raise NetlistError(f"{inst.name}.{pin} 未连接")
                snk.setdefault(net, []).append(Sink(inst.name, pin))
        for po in self.pos:
            snk.setdefault(self.po_drivers.get(po, po), []).append(Sink(None, po))
        return snk

    def topo_order(self) -> List[str]:
        """单元的拓扑序；有组合环时报错并给出环上的单元。"""
        drv = self.drivers()
        indeg: Dict[str, int] = {}
        fanout: Dict[str, List[str]] = {n: [] for n in self.insts}
        for inst in self.insts.values():
            cd = self.lib[inst.cell]
            preds = set()
            for pin in cd.inputs:
                d = drv.get(inst.conns[pin])
                if d is None:
                    raise NetlistError(f"{inst.name}.{pin} 接的网线 {inst.conns[pin]} 没有驱动")
                if d[0] is not None:
                    preds.add(d[0])
            indeg[inst.name] = len(preds)
            for p in preds:
                fanout[p].append(inst.name)
        order = [n for n, k in indeg.items() if k == 0]
        i = 0
        while i < len(order):
            for s in fanout[order[i]]:
                indeg[s] -= 1
                if indeg[s] == 0:
                    order.append(s)
            i += 1
        if len(order) != len(self.insts):
            stuck = sorted(n for n, k in indeg.items() if k > 0)
            raise NetlistError(f"网表中存在组合环（AQFP 插入只处理无环电路），涉及：{stuck[:10]}")
        return order

    # ------------------------------------------------------------------ 统计
    def count_by_origin(self) -> Dict[str, int]:
        out: Dict[str, int] = {}
        for inst in self.insts.values():
            out[inst.origin] = out.get(inst.origin, 0) + 1
        return out

    def count_by_role(self) -> Dict[str, int]:
        out: Dict[str, int] = {}
        for inst in self.insts.values():
            r = self.lib[inst.cell].role
            out[r] = out.get(r, 0) + 1
        return out

    def depth(self) -> int:
        levels = [i.level for i in self.insts.values() if i.level is not None]
        return max(levels) if levels else 0

    def copy(self) -> "Netlist":
        n = Netlist(self.name, self.lib)
        n.ports = [Port(p.name, p.direction, list(p.bits), p.msb, p.lsb) for p in self.ports]
        n.insts = {k: Inst(v.name, v.cell, dict(v.conns), v.level, v.origin, dict(v.attrs))
                   for k, v in self.insts.items()}
        n.po_drivers = dict(self.po_drivers)
        return n

    def add(self, inst: Inst) -> None:
        if inst.name in self.insts:
            raise NetlistError(f"实例名重复：{inst.name}")
        self.insts[inst.name] = inst

    def all_nets(self) -> List[str]:
        nets = set(self.pis) | set(self.pos)
        for inst in self.insts.values():
            nets.update(inst.conns.values())
        return sorted(nets)

    def fresh_name(self, base: str, used: set) -> str:
        if base not in used:
            used.add(base)
            return base
        i = 1
        while f"{base}_{i}" in used:
            i += 1
        used.add(f"{base}_{i}")
        return f"{base}_{i}"


def sanitize(name: str) -> str:
    """把 a[3]、\\foo.bar 之类变成可做新标识符前缀的 a_3_ / foo_bar。"""
    out = []
    for ch in name.lstrip("\\"):
        out.append(ch if (ch.isalnum() or ch == "_") else "_")
    s = "".join(out).strip()
    if not s or s[0].isdigit():
        s = "n" + s
    return s


def remove_buffers_and_splitters(nl: Netlist) -> Tuple[Netlist, int]:
    """把网表里已有的 buffer/splitter 当作导线去掉（用于对已插过的网表重新插入）。"""
    lib = nl.lib
    alias: Dict[str, str] = {}
    keep: Dict[str, Inst] = {}
    removed = 0
    for inst in nl.insts.values():
        role = lib[inst.cell].role
        if role in ("buffer", "splitter"):
            src = inst.conns[lib[inst.cell].inputs[0]]
            for o in lib[inst.cell].outputs:
                if o in inst.conns:
                    alias[inst.conns[o]] = src
            removed += 1
        else:
            keep[inst.name] = inst

    def root(n: str) -> str:
        seen = set()
        while n in alias and n not in seen:
            seen.add(n)
            n = alias[n]
        return n

    out = nl.copy()
    out.insts = {}
    for name, inst in keep.items():
        conns = {}
        for pin, net in inst.conns.items():
            conns[pin] = root(net) if pin in lib[inst.cell].inputs else net
        out.insts[name] = Inst(inst.name, inst.cell, conns, None, "logic", dict(inst.attrs))
    for po in out.pos:
        d = root(out.po_drivers.get(po, po))
        if d != po:
            out.po_drivers[po] = d
        else:
            out.po_drivers.pop(po, None)
    return out, removed


def iter_logic_inputs(nl: Netlist, inst: Inst) -> Iterable[Tuple[str, str]]:
    for pin in nl.lib[inst.cell].inputs:
        yield pin, inst.conns[pin]
