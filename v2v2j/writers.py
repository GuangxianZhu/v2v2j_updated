"""输出：Verilog 网表、JSON、Graphviz DOT、文本报告。JoSIM 网表见 josim.py。"""
from __future__ import annotations

import json
import re
from typing import Dict, List, Optional

from .netlist import CONSTS, Netlist

_SIMPLE_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_$]*$")
_PORT_BIT = re.compile(r"^(.+)\[(\d+)\]$")


def vid(name: str) -> str:
    """Verilog 标识符：不合法的名字用转义写法 \\name 加空格。"""
    return name if _SIMPLE_ID.match(name) else f"\\{name} "


def _net_ref(net: str, port_bits: set) -> str:
    if net in CONSTS:
        return net
    if net in port_bits:
        m = _PORT_BIT.match(net)
        if m:
            return f"{vid(m.group(1))}[{m.group(2)}]"
    return vid(net)


# 励磁方案：相位 -> (交流线, 交流线是否反接, 直流线是否反接)。
# 照抄 JoSIM 仓库自带的 HSTP π-DQFP buffer 链例子（逐个实例核对过节点）：
# 两条交流线 X1、X2（相差 90°）+ 一条直流偏置线；交流反接 = 再移相 180°。
#   相位 0：X1 正接、DC 正接    相位 1：X2 正接、DC 反接
#   相位 2：X1 反接、DC 正接    相位 3：X2 反接、DC 反接
# 这是"该单元库的物理约定"，换库时需要核对。
PHASE_SCHEME = {0: ("X1", False, False), 1: ("X2", False, True),
                2: ("X1", True, False), 3: ("X2", True, True)}


def phase_of_level(level: int) -> int:
    """第 1 级单元用相位 0（主输入算第 0 级，由外部在相位 3 提供）。"""
    return (level - 1) % 4


def excitation_chains(nl: Netlist) -> Dict[str, List[tuple]]:
    """把单元按相位串成励磁线：{线名: [(实例名, 是否反接), ...]}，以及直流线 "DC"。"""
    order = sorted(nl.insts.values(), key=lambda i: ((i.level or 0), i.name))
    chains: Dict[str, List[tuple]] = {"X1": [], "X2": [], "DC": []}
    for inst in order:
        line, rev, dc_rev = PHASE_SCHEME[phase_of_level(inst.level or 1)]
        chains[line].append((inst.name, rev))
        chains["DC"].append((inst.name, dc_rev))
    # 同一条线上：先串正接的，再串反接的（串联电流处处相同，顺序不影响电学结果）
    for k in ("X1", "X2"):
        chains[k].sort(key=lambda t: t[1])
    return chains


def scalarize(nl: Netlist) -> Netlist:
    """把总线端口和所有网线名改成简单标识符（a[3] → a_3_），给只认简单名字的老工具（如原版）用。"""
    from .netlist import Port, sanitize
    out = nl.copy()
    ren = {}
    used = set()
    for n in nl.all_nets():
        if n in CONSTS:
            continue
        new = sanitize(n)
        while new in used:
            new += "_"
        used.add(new)
        ren[n] = new
    ren.update({c: c for c in CONSTS})
    out.ports = [Port(ren[b], p.direction, [ren[b]]) for p in nl.ports for b in p.bits]
    for inst in out.insts.values():
        inst.conns = {k: ren[v] for k, v in inst.conns.items()}
    out.po_drivers = {ren[k]: ren[v] for k, v in nl.po_drivers.items()}
    return out


def to_verilog(nl: Netlist, header: str = "", with_bias: bool = False) -> str:
    lib = nl.lib
    port_bits = {b for p in nl.ports for b in p.bits if p.width > 1}
    lines = []
    if header:
        lines += [f"// {h}" for h in header.splitlines()]
    lines.append(f"module {vid(nl.name)}({', '.join(vid(p.name) for p in nl.ports)}"
                 + (", xclk1, xclk2, dcin" if with_bias else "") + ");")
    for p in nl.ports:
        rng = f" [{p.width - 1}:0]" if p.width > 1 else ""
        lines.append(f"  {p.direction}{rng} {vid(p.name)};")
    if with_bias:
        lines.append("  inout xclk1, xclk2, dcin;")
    io = set(nl.pis) | set(nl.pos)
    wires = sorted({n for i in nl.insts.values() for n in i.conns.values()} - io - set(CONSTS))
    for w in wires:
        lines.append(f"  wire {vid(w)};")

    bias: Dict[str, Dict[str, str]] = {}
    if with_bias:
        chains = excitation_chains(nl)
        for line, items in chains.items():
            src = {"X1": "xclk1", "X2": "xclk2", "DC": "dcin"}[line]
            prev = src
            for j, (iname, rev) in enumerate(items):
                nxt = f"{line.lower()}_{j}" if j < len(items) - 1 else None
                if nxt:
                    lines.append(f"  wire {nxt};")
                pins = ("xin", "xout") if line != "DC" else ("din", "dout")
                a, b = (pins[1], pins[0]) if rev else pins
                bias.setdefault(iname, {})[a] = prev
                bias[iname][b] = nxt or ""
                prev = nxt or ""
    lines.append("")
    for inst in sorted(nl.insts.values(), key=lambda i: ((i.level or 0), i.name)):
        cd = lib[inst.cell]
        conns = []
        for pin in cd.ports:
            if pin in inst.conns:
                conns.append(f".{pin}({_net_ref(inst.conns[pin], port_bits)})")
            elif pin in bias.get(inst.name, {}):
                conns.append(f".{pin}({bias[inst.name][pin]})")
        note = f"  // L{inst.level} 相位{phase_of_level(inst.level)}" if inst.level is not None else ""
        lines.append(f"  {inst.cell} {vid(inst.name)} ({', '.join(conns)});{note}")
    for po in nl.pos:
        if po in nl.po_drivers:
            lines.append(f"  assign {_net_ref(po, port_bits)} = {_net_ref(nl.po_drivers[po], port_bits)};")
    lines.append("endmodule")
    return "\n".join(lines) + "\n"


def to_json(nl: Netlist, extra: Optional[dict] = None) -> str:
    lib = nl.lib
    data = {
        "module": nl.name,
        "ports": [{"name": p.name, "direction": p.direction, "bits": p.bits} for p in nl.ports],
        "po_aliases": nl.po_drivers,
        "cells": [
            {
                "name": i.name, "type": i.cell, "role": lib[i.cell].role, "origin": i.origin,
                "level": i.level, "phase": phase_of_level(i.level) if i.level is not None else None,
                "inputs": {p: i.conns[p] for p in lib[i.cell].inputs if p in i.conns},
                "outputs": {p: i.conns[p] for p in lib[i.cell].outputs if p in i.conns},
            }
            for i in sorted(nl.insts.values(), key=lambda i: ((i.level or 0), i.name))
        ],
    }
    if extra:
        data.update(extra)
    return json.dumps(data, ensure_ascii=False, indent=1)


def to_dot(nl: Netlist) -> str:
    """按级数分列的连线图：每一列是一个相位，buffer 灰色、splitter 黄色。"""
    lib = nl.lib
    drv = nl.drivers()
    color = {"buffer": "#d9d9d9", "splitter": "#ffe599", "gate": "#cfe2f3", "inverter": "#f4cccc"}
    out = ["digraph aqfp {", "  rankdir=LR; node [shape=box, style=filled, fontname=Helvetica];"]
    by_level: Dict[int, List[str]] = {}
    for i in nl.insts.values():
        by_level.setdefault(i.level or 0, []).append(i.name)
        out.append(f'  "{i.name}" [label="{i.cell}\\n{i.name}\\nL{i.level}", '
                   f'fillcolor="{color.get(lib[i.cell].role, "white")}"];')
    for pi in nl.pis:
        out.append(f'  "PI:{pi}" [label="{pi}", shape=circle, fillcolor="#b6d7a8"];')
    for po in nl.pos:
        out.append(f'  "PO:{po}" [label="{po}", shape=doublecircle, fillcolor="#b6d7a8"];')
    out.append('  { rank=same; ' + " ".join(f'"PI:{p}";' for p in nl.pis) + " }")
    for L, names in sorted(by_level.items()):
        out.append("  { rank=same; " + " ".join(f'"{n}";' for n in names) + " }")
    for i in nl.insts.values():
        for p in lib[i.cell].inputs:
            net = i.conns[p]
            if net in CONSTS:
                continue
            d = drv[net][0]
            src = d if d is not None else f"PI:{net}"
            out.append(f'  "{src}" -> "{i.name}";')
    for po in nl.pos:
        net = nl.po_drivers.get(po, po)
        if net in CONSTS:
            continue
        d = drv[net][0]
        out.append(f'  "{d if d is not None else "PI:" + net}" -> "PO:{po}";')
    out.append("}")
    return "\n".join(out) + "\n"
