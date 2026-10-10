"""独立校验器：不信任插入算法给出的级数，从网表结构本身重新推一遍。

检查项
1. 单驱动：每根网线恰好一个驱动；不存在悬空输入。
2. 单负载（AQFP 约束 ①）：除常量外，每根网线恰好一个负载（单元输入脚或主输出）。
   splitter 的每个输出脚各自算一根网线。
3. 相位一致（AQFP 约束 ②）：每个单元的所有输入来自同一级；主输入视为第 0 级。
4. 输出对齐：所有主输出来自同一级（可关闭）。
5. 功能等价：与参考网表在穷举/随机向量下逐位比对。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .netlist import CONSTS, Netlist, NetlistError
from .sim import equivalent


@dataclass
class Report:
    ok: bool = True
    errors: List[str] = field(default_factory=list)
    info: Dict[str, object] = field(default_factory=dict)

    def fail(self, msg: str) -> None:
        self.ok = False
        self.errors.append(msg)

    def __str__(self) -> str:
        head = "通过 ✅" if self.ok else f"失败 ❌（{len(self.errors)} 处）"
        lines = [f"校验{head}"]
        for k, v in self.info.items():
            lines.append(f"  {k}: {v}")
        for e in self.errors[:30]:
            lines.append(f"  - {e}")
        if len(self.errors) > 30:
            lines.append(f"  ……另有 {len(self.errors) - 30} 处")
        return "\n".join(lines)


def structural_levels(nl: Netlist, balance_pis: bool = True, report: Optional[Report] = None) -> Dict[str, int]:
    """从结构推出每个单元的级数；发现输入相位不一致时记入 report。"""
    report = report or Report()
    drv = nl.drivers()
    level: Dict[str, int] = {}
    pi_level: Dict[str, Optional[int]] = {pi: (0 if balance_pis else None) for pi in nl.pis}
    lib = nl.lib
    for name in nl.topo_order():
        inst = nl.insts[name]
        cd = lib[inst.cell]
        src: Dict[str, Optional[int]] = {}
        for p in cd.inputs:
            net = inst.conns[p]
            if net in CONSTS:
                continue
            d = drv[net][0]
            src[p] = level[d] if d is not None else pi_level[net]
        known = [v for v in src.values() if v is not None]
        L = (max(known) + 1) if known else 1
        for p, v in src.items():
            if v is None:  # 自由主输入：由第一个读它的单元确定
                pi_level[inst.conns[p]] = L - 1
            elif v != L - 1:
                report.fail(f"相位不一致：{name}({inst.cell}).{p} 的信号来自第 {v} 级，"
                            f"但该单元在第 {L} 级（需要第 {L - 1} 级）")
        level[name] = L
        if inst.level is not None and inst.level != L:
            report.fail(f"{name} 标注为第 {inst.level} 级，结构推出第 {L} 级")
    report.info["_pi_level"] = pi_level
    return level


def verify(nl: Netlist, reference: Optional[Netlist] = None, balance_pis: bool = True,
           balance_pos: bool = True, check_function: bool = True) -> Report:
    rep = Report()
    lib = nl.lib
    try:
        drv = nl.drivers()
        snk = nl.sinks()
        nl.topo_order()
    except NetlistError as e:
        rep.fail(str(e))
        return rep

    for inst in nl.insts.values():
        if inst.cell not in lib:
            rep.fail(f"{inst.name} 使用了库里没有的单元 {inst.cell}")
    for net, sinks in snk.items():
        if net not in drv:
            rep.fail(f"网线 {net} 没有驱动（被 {[s.inst or 'PO' for s in sinks]} 读取）")
        if net in CONSTS:
            continue
        if len(sinks) != 1:
            who = [f"{s.inst}.{s.pin}" if s.inst else f"输出 {s.pin}" for s in sinks]
            rep.fail(f"扇出违规：网线 {net} 驱动了 {len(sinks)} 个负载 {who}（AQFP 每个输出只能驱动 1 个）")
    for net, (inst, pin) in drv.items():
        if inst is not None and net not in snk:
            rep.fail(f"{inst}.{pin} 的输出 {net} 没有负载")

    level = structural_levels(nl, balance_pis, rep)
    pi_level = rep.info.pop("_pi_level")
    po_lv = {}
    for po in nl.pos:
        net = nl.po_drivers.get(po, po)
        if net in CONSTS:
            continue
        d = drv.get(net, (None, ""))[0]
        po_lv[po] = level[d] if d is not None else pi_level.get(net)  # type: ignore[union-attr]
    vals = {v for v in po_lv.values() if v is not None}
    if balance_pos and len(vals) > 1:
        rep.fail(f"主输出相位不对齐：{po_lv}")
    rep.info["深度（输出相位）"] = max(vals) if vals else 0
    roles = nl.count_by_role()
    rep.info["单元数"] = (f"逻辑门 {roles.get('gate', 0) + roles.get('inverter', 0)}，"
                       f"buffer {roles.get('buffer', 0)}，splitter {roles.get('splitter', 0)}，"
                       f"合计 {len(nl.insts)}")

    if check_function and reference is not None:
        try:
            cex = equivalent(reference, nl)
        except Exception as e:  # 端口不一致等
            rep.fail(f"功能比对无法进行：{e}")
        else:
            if cex is not None:
                rep.fail(f"功能不等价，反例输入：{cex}")
            else:
                n = len(nl.pis)
                rep.info["功能等价"] = "穷举全部输入一致" if n <= 14 else "2048 组随机输入一致"
    return rep
