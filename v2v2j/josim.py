"""JoSIM 网表生成与结果读取（v2v2j 里的那个 "j"）。

生成内容
- 每个单元一行 ``X<实例> <子电路名> <节点...>``（JoSIM 默认的 JSIM 顺序：名字在前、节点在后）；
- 励磁：两条交流线 X1、X2（相差 90°）+ 一条直流线，按 writers.PHASE_SCHEME 串联：
  相位 0 = X1 正接，1 = X2 正接，2 = X1 反接，3 = X2 反接。电流源用"电压源 + 1kΩ"实现；
- 主输入：PWL 电流（±in_amp），每个时钟周期换一组；
- 主输出：每个输出后面接一个下一相位的 buffer 作负载（AQFP 输出必须有负载），
  并串一个 0.1pH 小电感 Lpo_<名> 作电流探针；
- 旁路文件 *.timing.json：记录每组输入何时施加、每个输出何时采样，供 read_results 判读。

默认时序参数取自 JoSIM 仓库自带的 HSTP π-DQFP buffer 链例子（5 GHz，X1 延迟 100ps，X2 延迟 150ps），
实测第 1 级单元在 t = 250ps + n·200ps 输出达到峰值，此后每级晚 50ps（1/4 周期）。
换成你自己的单元库时，这些数要按你的库重新标定——这是本模块最强的假设。
"""
from __future__ import annotations

import csv
import json
import re
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from .netlist import CONSTS, Netlist
from .writers import excitation_chains, phase_of_level


@dataclass
class JosimConfig:
    include: str = "cells.cir"                    # 单元库子电路文件
    cellmap: Dict[str, dict] = field(default_factory=dict)  # 单元名 -> {"subckt":..., "ports":[...]}
    freq_ghz: float = 5.0
    x_amp_mv: float = 811.0                       # 交流励磁幅度（经 1kΩ → 811 µA）
    x1_delay_ps: float = 100.0
    x2_delay_ps: float = 150.0
    dc_mv: float = 1131.0
    in_amp_mv: float = 5.0                        # 输入 ±5 mV / 1kΩ = ±5 µA
    first_peak_ps: float = 250.0                  # 第 1 级单元第一次输出峰值时刻
    setup_ps: float = 99.0                        # 输入在第 1 级峰值前多久切换
    tstep_ps: float = 0.2
    probe_l_ph: float = 0.1

    @property
    def period_ps(self) -> float:
        return 1000.0 / self.freq_ghz


def _node(net: str, nodes: Dict[str, str]) -> str:
    if net not in nodes:
        s = re.sub(r"[^A-Za-z0-9_]", "_", net)
        nodes[net] = f"n_{s}"
        # 防碰撞
        vals = list(nodes.values())
        if vals.count(nodes[net]) > 1:
            nodes[net] = f"{nodes[net]}_{len(nodes)}"
    return nodes[net]


def to_josim(nl: Netlist, stream: List[Dict[str, int]], cfg: Optional[JosimConfig] = None,
             title: str = "") -> tuple:
    """返回 (网表文本, 时序信息 dict)。stream 是逐周期施加的输入向量。"""
    cfg = cfg or JosimConfig()
    lib = nl.lib
    T = cfg.period_ps
    D = max((i.level or 0) for i in nl.insts.values()) if nl.insts else 0
    nodes: Dict[str, str] = {}
    out: List[str] = [f"* {title or nl.name} —— 由 v2v2j 生成", f".include {cfg.include}", ""]

    # ---- 主输出负载：下一相位的 buffer ----
    insts = list(nl.insts.values())
    loads = []
    buf = lib.buffer
    for po in nl.pos:
        src = nl.po_drivers.get(po, po)
        if src in CONSTS:
            continue
        loads.append((po, src))

    # 为励磁串链准备一个"伪网表"：真实单元 + 负载 buffer
    class _Tmp:
        def __init__(self, name, cell, level):
            self.name, self.cell, self.level = name, cell, level

    class _NL:
        pass

    tmp = _NL()
    tmp.insts = {i.name: i for i in insts}
    for po, _ in loads:
        n = f"load_{re.sub(r'[^A-Za-z0-9_]', '_', po)}"
        tmp.insts[n] = _Tmp(n, buf, D + 1)
    chains = excitation_chains(tmp)  # type: ignore[arg-type]

    # 每个实例的偏置脚节点
    bias: Dict[str, Dict[str, str]] = {}
    src_node = {"X1": "nx1", "X2": "nx2", "DC": "ndc"}
    for line, items in chains.items():
        prev = src_node[line]
        for j, (iname, rev) in enumerate(items):
            nxt = f"{line.lower()}_{j}" if j < len(items) - 1 else "0"
            pins = ("xin", "xout") if line != "DC" else ("din", "dout")
            a, b = (pins[1], pins[0]) if rev else pins
            bias.setdefault(iname, {})[a] = prev
            bias[iname][b] = nxt
            prev = nxt

    out.append("* 励磁与直流偏置（电压源经 1kΩ 近似电流源）")
    out.append(f"Vx1 vx1 0 SIN(0 {cfg.x_amp_mv}mV {cfg.freq_ghz}GHz {cfg.x1_delay_ps}ps 0)")
    out.append(f"Vx2 0 vx2 SIN(0 {cfg.x_amp_mv}mV {cfg.freq_ghz}GHz {cfg.x2_delay_ps}ps 0)")
    out.append(f"Vdc vdc 0 PWL(0ps 0mV 20ps {cfg.dc_mv}mV)")
    out.append("Rx1 vx1 nx1 1000ohm")
    out.append("Rx2 vx2 nx2 1000ohm")
    out.append("Rdc vdc ndc 1000ohm")
    out.append("")

    # ---- 主输入：PWL ----
    out.append("* 主输入：每个时钟周期一组向量")
    timing = {"period_ps": T, "inputs": {}, "outputs": {}, "depth": D, "n_vectors": len(stream)}
    t_first = cfg.first_peak_ps
    for pi in nl.pis:
        pts = ["0ps 0mV"]
        level_prev = None
        for k, vec in enumerate(stream):
            t_sw = t_first + k * T - cfg.setup_ps
            v = cfg.in_amp_mv if vec[pi] else -cfg.in_amp_mv
            if level_prev is None:
                pts.append(f"1ps {v}mV")
            elif v != level_prev:
                pts.append(f"{t_sw:.1f}ps {level_prev}mV")
                pts.append(f"{t_sw + 1:.1f}ps {v}mV")
            level_prev = v
        nd = _node(pi, nodes)
        tag = re.sub(r"[^A-Za-z0-9_]", "_", pi)
        out.append(f"Vin_{tag} vin_{tag} 0 PWL({' '.join(pts)})")
        out.append(f"Rin_{tag} vin_{tag} {nd} 1000ohm")
        timing["inputs"][pi] = [vec[pi] for vec in stream]
    out.append("")

    # ---- 单元实例 ----
    out.append("* 单元实例（L=级数，P=相位）")
    for inst in sorted(insts, key=lambda i: ((i.level or 0), i.name)):
        cm = cfg.cellmap.get(inst.cell, {})
        sub = cm.get("subckt", inst.cell)
        ports = cm.get("ports", lib[inst.cell].ports)
        nds = []
        for p in ports:
            if p in inst.conns:
                nds.append(_node(inst.conns[p], nodes) if inst.conns[p] not in CONSTS else "0")
            elif p in bias.get(inst.name, {}):
                nds.append(bias[inst.name][p])
            else:
                nds.append("0")
        # JoSIM 不支持行尾注释，所以级数/相位写在上一行
        out.append(f"* {inst.name}: {inst.cell} L{inst.level} P{phase_of_level(inst.level)}")
        out.append(f"X{re.sub(r'[^A-Za-z0-9_]', '_', inst.name)} {sub} {' '.join(nds)}")
    # ---- 输出负载与探针 ----
    out.append("")
    out.append("* 主输出：0.1pH 探针电感 + 下一相位 buffer 作负载")
    cm = cfg.cellmap.get(buf, {})
    sub = cm.get("subckt", buf)
    ports = cm.get("ports", lib[buf].ports)
    a_pin, q_pin = lib[buf].inputs[0], lib[buf].outputs[0]
    for po, src in loads:
        tag = re.sub(r"[^A-Za-z0-9_]", "_", po)
        ld = f"load_{tag}"
        out.append(f"Lpo_{tag} {_node(src, nodes)} po_{tag} {cfg.probe_l_ph}pH")
        nds = []
        for p in ports:
            if p == a_pin:
                nds.append(f"po_{tag}")
            elif p == q_pin:
                nds.append("0")
            else:
                nds.append(bias[ld].get(p, "0"))
        out.append(f"X{ld} {sub} {' '.join(nds)}")
        d_lv = D
        for k in range(len(stream)):
            timing["outputs"].setdefault(po, []).append(
                round(t_first + k * T + (d_lv - 1) * T / 4, 3))
    t_end = t_first + (len(stream) + 1) * T + D * T / 4
    out.append("")
    out.append(f".tran {cfg.tstep_ps}ps {t_end:.0f}ps 0ps {cfg.tstep_ps}ps")
    for po, _ in loads:
        out.append(f".print i(Lpo_{re.sub(r'[^A-Za-z0-9_]', '_', po)})")
    out.append(".end")
    timing["config"] = asdict(cfg)
    return "\n".join(out) + "\n", timing


def read_results(csv_path: str, timing: dict, threshold_ua: float = 4.0) -> List[Dict[str, Optional[int]]]:
    """读 JoSIM 的 CSV，在每个输出的采样时刻取电流符号：> +阈值 → 1，< -阈值 → 0，否则 X。"""
    with open(csv_path, newline="") as f:
        rows = list(csv.reader(f))
    head = [h.strip().strip('"').upper() for h in rows[0]]
    data = [[float(x) for x in r] for r in rows[1:] if r]
    times = [r[0] for r in data]

    def at(col: int, t_ps: float) -> float:
        t = t_ps * 1e-12
        lo, hi = 0, len(times) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if times[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        return data[lo][col]

    n = timing["n_vectors"]
    res: List[Dict[str, Optional[int]]] = [dict() for _ in range(n)]
    for po, ts in timing["outputs"].items():
        tag = "I(LPO_" + re.sub(r"[^A-Za-z0-9_]", "_", po).upper() + ")"
        col = head.index(tag)
        for k, t in enumerate(ts):
            i_ua = at(col, t) * 1e6
            res[k][po] = 1 if i_ua > threshold_ua else (0 if i_ua < -threshold_ua else None)
    return res
