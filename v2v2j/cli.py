"""命令行入口：python -m v2v2j <子命令> ...

  run       门级或 RTL Verilog → 插入 buffer/splitter → Verilog + JSON + DOT (+ JoSIM)
  verify    检查一个（已插好的）网表是否满足 AQFP 约束
  compare   原版算法 vs 新版，同一把尺子评判
  josim-read  读 JoSIM 仿真结果，与期望输出逐周期比对
"""
from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import subprocess
import sys
from typing import List, Optional

from . import __version__
from .insert import insert
from .library import Library
from .netlist import Netlist
from .schedule import Config
from .sim import comb_eval, stream_check
from .verify import verify
from .verilog_parser import parse_verilog_file
from .writers import to_dot, to_json, to_verilog

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LIB = os.path.join(os.path.dirname(HERE), "lib", "aqfp_hstp033_vlib.v")


def _load(args) -> tuple:
    lib = Library.from_file(args.lib)
    for w in lib.lint:
        print(f"[单元库警告] {w}", file=sys.stderr)
    if args.rtl:
        from .yosys_flow import rtl_to_netlist
        nl = rtl_to_netlist(args.inputs, lib, args.top, args.gates)
    else:
        if len(args.inputs) != 1:
            raise SystemExit("门级输入只支持一个文件（RTL 才能多个，加 --rtl）")
        nl = parse_verilog_file(args.inputs[0], lib, args.top)
    return lib, nl


def _cfg(args) -> Config:
    return Config(max_fanout=args.max_fanout, balance_pis=not args.free_pis,
                  balance_pos=not args.free_pos, branch_pis=args.branch_pis,
                  method=args.method, refine=not args.no_refine, depth=args.depth)


def cmd_run(args) -> int:
    lib, nl = _load(args)
    cfg = _cfg(args)
    res = insert(nl, cfg)
    out = res.netlist
    os.makedirs(args.outdir, exist_ok=True)
    base = os.path.join(args.outdir, nl.name)
    rep = verify(out, reference=nl, balance_pis=cfg.balance_pis, balance_pos=cfg.balance_pos)
    w, x, t = stream_check(out, nl, n=64, po_level=res.schedule.depth) if (cfg.balance_pis and cfg.balance_pos) else (0, 0, 0)

    header = (f"由 v2v2j {__version__} 生成：{len(nl.insts)} 个逻辑门 → 插入 buffer {res.stats['buffers']} 个、"
              f"splitter {res.stats['splitters']} 个；深度 {res.stats['depth']}；调度 {res.schedule.method}")
    with open(base + "_aqfp.v", "w", encoding="utf-8") as f:
        f.write(to_verilog(out, header=header, with_bias=args.bias))
    with open(base + "_aqfp.json", "w", encoding="utf-8") as f:
        f.write(to_json(out, {"stats": res.stats, "schedule": res.schedule.method,
                              "config": vars(cfg)}))
    with open(base + "_aqfp.dot", "w", encoding="utf-8") as f:
        f.write(to_dot(out))
    if shutil.which("dot") and args.png:
        subprocess.run(["dot", "-Tpng", base + "_aqfp.dot", "-o", base + "_aqfp.png"], check=False)

    lines = [f"模块 {nl.name}：{len(nl.pis)} 个输入位，{len(nl.pos)} 个输出位，{len(nl.insts)} 个逻辑门",
             f"单元库：{lib.summary()}",
             f"调度：{res.schedule.method}（{'；'.join(res.schedule.notes)}）",
             f"结果：buffer {res.stats['buffers']}，splitter {res.stats['splitters']}，"
             f"深度 {res.stats['depth']}，单元总数 {res.stats['jj_cells_total']}"]
    lines += [f"警告：{w_}" for w_ in res.warnings]
    lines.append(str(rep))
    if t:
        lines.append(f"流水线相位仿真：连续 64 个周期每周期一组新输入，{t} 个输出位中错误 {w}、X {x}")
    if args.josim_cellmap:
        lines.append(_emit_josim(args, out, nl, base))
    text = "\n".join(lines)
    print(text)
    with open(base + "_report.txt", "w", encoding="utf-8") as f:
        f.write(text + "\n")
    return 0 if rep.ok else 1


def _emit_josim(args, out: Netlist, golden: Netlist, base: str) -> str:
    from .josim import JosimConfig, to_josim
    with open(args.josim_cellmap, encoding="utf-8") as f:
        cm = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    missing = sorted({i.cell for i in out.insts.values()} - set(cm))
    rng = random.Random(args.seed)
    stream = [{p: rng.getrandbits(1) for p in out.pis} for _ in range(args.vectors)]
    cfg = JosimConfig(include=os.path.abspath(args.josim_include) if args.josim_include else "cells.cir", cellmap=cm)
    txt, timing = to_josim(out, stream, cfg)
    order = golden.topo_order()
    timing["expected"] = [comb_eval(golden, v, order) for v in stream]
    with open(base + ".cir", "w", encoding="utf-8") as f:
        f.write(txt)
    with open(base + ".timing.json", "w", encoding="utf-8") as f:
        json.dump(timing, f, ensure_ascii=False, indent=1)
    msg = f"JoSIM 网表：{base}.cir（{args.vectors} 个周期的随机输入；时序信息 {base}.timing.json）"
    if missing:
        msg += f"\n  注意：cellmap 里没有 {missing}，这些单元按 Verilog 库端口顺序、同名子电路输出"
    return msg


def cmd_verify(args) -> int:
    lib = Library.from_file(args.lib)
    nl = parse_verilog_file(args.inputs[0], lib, args.top)
    ref = parse_verilog_file(args.reference, lib, args.top) if args.reference else None
    rep = verify(nl, reference=ref, balance_pis=not args.free_pis, balance_pos=not args.free_pos)
    print(rep)
    return 0 if rep.ok else 1


def cmd_compare(args) -> int:
    from .legacy import legacy_to_netlist, run_legacy
    lib, nl = _load(args)
    print(f"模块 {nl.name}：{len(nl.insts)} 个逻辑门\n")
    print(f"{'版本':8s}{'buffer':>8s}{'splitter':>10s}{'深度':>6s}  校验 / 流水线仿真")
    try:
        with open(args.inputs[0], encoding="utf-8") as f:
            text = f.read()
        if args.rtl:
            from .writers import scalarize
            nl = scalarize(nl)  # 原版的正则不认总线，先把端口拆成标量
            text = to_verilog(nl)
        data = run_legacy(args.lib, text)
        old = legacy_to_netlist(data, lib)
        rep = verify(old, reference=nl)
        roles = old.count_by_role()
        w, x, t = stream_check(old, nl, n=64, po_level=max(i.level for i in old.insts.values()))
        print(f"{'原版':8s}{roles.get('buffer', 0):>8d}{roles.get('splitter', 0):>10d}"
              f"{max(i.level for i in old.insts.values()):>6d}  "
              f"{'通过' if rep.ok else '失败 %d 处' % len(rep.errors)} / 错误{w} X{x} (共{t})")
        for e in rep.errors[:6]:
            print(f"          - {e}")
    except Exception as e:  # 原版直接崩溃的情况
        print(f"{'原版':8s}{'—':>8s}{'—':>10s}{'—':>6s}  崩溃：{type(e).__name__}: {e}")
    res = insert(nl, _cfg(args))
    rep = verify(res.netlist, reference=nl)
    w, x, t = stream_check(res.netlist, nl, n=64, po_level=res.schedule.depth)
    print(f"{'新版':8s}{res.stats['buffers']:>8d}{res.stats['splitters']:>10d}{res.stats['depth']:>6d}  "
          f"{'通过' if rep.ok else '失败'} / 错误{w} X{x} (共{t})   [{res.schedule.method}]")
    return 0


def cmd_josim_read(args) -> int:
    from .josim import read_results
    with open(args.timing, encoding="utf-8") as f:
        timing = json.load(f)
    got = read_results(args.csv, timing, args.threshold)
    exp = timing.get("expected")
    bad = 0
    for k, row in enumerate(got):
        e = exp[k] if exp else None
        ok = e is None or all(row[p] == e[p] for p in row)
        bad += not ok
        if args.verbose or not ok:
            print(f"周期 {k:3d}：读到 {row}" + ("" if e is None else f"  期望 {e}  {'✓' if ok else '✗'}"))
    print(f"{len(got)} 个周期，{'全部正确' if bad == 0 else f'{bad} 个周期有错'}")
    return 0 if bad == 0 else 1


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="v2v2j", description="AQFP buffer/splitter 插入工具（v2v2j 升级版）")
    p.add_argument("--version", action="version", version=__version__)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp, multi=True):
        sp.add_argument("inputs", nargs="+" if multi else 1, help="Verilog 文件")
        sp.add_argument("--lib", default=DEFAULT_LIB, help="AQFP 单元库（行为级 Verilog）")
        sp.add_argument("--top", help="顶层模块名")
        sp.add_argument("--free-pis", action="store_true", help="不要求主输入在同一相位到达")
        sp.add_argument("--free-pos", action="store_true", help="不要求主输出对齐到同一相位")

    def algo(sp):
        sp.add_argument("--rtl", action="store_true", help="输入是 RTL，先用 Yosys 综合")
        sp.add_argument("--gates", default="AND,OR", help="Yosys abc -g 的门类型（默认 AND,OR）")
        sp.add_argument("--method", default="best", choices=["asap", "alap", "lp", "best"])
        sp.add_argument("--no-refine", action="store_true", help="关闭精确代价局部搜索")
        sp.add_argument("--max-fanout", type=int, help="splitter 最大扇出")
        sp.add_argument("--branch-pis", action="store_true", help="主输入可不经 splitter 直接分支")
        sp.add_argument("--depth", type=int, help="指定输出相位（≥ 最小深度），用于对齐到外部时序")

    r = sub.add_parser("run", help="插入 buffer/splitter 并输出")
    common(r)
    algo(r)
    r.add_argument("-o", "--outdir", default="out")
    r.add_argument("--bias", action="store_true", help="Verilog 里按相位串好励磁线（xclk1/xclk2/dcin）")
    r.add_argument("--png", action="store_true", help="若装了 graphviz，顺便渲染 PNG")
    r.add_argument("--josim-cellmap", help="生成 JoSIM 网表：单元名 → 子电路与端口顺序（JSON）")
    r.add_argument("--josim-include", help="JoSIM 网表里 .include 的单元库文件路径")
    r.add_argument("--vectors", type=int, default=16, help="JoSIM 仿真施加的随机输入周期数")
    r.add_argument("--seed", type=int, default=1)
    r.set_defaults(func=cmd_run)

    v = sub.add_parser("verify", help="校验已插好的门级网表")
    common(v, multi=False)
    v.add_argument("--reference", help="插入前的网表，用于功能等价比对")
    v.set_defaults(func=cmd_verify)

    c = sub.add_parser("compare", help="原版算法 vs 新版")
    common(c)
    algo(c)
    c.set_defaults(func=cmd_compare)

    j = sub.add_parser("josim-read", help="读 JoSIM CSV 并与期望比对")
    j.add_argument("csv")
    j.add_argument("timing")
    j.add_argument("--threshold", type=float, default=4.0, help="判 0/1 的电流阈值 (µA)")
    j.add_argument("-v", "--verbose", action="store_true")
    j.set_defaults(func=cmd_josim_read)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
