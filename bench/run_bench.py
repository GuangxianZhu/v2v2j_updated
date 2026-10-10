"""基准测试：原版 vs 新版各调度算法。

运行：python bench/run_bench.py      （需要 yowasp-yosys 或 yosys；LP 需要 scipy）
输出：bench/results.csv 与 bench/RESULTS.md

度量的是插入的 buffer + splitter 个数（AQFP 里两者面积/JJ 数相近，mockturtle 文档也把它们视为同一种元件），
在"输出相位 = 最小可行深度"的约束下比较。所有结果都经过独立校验器 + 流水线相位仿真。
"""
import csv
import glob
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from v2v2j.insert import insert  # noqa: E402
from v2v2j.legacy import legacy_to_netlist, run_legacy  # noqa: E402
from v2v2j.library import Library  # noqa: E402
from v2v2j.schedule import Config  # noqa: E402
from v2v2j.sim import stream_check  # noqa: E402
from v2v2j.verify import verify  # noqa: E402
from v2v2j.writers import scalarize, to_verilog  # noqa: E402
from v2v2j.yosys_flow import rtl_to_netlist  # noqa: E402

LIB_PATH = os.path.join(os.path.dirname(HERE), "lib", "aqfp_hstp033_vlib.v")
METHODS = [("ASAP", Config(method="asap", refine=False)),
           ("ALAP", Config(method="alap", refine=False)),
           ("LP", Config(method="lp", refine=False)),
           ("best", Config(method="best", refine=True))]


def legacy_result(nl, lib):
    text = to_verilog(scalarize(nl))
    try:
        data = run_legacy(LIB_PATH, text)
    except Exception as e:
        return f"崩溃（{type(e).__name__}）", None, None
    old = legacy_to_netlist(data, lib)
    ref = scalarize(nl)
    rep = verify(old, reference=ref)
    roles = old.count_by_role()
    bs = roles.get("buffer", 0) + roles.get("splitter", 0)
    depth = max(i.level for i in old.insts.values())
    w, x, t = stream_check(old, ref, n=32, po_level=depth)
    status = "通过" if rep.ok and w == 0 and x == 0 else f"错误：{len(rep.errors)} 处违规，流水线 {w + x}/{t} 位错/X"
    return status, bs, depth


def main():
    lib = Library.from_file(LIB_PATH)
    rows = []
    for path in sorted(glob.glob(os.path.join(HERE, "rtl", "*.v"))):
        name = os.path.basename(path)[:-2]
        nl = rtl_to_netlist([path], lib)
        row = {"电路": name, "输入位": len(nl.pis), "输出位": len(nl.pos), "逻辑门": len(nl.insts)}
        st, bs, d = legacy_result(nl, lib)
        row["原版 B+S"] = bs if bs is not None else "—"
        row["原版"] = st
        for label, cfg in METHODS:
            t0 = time.time()
            res = insert(nl, cfg)
            dt = time.time() - t0
            rep = verify(res.netlist, reference=nl)
            w, x, t = stream_check(res.netlist, nl, n=32, po_level=res.schedule.depth)
            assert rep.ok and w == 0 and x == 0, (name, label, str(rep)[:500])
            row[f"{label} B+S"] = res.stats["buffers"] + res.stats["splitters"]
            row["深度"] = res.stats["depth"]
            if label == "best":
                row["best 来自"] = res.schedule.method
                row["best 用时(s)"] = round(dt, 2)
        rows.append(row)
        print(row, flush=True)

    keys = list(rows[0].keys())
    with open(os.path.join(HERE, "results.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    cols = ["电路", "逻辑门", "深度", "原版 B+S", "ASAP B+S", "ALAP B+S", "LP B+S", "best B+S", "best 来自", "原版"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        lines.append("| " + " | ".join(str(r[c]) for c in cols) + " |")
    tot = {c: sum(r[c] for r in rows) for c in ["ASAP B+S", "ALAP B+S", "LP B+S", "best B+S"]}
    lines.append("| **合计** | | | | " + " | ".join(str(tot[c]) for c in ["ASAP B+S", "ALAP B+S", "LP B+S", "best B+S"]) + " | | |")
    with open(os.path.join(HERE, "_last_run_table.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
