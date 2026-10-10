"""与 mockturtle（EPFL LSI 的逻辑综合库）的 AQFP buffer insertion 对比。

需要先自己编译 bench/mt_bench.cpp（见 README），然后：
    MT_BENCH=/path/to/mt_bench python bench/mockturtle_compare.py

做法：把同一个 AND/OR 逻辑网表（插入前）写成 assign 表达式形式的 Verilog 交给 mockturtle，
它读成 AIG（AIG 的取反边免费，和 AQFP 里反相免费是同一个假设，门数不变），
它先做结构哈希（门数会比 Yosys 的输出少），我们再把它实际使用的 AIG 读回来，两边跑同一个网表；
在同样的假设下（主输入需 splitter、主输入/输出都要对齐、splitter 最多 4 路）插入，比较 buffer+splitter 数。
"""
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from v2v2j.insert import insert  # noqa: E402
from v2v2j.library import Library  # noqa: E402
from v2v2j.schedule import Config  # noqa: E402
from v2v2j.writers import scalarize  # noqa: E402
from v2v2j.yosys_flow import rtl_to_netlist  # noqa: E402

LIB_PATH = os.path.join(os.path.dirname(HERE), "lib", "aqfp_hstp033_vlib.v")


def to_expr_verilog(nl) -> str:
    nl = scalarize(nl)
    lib = nl.lib
    lines = [f"module top({', '.join(nl.pis + nl.pos)});"]
    lines += [f"  input {p};" for p in nl.pis]
    lines += [f"  output {p};" for p in nl.pos]
    wires = sorted({i.conns[o] for i in nl.insts.values() for o in lib[i.cell].outputs} - set(nl.pos))
    lines += [f"  wire {w};" for w in wires]
    for inst in nl.insts.values():
        cd = lib[inst.cell]
        expr = cd.funcs[cd.outputs[0]]
        for pin in sorted(cd.inputs, key=len, reverse=True):
            expr = expr.replace(pin, f"@{pin}@")
        for pin in cd.inputs:
            expr = expr.replace(f"@{pin}@", inst.conns[pin])
        lines.append(f"  assign {inst.conns[cd.outputs[0]]} = {expr};")
    for po, src in nl.po_drivers.items():
        lines.append(f"  assign {po} = {src};")
    lines.append("endmodule")
    return "\n".join(lines) + "\n"


def parse_mt_aig(text: str, lib):
    """读 mockturtle write_verilog 输出的 AIG（assign n = [~]a & [~]b; assign y = [~]n;）。
    主输出上的取反：若驱动门只给这个输出用，就用德摩根律换成对偶门（免费）；否则放一个 inv。"""
    import re
    from v2v2j.netlist import Inst, Netlist, Port
    ins = re.search(r"input([^;]*);", text).group(1).replace(" ", "").split(",")
    outs = re.search(r"output([^;]*);", text).group(1).replace(" ", "").split(",")
    nl = Netlist("top", lib)
    nl.ports = [Port(x, "input", [x]) for x in ins] + [Port(x, "output", [x]) for x in outs]
    gates, po_lits = {}, {}
    for lhs, rhs in re.findall(r"assign\s+(\w+)\s*=\s*([^;]+);", text):
        lits = [t.strip() for t in rhs.split("&")]
        parsed = [(t.lstrip("~").strip(), t.startswith("~")) for t in lits]
        if lhs in outs:
            po_lits[lhs] = parsed[0]
        else:
            gates[lhs] = parsed
    uses = {}
    for g, lits in gates.items():
        for s, _ in lits:
            uses[s] = uses.get(s, 0) + 1
    for po, (s, _) in po_lits.items():
        uses[s] = uses.get(s, 0) + 1
    flip = {s for po, (s, inv) in po_lits.items() if inv and s in gates and uses[s] == 1}
    for g, ((a, ia), (b, ib)) in gates.items():
        kind = "AND"
        if g in flip:
            kind, ia, ib = "OR", not ia, not ib
        table = tuple(int(((x ^ ia) & (y ^ ib)) if kind == "AND" else ((x ^ ia) | (y ^ ib)))
                      for x in (0, 1) for y in (0, 1))
        cell, perm = lib.find_gate(table, 2)
        nl.add(Inst(g, cell, {perm[0]: a, perm[1]: b, lib[cell].outputs[0]: g}))
    for po, (s, inv) in po_lits.items():
        if inv and s not in flip:
            nl.add(Inst(f"inv_{po}", "inv", {"a": s, "q": po}))
        else:
            nl.po_drivers[po] = s
    return nl


def main():
    exe = os.environ.get("MT_BENCH")
    if not exe:
        raise SystemExit("请设置 MT_BENCH 指向编译好的 mt_bench")
    lib = Library.from_file(LIB_PATH)
    rows = []
    for path in sorted(glob.glob(os.path.join(HERE, "rtl", "*.v"))):
        name = os.path.basename(path)[:-2]
        nl = rtl_to_netlist([path], lib)
        vfile = os.path.join(HERE, "mt_tmp.v")
        with open(vfile, "w") as f:
            f.write(to_expr_verilog(nl))
        afile = os.path.join(HERE, "mt_tmp_aig.v")
        out = subprocess.run([exe, vfile, "4", afile], capture_output=True, text=True, check=True).stdout
        mt = json.loads(out)
        with open(afile) as f:
            aig = parse_mt_aig(f.read(), lib)   # 两边用同一个 AIG，保证可比
        os.remove(afile)
        ours = insert(aig, Config())
        from v2v2j.verify import verify
        assert verify(ours.netlist, reference=aig).ok
        n_inv = sum(1 for i in aig.insts.values() if i.cell == "inv")
        row = {"电路": name, "AIG 门数": mt["gates"], "输出端 inv": n_inv,
               "深度 v2v2j/mt": f"{ours.stats['depth']}/{mt['better_depth+until_sat']['depth'] - 4}",
               "v2v2j best": ours.stats["buffers"] + ours.stats["splitters"]}
        for k in ("ASAP_depth", "ALAP_depth", "better_depth", "better_depth+until_sat"):
            r = mt[k]
            row[f"mt {k}"] = f"{r['buffers']}{'' if r['ok'] else '(校验失败)'}"
        # mockturtle 的 depth 含主输入寄存器的 4 个相位偏移（ci_phases = {4}），上面已减去
        rows.append(row)
        print(row, flush=True)
    os.remove(vfile)
    cols = list(rows[0])
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(str(r[c]) for c in cols) + " |" for r in rows]
    with open(os.path.join(HERE, "_last_mt_table.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
