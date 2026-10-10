"""生成教学网页 docs/aqfp_demo.html：把示例电路的"原版 / 新版"结果算好嵌进页面。

运行：python docs/build_demo.py      （add4 示例需要 Yosys；没有就跳过）
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from v2v2j.insert import insert  # noqa: E402
from v2v2j.legacy import legacy_to_netlist, run_legacy  # noqa: E402
from v2v2j.library import Library  # noqa: E402
from v2v2j.verify import verify  # noqa: E402
from v2v2j.verilog_parser import parse_verilog  # noqa: E402
from v2v2j.writers import scalarize, to_verilog  # noqa: E402

LIB_PATH = os.path.join(ROOT, "lib", "aqfp_hstp033_vlib.v")
LIB = Library.from_file(LIB_PATH)

EXAMPLES = [
    dict(id="adder", title="全加器（原仓库自带）",
         desc="原版当年就是用它调试的：7 个门，a、b、c 都有扇出。",
         src=open(os.path.join(ROOT, "examples", "adder_mapped.v")).read(),
         quiz=dict(q="原版在这个电路上插了 14 个 buffer、6 个 splitter。新版会插多少个 buffer？",
                   options=["还是 14 个", "10 个", "6 个", "0 个"], answer=1,
                   explain="c 先分裂成两份、再各补 4 个 buffer，要 8 个；先补 4 个、最后一级再分裂只要 4 个。省下的 4 个就是 14→10。")),
    dict(id="reconv", title="重汇聚",
         desc="a 经两条长短不同的路到达 g1：一条直达，一条先过 g2。",
         src="""module reconv(a, b, q); input a; input b; output q; wire w;
  and_bb g1 (.a(a), .b(w), .q(q));
  and_bb g2 (.a(a), .b(b), .q(w));
endmodule""",
         quiz=dict(q="原版的 assign_levels 用 DFS、谁先访问到就定级。g1 在单元列表里排在 g2 前面，原版会把 g1 定在第几级？",
                   options=["第 2 级（只看了 a→g1 这条短路）", "第 3 级（最长路径）", "第 1 级", "报错"], answer=0,
                   explain="DFS 先从 a 走到 g1，给了 2；之后从 g2 再走到 g1 时，g1 已经有级数就不再更新。正确做法是 max(所有输入)+1 = 3。")),
    dict(id="pideep", title="主输入直连深层门",
         desc="c 没有扇出，直接接到第 2 级的 g2。",
         src="""module pideep(a, b, c, q); input a; input b; input c; output q; wire w;
  and_bb g1 (.a(a), .b(b), .q(w));
  and_bb g2 (.a(w), .b(c), .q(q));
endmodule""",
         quiz=dict(q="c 在第 0 相位到达，g2 在第 2 级。原版会在 c 和 g2 之间插 buffer 吗？",
                   options=["会，插 1 个", "不会"], answer=1,
                   explain="原版的 add_buffers 只遍历'单元的输出脚'；主输入不是单元，所以永远轮不到它。原仓库的 adder 能过，只是因为 a、b、c 恰好都有扇出，先被 splitter 变成了单元。")),
    dict(id="pointernal", title="输出兼作内部信号",
         desc="g1 的输出 p 既是主输出，又接到 g2。",
         src="""module pointernal(a, b, c, p, q); input a; input b; input c; output p; output q;
  and_bb g1 (.a(a), .b(b), .q(p));
  and_bb g2 (.a(p), .b(c), .q(q));
endmodule""",
         quiz=dict(q="p 这根线有几个负载？",
                   options=["1 个（g2）", "2 个（g2 和主输出 p）"], answer=1,
                   explain="主输出本身也是负载。原版统计扇出时只数了单元输入脚，于是 p 少了一个 splitter；它也不做输出对齐，p 和 q 在不同相位出来。")),
    dict(id="fanout5", title="扇出 5",
         desc="a、b 各驱动 5 个门。",
         src="module fanout5(a, b, q1, q2, q3, q4, q5); input a; input b; "
             "output q1; output q2; output q3; output q4; output q5;"
             + "".join(f"\n  and_bb g{i} (.a(a), .b(b), .q(q{i}));" for i in range(1, 6)) + "\nendmodule",
         quiz=dict(q="splitter 最多 4 路。新版给 a 搭的树最少几个单元？",
                   options=["1 个 spl4 + 1 个 bfr", "1 个 spl2 + 1 个 spl4", "2 个 spl4", "原版和新版都会报错"], answer=1,
                   explain="先 spl2 分两份：一份直接给 g1（于是 g1 在第 2 级），另一份再 spl4 给 g2~g5（第 3 级）。g1 早一级出来，再补 1 个 buffer 对齐输出。原版遇到扇出 >4 直接 ValueError。")),
]


def pack(nl, extra=None):
    cells = []
    for i in sorted(nl.insts.values(), key=lambda i: ((i.level or 0), i.name)):
        cd = LIB[i.cell]
        cells.append(dict(n=i.name, c=i.cell, r=cd.role, o=i.origin, L=i.level,
                          i={p: i.conns[p] for p in cd.inputs if p in i.conns},
                          q={p: i.conns[p] for p in cd.outputs if p in i.conns}))
    d = dict(pis=nl.pis, pos=nl.pos, alias=nl.po_drivers, cells=cells)
    if extra:
        d.update(extra)
    return d


def build():
    examples = []
    used = set()
    exs = list(EXAMPLES)
    try:
        from v2v2j.yosys_flow import rtl_to_netlist
        for f, t, dsc in [("c17.v", "ISCAS c17（Yosys 综合）", "6 个 NAND 的经典小电路，经 Yosys 综合、NOT 全部吸收进输入极性。"),
                          ("add4.v", "4 位加法器（Yosys 综合）", "assign {cout,s} = a + b + cin，39 个门。")]:
            path = os.path.join(ROOT, "bench", "rtl", f) if f == "c17.v" else os.path.join(ROOT, "examples", "rtl", f)
            nl = scalarize(rtl_to_netlist([path], LIB))
            exs.append(dict(id=f[:-2], title=t, desc=dsc, src=to_verilog(nl), quiz=None))
    except Exception as e:  # 没有 Yosys
        print("跳过 Yosys 示例：", e)
    for ex in exs:
        nl = parse_verilog(ex["src"], LIB)
        res = insert(nl)
        rep = verify(res.netlist, reference=nl)
        assert rep.ok, rep
        new = pack(res.netlist, dict(depth=res.schedule.depth, method=res.schedule.method,
                                     buffers=res.stats["buffers"], splitters=res.stats["splitters"], errors=[]))
        try:
            old_nl = legacy_to_netlist(run_legacy(LIB_PATH, ex["src"]), LIB)
            rep_old = verify(old_nl, reference=nl)
            roles = old_nl.count_by_role()
            old = pack(old_nl, dict(depth=max(i.level for i in old_nl.insts.values()),
                                    buffers=roles.get("buffer", 0), splitters=roles.get("splitter", 0),
                                    errors=rep_old.errors[:12]))
        except Exception as e:
            old = dict(crash=f"{type(e).__name__}: {e}")
        logic = pack(nl)
        examples.append(dict(id=ex["id"], title=ex["title"], desc=ex["desc"], quiz=ex["quiz"],
                             logic=logic, new=new, old=old))
        used |= {c["c"] for v in (logic, new, old) if "cells" in v for c in v["cells"]}
    used |= {n for n, c in LIB.cells.items() if c.role != "other"}
    lib = {}
    for name in sorted(used):
        cd = LIB[name]
        tabs = {o: list(cd.truth_table(o)) for o in cd.outputs}
        lib[name] = dict(i=cd.inputs, q=cd.outputs, r=cd.role, t=tabs)
    data = dict(lib=lib, examples=examples)
    with open(os.path.join(HERE, "demo_template.html"), encoding="utf-8") as f:
        html = f.read()
    html = html.replace("/*__DATA__*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    out = os.path.join(HERE, "aqfp_demo.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print("写出", out, f"{len(html) // 1024} KB，示例 {len(examples)} 个")


if __name__ == "__main__":
    build()
