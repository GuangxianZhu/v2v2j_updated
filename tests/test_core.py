"""核心功能测试：解析、插入、校验、仿真。运行：python -m pytest -q"""
import os
import random

import pytest

from v2v2j import trees
from v2v2j.insert import insert
from v2v2j.library import Library
from v2v2j.schedule import Config
from v2v2j.sim import comb_eval, stream_check
from v2v2j.verify import verify
from v2v2j.verilog_parser import ParseError, parse_verilog
from v2v2j.writers import to_json, to_verilog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = Library.from_file(os.path.join(ROOT, "lib", "aqfp_hstp033_vlib.v"))

# 原版失败的 6 类情形 + 自带的加法器
CASES = {
    "adder": open(os.path.join(ROOT, "examples", "adder_mapped.v")).read(),
    "reconvergent": """module t(a,b,q); input a; input b; output q; wire w;
        and_bb g1 (.a(a), .b(w), .q(q));
        and_bb g2 (.a(a), .b(b), .q(w));
        endmodule""",
    "pi_to_deep_gate": """module t(a,b,c,q); input a; input b; input c; output q; wire w;
        and_bb g1 (.a(a), .b(b), .q(w));
        and_bb g2 (.a(w), .b(c), .q(q));
        endmodule""",
    "po_also_internal": """module t(a,b,c,p,q); input a; input b; input c; output p; output q;
        and_bb g1 (.a(a), .b(b), .q(p));
        and_bb g2 (.a(p), .b(c), .q(q));
        endmodule""",
    "fanout9": "module t(a,b," + ",".join(f"q{i}" for i in range(9)) + "); input a; input b; "
               + " ".join(f"output q{i};" for i in range(9))
               + "".join(f" or_bi g{i} (.a(a), .b(b), .q(q{i}));" for i in range(9)) + " endmodule",
    "bus_and_assign": """module t(input [2:0] a, output [1:0] y, output z);
        wire w;
        maj_bbb m (.a(a[0]), .b(a[1]), .c(a[2]), .q(w));
        assign y = {w, a[0]};
        inv i0 (.a(w), .q(z));
        endmodule""",
}


@pytest.mark.parametrize("name", sorted(CASES))
@pytest.mark.parametrize("method", ["asap", "alap", "lp", "best"])
def test_insert_is_correct(name, method):
    nl = parse_verilog(CASES[name], LIB)
    res = insert(nl, Config(method=method))
    rep = verify(res.netlist, reference=nl)
    assert rep.ok, str(rep)
    wrong, xs, total = stream_check(res.netlist, nl, n=24, po_level=res.schedule.depth)
    assert wrong == 0 and xs == 0 and total > 0


@pytest.mark.parametrize("k", [2, 3, 4])
def test_max_fanout_respected(k):
    nl = parse_verilog(CASES["fanout9"], LIB)
    res = insert(nl, Config(max_fanout=k))
    assert verify(res.netlist, reference=nl).ok
    for inst in res.netlist.insts.values():
        assert len(LIB[inst.cell].outputs) <= k


def test_free_pis_and_pos():
    nl = parse_verilog(CASES["pi_to_deep_gate"], LIB)
    res = insert(nl, Config(balance_pis=False, balance_pos=False))
    assert verify(res.netlist, reference=nl, balance_pis=False, balance_pos=False).ok
    assert res.stats["buffers"] == 0  # 主输入可以晚到，就不需要 buffer


def test_buffer_sharing_beats_legacy_on_adder():
    nl = parse_verilog(CASES["adder"], LIB)
    res = insert(nl)
    assert res.stats["depth"] == 8
    assert res.stats["buffers"] + res.stats["splitters"] <= 16  # 原版 20


def test_reinsert_is_idempotent():
    nl = parse_verilog(CASES["adder"], LIB)
    once = insert(nl).netlist
    again = insert(once)
    assert any("已有" in w for w in again.warnings)
    assert again.stats["buffers"] + again.stats["splitters"] <= 16


def test_verilog_roundtrip():
    nl = parse_verilog(CASES["bus_and_assign"], LIB)
    res = insert(nl)
    text = to_verilog(res.netlist)
    back = parse_verilog(text, LIB)
    assert verify(back, reference=nl).ok
    assert '"cells"' in to_json(res.netlist)


def test_verify_catches_violations():
    bad = parse_verilog(CASES["reconvergent"], LIB)  # 未插入的网表：扇出与相位都违规
    rep = verify(bad)
    assert not rep.ok
    assert any("扇出" in e for e in rep.errors)
    assert any("相位" in e for e in rep.errors)


def test_parser_rejects_behavioral():
    with pytest.raises(ParseError):
        parse_verilog("module m(input a, output y); always @(*) y = a; endmodule", LIB)


def test_library_detects_typo_in_original():
    old = Library.from_file(os.path.join(ROOT, "legacy", "vlib.txt"))
    assert any("or_bb" in w for w in old.lint)
    assert LIB["or_bb"].role == "gate"


# ---------------------------------------------------------------- 树的最优性
def _brute_min_cost(reqs, k, root):
    """小规模穷举：按级 BFS 搜所有分配方式，验证贪心给出的是最少单元数。"""
    import functools
    hi = max(reqs)
    need = [reqs.count(t) for t in range(hi + 1)]

    @functools.lru_cache(None)
    def best(t, copies):
        # 第 t 级有 copies 份拷贝；先满足本级负载，剩下的往下送
        rest = copies - need[t]
        if rest < 0:
            return float("inf")
        if t == hi:
            return 0 if rest == 0 else float("inf")
        if rest == 0:
            return float("inf") if any(need[t + 1:]) else 0
        out = float("inf")
        # rest 个单元，每个产出 1..k 份
        for total in range(rest, rest * k + 1):
            out = min(out, rest + best(t + 1, total))
        return out

    return best(root, 1)


@pytest.mark.parametrize("seed", range(40))
def test_tree_greedy_is_optimal(seed):
    rng = random.Random(seed)
    k = rng.choice([2, 3, 4])
    n = rng.randint(1, 7)
    root = 0
    reqs = [rng.randint(trees.splitter_depth(n, k), trees.splitter_depth(n, k) + 3) for _ in range(n)]
    if n == 1:
        reqs = [rng.randint(0, 3)]
    _r, _c, cost = trees.plan(reqs, k, root)
    assert cost == _brute_min_cost(reqs, k, root)


def test_kraft_feasibility():
    # 5 个负载、k=4：3 个在深度 1、2 个在深度 2 —— Σ 4^-d = 3/4 + 2/16 ≤ 1，可行
    assert trees.cost([1, 1, 1, 2, 2], 4, 0) >= 0
    # 5 个都在深度 1 —— 5/4 > 1，不可行
    assert trees.cost([1, 1, 1, 1, 1], 4, 0) < 0


def test_comb_eval_matches_library_functions():
    nl = parse_verilog(CASES["bus_and_assign"], LIB)
    for a in range(8):
        v = {f"a[{i}]": (a >> i) & 1 for i in range(3)}
        out = comb_eval(nl, v)
        maj = int(bin(a).count("1") >= 2)
        assert out == {"y[0]": v["a[0]"], "y[1]": maj, "z": 1 - maj}
