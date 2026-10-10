"""把原版算法的问题固定成测试：这些测试断言"原版确实会错"。
若有一天它们失败了，说明原版被改动了——legacy 应保持原样作为对照。"""
import os

import pytest

from v2v2j.legacy import legacy_to_netlist, run_legacy
from v2v2j.library import Library
from v2v2j.sim import stream_check
from v2v2j.verify import verify
from v2v2j.verilog_parser import parse_verilog

from .test_core import CASES, LIB, ROOT

LIB_PATH = os.path.join(ROOT, "lib", "aqfp_hstp033_vlib.v")


def _legacy(name):
    data = run_legacy(LIB_PATH, CASES[name])
    return legacy_to_netlist(data, LIB), parse_verilog(CASES[name], LIB)


def test_legacy_ok_on_its_own_example():
    old, ref = _legacy("adder")
    assert verify(old, reference=ref).ok


@pytest.mark.parametrize("name", ["reconvergent", "pi_to_deep_gate"])
def test_legacy_phase_violation(name):
    old, ref = _legacy(name)
    rep = verify(old, reference=ref)
    assert any("相位不一致" in e for e in rep.errors)
    wrong, xs, total = stream_check(old, ref, n=16, po_level=max(i.level for i in old.insts.values()))
    assert wrong + xs > 0


def test_legacy_misses_po_fanout():
    old, ref = _legacy("po_also_internal")
    rep = verify(old, reference=ref)
    assert any("扇出违规" in e for e in rep.errors)
    assert any("不对齐" in e for e in rep.errors)


def test_legacy_crashes_on_fanout_over_4():
    with pytest.raises(ValueError):
        _legacy("fanout9")


def test_legacy_misparses_buses():
    with pytest.raises(Exception):
        old, ref = _legacy("bus_and_assign")
        assert verify(old, reference=ref).ok
