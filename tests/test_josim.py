"""JoSIM 电路级仿真测试（需要 josim-cli；找不到就跳过）。

用 JoSIM 仓库自带的 π-DQFP buffer/inverter 模型，把 y = ~a 插成 2..9 级流水线，
每个时钟周期换一组输入，检查每个周期的输出都正确——验证的是励磁接线、相位方案和采样时刻。
"""
import json
import os
import random
import shutil
import subprocess

import pytest

from v2v2j.insert import insert
from v2v2j.josim import JosimConfig, read_results, to_josim
from v2v2j.schedule import Config
from v2v2j.verilog_parser import parse_verilog

from .test_core import LIB, ROOT

JOSIM = os.environ.get("JOSIM") or shutil.which("josim-cli")
pytestmark = pytest.mark.skipif(not JOSIM, reason="没有 josim-cli（可设环境变量 JOSIM 指向它）")

INV = "module inv_pipe(input a, output y); inv u (.a(a), .q(y)); endmodule"


def _run(nl, tmp_path, stream):
    cm = json.load(open(os.path.join(ROOT, "josim", "demo_cellmap.json"), encoding="utf-8"))
    cm = {k: v for k, v in cm.items() if not k.startswith("_")}
    cfg = JosimConfig(include=os.path.join(ROOT, "josim", "demo_cells_pi_dqfp.cir"), cellmap=cm)
    txt, tim = to_josim(nl, stream, cfg)
    cir, csv = tmp_path / "t.cir", tmp_path / "t.csv"
    cir.write_text(txt)
    subprocess.run([JOSIM, "-m", "0", str(cir), "-o", str(csv)], check=True, capture_output=True)
    return [r["y"] for r in read_results(str(csv), tim)]


@pytest.mark.parametrize("depth", [2, 3, 4, 5, 6, 9])
def test_inverter_pipeline(depth, tmp_path):
    nl = parse_verilog(INV, LIB)
    res = insert(nl, Config(depth=depth, method="asap"))
    rng = random.Random(depth)
    stream = [{"a": rng.getrandbits(1)} for _ in range(10)]
    assert _run(res.netlist, tmp_path, stream) == [1 - v["a"] for v in stream]


def test_wrong_phase_is_visible(tmp_path):
    """负对照：把两个相邻 buffer 的级数对调（相位接反），电路级仿真应当出错。"""
    nl = parse_verilog(INV, LIB)
    res = insert(nl, Config(depth=6, method="asap"))
    by_level = {i.level: i for i in res.netlist.insts.values()}
    by_level[3].level, by_level[4].level = 4, 3
    rng = random.Random(0)
    stream = [{"a": rng.getrandbits(1)} for _ in range(10)]
    assert _run(res.netlist, tmp_path, stream) != [1 - v["a"] for v in stream]
