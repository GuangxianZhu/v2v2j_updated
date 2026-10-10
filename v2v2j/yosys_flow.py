"""RTL → AQFP 门级网表：调用 Yosys 综合成 AND/OR/NOT，再映射到 AQFP 单元。

为什么不直接用 Yosys + liberty 文件映射？AQFP 的反相几乎是免费的（输入变压器反接即可），
所以单元库里每种门都有 4 个"输入极性"版本（and_bb / and_bi / and_ib / and_ii）。
通用的映射器不理解这一点，会生成很多独立的 inv 单元；而 inv 和 buffer 一样要占一级相位。
这里自己做映射：
1. 把 NOT 链折叠成"字面量"（信号 + 极性）；
2. 门输入上的反相 → 选对应极性的单元版本；
3. 若一个门的输出被所有使用者都取反 → 用德摩根律换成对偶门（AND↔OR）并翻转输入极性；
4. 只有"取反后直接送到主输出"且无法吸收时才放一个 inv 单元。

Yosys 的来源：优先用系统里的 yosys；没有的话用 pip 包 yowasp-yosys（WebAssembly 版，Windows 也能装）。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from typing import Dict, List, Optional, Tuple

from .library import Library
from .netlist import CONST0, CONST1, Inst, Netlist, Port


class YosysError(RuntimeError):
    pass


def find_yosys() -> List[str]:
    for exe in ("yosys", "yowasp-yosys"):
        p = shutil.which(exe)
        if p:
            return [p]
    try:  # 作为 Python 模块调用（pip 装了但 Scripts 目录不在 PATH 时）
        import yowasp_yosys  # noqa: F401
        import sys
        return [sys.executable, "-m", "yowasp_yosys"]
    except ImportError:
        pass
    raise YosysError("找不到 Yosys。请安装：pip install yowasp-yosys（或系统级 yosys）")


def synthesize(rtl_files: List[str], top: Optional[str] = None, gates: str = "AND,OR",
               extra: str = "") -> dict:
    """返回 Yosys write_json 的结果。gates 传给 abc -g（如 "AND,OR" 或 "aig"）。"""
    cmd = find_yosys()
    # yowasp-yosys 是 WebAssembly 程序，只能看到当前目录；所以把源文件拷进临时目录、用相对路径
    with tempfile.TemporaryDirectory() as td:
        local = []
        for i, f in enumerate(rtl_files):
            dst = f"src{i}_{os.path.basename(f)}"
            shutil.copy(f, os.path.join(td, dst))
            local.append(dst)
        out = os.path.join(td, "out.json")
        reads = "; ".join(f"read_verilog -sv {f}" for f in local)
        topcmd = f"-top {top}" if top else "-auto-top"
        script = (f"{reads}; hierarchy {topcmd}; synth -flatten {('-top ' + top) if top else ''}; "
                  f"{extra} abc -g {gates}; opt_clean -purge; write_json out.json")
        r = subprocess.run(cmd + ["-q", "-p", script], capture_output=True, text=True, cwd=td)
        if r.returncode != 0 or not os.path.exists(out):
            raise YosysError(f"Yosys 失败：\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        with open(out, encoding="utf-8") as f:
            return json.load(f)


def _bit_names(mod: dict) -> Dict[object, str]:
    names: Dict[object, Tuple[int, str]] = {}

    def offer(bit, name, prio):
        if isinstance(bit, str):
            return
        if bit not in names or prio > names[bit][0]:
            names[bit] = (prio, name)

    for pname, p in mod["ports"].items():
        bits = p["bits"]
        for i, b in enumerate(bits):
            offer(b, pname if len(bits) == 1 else f"{pname}[{i}]", 3)
    for nname, n in mod.get("netnames", {}).items():
        if n.get("hide_name"):
            continue
        bits = n["bits"]
        off = n.get("offset", 0)
        for i, b in enumerate(bits):
            offer(b, nname if len(bits) == 1 else f"{nname}[{i + off}]", 1)
    out = {b: nm for b, (_p, nm) in names.items()}
    return out


def json_to_netlist(data: dict, lib: Library, top: Optional[str] = None) -> Netlist:
    mods = data["modules"]
    if top is None:
        tops = [n for n, m in mods.items() if m.get("attributes", {}).get("top")]
        top = tops[0] if tops else list(mods)[-1]
    mod = mods[top]
    bname = _bit_names(mod)

    def net(bit) -> str:
        if bit in ("0", 0) and isinstance(bit, str):
            return CONST0
        if bit == "0":
            return CONST0
        if bit == "1":
            return CONST1
        if isinstance(bit, str):  # "x" / "z"
            return CONST0
        return bname.get(bit, f"n{bit}")

    nl = Netlist(top, lib)
    for pname, p in mod["ports"].items():
        if p["direction"] not in ("input", "output"):
            raise YosysError(f"端口 {pname} 是 {p['direction']}，不支持")
        bits = [net(b) if not isinstance(b, str) else None for b in p["bits"]]
        if p["direction"] == "input" and None in bits:
            raise YosysError(f"输入端口 {pname} 有常量位？")
        names = [pname if len(p["bits"]) == 1 else f"{pname}[{i}]" for i in range(len(p["bits"]))]
        nl.ports.append(Port(pname, p["direction"], names, len(names) - 1, 0))

    # ---- 1. 读入通用门 ----
    nots: Dict[object, object] = {}      # Y bit -> A bit
    gates: Dict[str, Tuple[str, object, object, object]] = {}  # name -> (AND/OR, A, B, Y)
    for cname, c in mod["cells"].items():
        t, con = c["type"], c["connections"]
        if t in ("$_NOT_",):
            nots[con["Y"][0]] = con["A"][0]
        elif t == "$_BUF_":
            nots[con["Y"][0]] = ("BUF", con["A"][0])
        elif t in ("$_AND_", "$_OR_"):
            gates[cname] = (t[2:-1], con["A"][0], con["B"][0], con["Y"][0])
        else:
            raise YosysError(f"综合结果里有未支持的单元 {t}（请用 abc -g AND,OR）")

    def lit(bit) -> Tuple[object, int]:
        inv = 0
        seen = 0
        while bit in nots and seen < 10000:
            src = nots[bit]
            if isinstance(src, tuple):
                bit = src[1]
            else:
                bit = src
                inv ^= 1
            seen += 1
        if bit in ("0", "1"):
            return ("1" if (bit == "1") ^ inv else "0"), 0
        if isinstance(bit, str):
            return "0", 0
        return bit, inv

    # ---- 2. 统计每个门输出的使用极性 ----
    gate_of_y = {g[3]: name for name, g in gates.items()}
    uses: Dict[object, List[int]] = {}
    for kind, a, b, y in gates.values():
        for x in (a, b):
            s, inv = lit(x)
            uses.setdefault(s, []).append(inv)
    po_bits = []
    for pname, p in mod["ports"].items():
        if p["direction"] == "output":
            for i, b in enumerate(p["bits"]):
                nm = pname if len(p["bits"]) == 1 else f"{pname}[{i}]"
                s, inv = lit(b)
                po_bits.append((nm, s, inv))
                uses.setdefault(s, []).append(inv)

    flipped = set()
    for y, name in gate_of_y.items():
        u = uses.get(y, [])
        if u and all(u):
            flipped.add(y)

    def eff(bit) -> Tuple[object, int]:
        s, inv = lit(bit)
        if s in flipped:
            inv ^= 1
        return s, inv

    # ---- 3. 选单元 ----
    used_names = set(nl.pis) | set(nl.pos)
    for name, (kind, a, b, y) in gates.items():
        if y in flipped:
            kind = "OR" if kind == "AND" else "AND"
            # 德摩根：~(A&B) = ~A | ~B，输入极性随之翻转
            la, ia = lit(a)
            lb, ib = lit(b)
            ia ^= 1
            ib ^= 1
            if la in flipped:
                ia ^= 1
            if lb in flipped:
                ib ^= 1
        else:
            (la, ia), (lb, ib) = eff(a), eff(b)
        table = []
        for x0 in (0, 1):
            for x1 in (0, 1):
                v0, v1 = x0 ^ ia, x1 ^ ib
                table.append(int((v0 & v1) if kind == "AND" else (v0 | v1)))
        found = lib.find_gate(tuple(table), 2)
        if not found:
            raise YosysError(f"单元库里没有实现 {kind} 带极性 ({ia},{ib}) 的单元")
        cell, perm = found
        ins = [net(la), net(lb)]
        conns = {perm[0]: ins[0], perm[1]: ins[1], lib[cell].outputs[0]: net(y)}
        iname = f"g{len(nl.insts)}"
        nl.add(Inst(iname, cell, conns))
        used_names.add(iname)

    # ---- 4. 主输出 ----
    for po, s, inv in po_bits:
        if s in flipped:
            inv ^= 1
        src = net(s)
        if inv and src in (CONST0, CONST1):
            src, inv = (CONST1 if src == CONST0 else CONST0), 0
        if inv:
            if not lib.inverters:
                raise YosysError("需要反相器单元但库里没有")
            cell = lib.inverters[0]
            cd = lib[cell]
            nl.add(Inst(f"inv_{len(nl.insts)}", cell, {cd.inputs[0]: src, cd.outputs[0]: po}))
        elif src != po:
            nl.po_drivers[po] = src
    return nl


def rtl_to_netlist(rtl_files: List[str], lib: Library, top: Optional[str] = None,
                   gates: str = "AND,OR") -> Netlist:
    return json_to_netlist(synthesize(rtl_files, top, gates), lib, top)
