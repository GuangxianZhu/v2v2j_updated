"""AQFP 单元库解析。

单元库是一份行为级 Verilog（例如 aqfp_hstp033 的 vlib）。我们从中取三样东西：
1. 端口顺序与方向（input / output / inout）——生成网表和 JoSIM 实例时要用；
2. 每个输出的布尔函数（取自 ``always @(gatex)`` 块里的赋值）——用于功能仿真与等价性检查；
3. 单元的"角色"：buffer / inverter / splitter / gate / other。角色由函数**推断**，不靠名字，
   因此换一套命名不同的库也能用（名字只在同类多选一时用来挑"最朴素"的那个）。

假设（失效条件）：
- 逻辑单元的行为写成 ``out = <表达式>;``，表达式只用 ~ ! & && | || ^ 和括号；
  写成 case/if 的单元会被识别为 role="other"，不参与插入，但仍可在网表里出现。
- inout 端口全部视为励磁/偏置线（xin/xout/din/dout 之类），不参与逻辑。
"""
from __future__ import annotations

import itertools
import re
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

_COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
_MODULE_RE = re.compile(r"\bmodule\s+(\w+)\s*\((.*?)\)\s*;(.*?)\bendmodule\b", re.S)
_DECL_RE = re.compile(r"\b(input|output|inout)\b\s*(?:wire|reg)?\s*(\[[^\]]*\])?\s*([^;]+);")
_ALWAYS_RE = re.compile(r"always\s*@\s*\([^)]*\)\s*(begin(.*?)\bend\b|([^;]+;))", re.S)
_ASSIGN_RE = re.compile(r"\b(\w+)\s*<?=\s*([^;]+);")


def _expr_to_python(expr: str, inputs: List[str]) -> Optional[str]:
    """把 Verilog 位运算表达式翻译成只含 0/1 整数运算的 Python 表达式。"""
    e = expr.strip()
    if re.search(r"[?:<>=+\-*/%]", e.replace("~", "")):
        return None
    for tok in re.findall(r"[A-Za-z_]\w*", e):
        if tok not in inputs:
            return None
    # 技巧：Python 的按位运算在最低位上与布尔运算一致（~0 = -1，最低位为 1），
    # 且 ~ > & > ^ > | 的优先级与 Verilog 相同。所以直接替换运算符、最后 &1 即可。
    # 局限：Verilog 的 && / || 优先级低于 | ，混写如 "a | b && c" 会被算错（库里没有这种写法）。
    if ("&&" in e or "||" in e) and re.search(r"(?<![&|])[&|^](?![&|])", e):
        return None  # 混写时宁可不认，也不要算错
    return e.replace("&&", "&").replace("||", "|").replace("!", "~")


@dataclass
class CellDef:
    name: str
    ports: List[str]                      # 端口声明顺序
    inputs: List[str]
    outputs: List[str]
    inouts: List[str]
    funcs: Dict[str, str] = field(default_factory=dict)   # 输出 -> python 表达式
    role: str = "other"                   # gate / buffer / inverter / splitter / other

    # ---- 求值 ----
    def _compiled(self) -> Dict[str, Callable]:
        if not hasattr(self, "_cache"):
            args = ",".join(self.inputs) or "_"
            self._cache = {o: eval(f"lambda {args}: ({f}) & 1") for o, f in self.funcs.items()}
        return self._cache

    def evaluate(self, values: Dict[str, int]) -> Dict[str, int]:
        """values: 输入端口 -> 0/1。返回 输出端口 -> 0/1。"""
        args = [values[i] for i in self.inputs]
        return {o: fn(*args) for o, fn in self._compiled().items()}

    def evaluate3(self, values: Dict[str, Optional[int]]) -> Dict[str, Optional[int]]:
        """三值求值：None 表示 X（未励磁/未知）。若结果与 X 的取值无关则给确定值。"""
        xs = [i for i in self.inputs if values.get(i) is None]
        if not xs:
            return self.evaluate(values)  # type: ignore[arg-type]
        outs: Dict[str, Optional[int]] = {}
        for assign in itertools.product((0, 1), repeat=len(xs)):
            v = dict(values)
            v.update(zip(xs, assign))
            r = self.evaluate(v)  # type: ignore[arg-type]
            for o, b in r.items():
                if o not in outs:
                    outs[o] = b
                elif outs[o] != b:
                    outs[o] = None
        return outs

    def truth_table(self, out: Optional[str] = None) -> Tuple[int, ...]:
        out = out or self.outputs[0]
        rows = []
        for bits in itertools.product((0, 1), repeat=len(self.inputs)):
            rows.append(self.evaluate(dict(zip(self.inputs, bits)))[out])
        return tuple(rows)


class Library:
    def __init__(self, cells: Dict[str, CellDef]):
        self.cells = cells
        self.lint: List[str] = []
        self._classify()

    # ------------------------------------------------------------------ 解析
    @classmethod
    def from_file(cls, path: str) -> "Library":
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return cls.from_text(f.read())

    @classmethod
    def from_text(cls, text: str) -> "Library":
        text = _COMMENT_RE.sub("", text)
        cells: Dict[str, CellDef] = {}
        lint: List[str] = []
        for name, ports, body in _MODULE_RE.findall(text):
            port_list = [p.strip() for p in ports.replace("\n", " ").split(",") if p.strip()]
            port_list = [p.split()[-1] for p in port_list]  # 兼容 ANSI 写法 "input a"
            d = CellDef(name, port_list, [], [], [])
            for kind, _rng, names in _DECL_RE.findall(body):
                lst = {"input": d.inputs, "output": d.outputs, "inout": d.inouts}[kind]
                lst.extend(n.strip() for n in names.split(",") if n.strip())
            stray = []
            for m in _ALWAYS_RE.finditer(body):
                block = m.group(2) if m.group(2) is not None else m.group(3)
                for lhs, rhs in _ASSIGN_RE.findall(block):
                    if lhs in d.outputs:
                        py = _expr_to_python(rhs, d.inputs)
                        if py is not None:
                            d.funcs[lhs] = py
                    elif not re.search(rf"\b(reg|wire)\b[^;]*\b{lhs}\b", body) and lhs not in d.inouts:
                        stray.append(lhs)
            if stray and d.outputs and set(d.funcs) != set(d.outputs):
                missing = [o for o in d.outputs if o not in d.funcs]
                lint.append(f"单元 {name}：输出 {missing} 从未被赋值，却给未声明的 {sorted(set(stray))} 赋了值"
                            f"（疑似笔误）；该单元将不参与功能仿真")
            cells[name] = d
        lib = cls(cells)
        lib.lint = lint
        return lib

    # ------------------------------------------------------------------ 分类
    def _classify(self) -> None:
        self.buffers: List[str] = []
        self.inverters: List[str] = []
        self.splitters: Dict[int, List[str]] = {}
        self.gates: List[str] = []
        for c in self.cells.values():
            if not c.inputs or not c.outputs or set(c.funcs) != set(c.outputs):
                c.role = "other"
                continue
            try:
                tts = [c.truth_table(o) for o in c.outputs]
            except Exception:  # 表达式翻译出错
                c.role = "other"
                continue
            if len(c.inputs) == 1 and all(t == (0, 1) for t in tts):
                if len(c.outputs) == 1:
                    c.role = "buffer"
                    self.buffers.append(c.name)
                else:
                    c.role = "splitter"
                    self.splitters.setdefault(len(c.outputs), []).append(c.name)
            elif len(c.inputs) == 1 and len(c.outputs) == 1 and tts[0] == (1, 0):
                c.role = "inverter"
                self.inverters.append(c.name)
            elif len(c.outputs) == 1:
                c.role = "gate"
                self.gates.append(c.name)
            else:
                c.role = "other"
        # 同类多选一：名字最短者优先（bfr 优先于 bfrL，spl3 优先于 spl3L）
        self.buffers.sort(key=lambda n: (len(n), n))
        self.inverters.sort(key=lambda n: (len(n), n))
        for k in self.splitters:
            self.splitters[k].sort(key=lambda n: (len(n), n))

    # ------------------------------------------------------------------ 查询
    def __contains__(self, name: str) -> bool:
        return name in self.cells

    def __getitem__(self, name: str) -> CellDef:
        return self.cells[name]

    @property
    def buffer(self) -> str:
        if not self.buffers:
            raise ValueError("单元库里没有找到 buffer（1 入 1 出、q=a 的单元）")
        return self.buffers[0]

    def splitter(self, fanout: int) -> str:
        if fanout == 1:
            return self.buffer
        if fanout not in self.splitters:
            raise ValueError(f"单元库里没有 {fanout} 路 splitter")
        return self.splitters[fanout][0]

    def max_splitter_fanout(self, limit: Optional[int] = None) -> int:
        """最大可用扇出 k：要求 2..k 全都存在（否则贪心建树时某些分配做不出来）。"""
        k = 1
        while (k + 1) in self.splitters and (limit is None or k + 1 <= limit):
            k += 1
        if k < 2:
            raise ValueError("单元库里至少需要一个 2 路 splitter")
        return k

    def find_gate(self, table: Tuple[int, ...], n_inputs: int) -> Optional[Tuple[str, List[str]]]:
        """找一个真值表等于 table 的单元（允许输入端口重新排列）。
        table 按 itertools.product((0,1), repeat=n) 的顺序给出（第 0 个输入为最高位）。
        返回 (单元名, 端口顺序)：端口顺序[i] 接第 i 个逻辑输入。"""
        cands = sorted(self.gates + self.inverters + self.buffers, key=lambda n: (len(n), n))
        for name in cands:
            c = self.cells[name]
            if len(c.inputs) != n_inputs:
                continue
            for perm in itertools.permutations(c.inputs):
                rows = []
                for bits in itertools.product((0, 1), repeat=n_inputs):
                    rows.append(c.evaluate(dict(zip(perm, bits)))[c.outputs[0]])
                if tuple(rows) == tuple(table):
                    return name, list(perm)
        return None

    def summary(self) -> str:
        spl = ", ".join(f"{k}路:{v[0]}" for k, v in sorted(self.splitters.items()))
        return (f"逻辑门 {len(self.gates)} 个；buffer={self.buffers[:1]}；inverter={self.inverters[:1]}；"
                f"splitter {spl}；其他(偏置/连线等) "
                f"{sum(1 for c in self.cells.values() if c.role == 'other')} 个")
