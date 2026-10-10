"""结构化（门级）Verilog 解析器。

原版用正则 ``(\\w+) (\\w+?) \\((.*?)\\);`` 抓实例，遇到下面这些 Yosys 常见写法就会出错或静默出错：
总线 ``input [3:0] a;`` 与位选 ``a[2]``、转义标识符 ``\\foo[0] ``、``assign``、常量 ``1'b0``、
拼接 ``{a, b}``、ANSI 端口头 ``module m(input [1:0] a, output y);``、注释与属性 ``(* keep *)``。
这里改为"分词 + 递归下降"，只支持门级子集：模块 / 端口 / wire / 单元实例 / assign。
行为级语句（always、运算符表达式）不支持——那种输入请走 yosys_flow 先综合。
"""
from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple

from .library import Library
from .netlist import CONST0, CONST1, Inst, Netlist, NetlistError, Port

_TOKEN_RE = re.compile(
    r"""
    (?P<ws>\s+)
  | (?P<comment>//[^\n]*|/\*.*?\*/)
  | (?P<attr>\(\*.*?\*\))
  | (?P<escid>\\\S+)
  | (?P<num>\d*'[sS]?[bBoOdDhH][0-9a-fA-FxXzZ_?]+|\d+)
  | (?P<id>[A-Za-z_$][\w$]*)
  | (?P<punct>[()\[\]{}:;,.#=])
    """,
    re.S | re.X,
)


class ParseError(NetlistError):
    pass


def tokenize(text: str) -> List[str]:
    toks: List[str] = []
    pos = 0
    while pos < len(text):
        m = _TOKEN_RE.match(text, pos)
        if not m:
            line = text.count("\n", 0, pos) + 1
            raise ParseError(f"第 {line} 行无法识别的字符：{text[pos:pos+20]!r}")
        pos = m.end()
        kind = m.lastgroup
        if kind in ("ws", "comment", "attr"):
            continue
        tok = m.group()
        if kind == "escid":
            tok = tok[1:]  # 去掉反斜杠；名字本身保留原样
        toks.append(tok)
    return toks


def _const_bits(tok: str) -> List[str]:
    """把 4'b1010 / 1'b0 / 3 之类展开成位列表（LSB 在前）。x/z 当 0 处理并给警告者自行判断。"""
    if "'" not in tok:
        v, width = int(tok), 32
        return [CONST1 if (v >> i) & 1 else CONST0 for i in range(width)]
    size, rest = tok.split("'", 1)
    rest = rest.lstrip("sS")
    base, digits = rest[0].lower(), rest[1:].replace("_", "").lower()
    width = int(size) if size else 32
    if any(c in "xz?" for c in digits):
        bits = [CONST0] * width
        return bits
    v = int(digits, {"b": 2, "o": 8, "d": 10, "h": 16}[base])
    return [CONST1 if (v >> i) & 1 else CONST0 for i in range(width)]


class _Parser:
    def __init__(self, toks: List[str]):
        self.t = toks
        self.i = 0

    def peek(self, k: int = 0) -> Optional[str]:
        j = self.i + k
        return self.t[j] if j < len(self.t) else None

    def take(self, expect: Optional[str] = None) -> str:
        tok = self.peek()
        if tok is None:
            raise ParseError(f"文件意外结束（期望 {expect}）")
        if expect is not None and tok != expect:
            ctx = " ".join(self.t[max(0, self.i - 5): self.i + 5])
            raise ParseError(f"期望 {expect!r}，得到 {tok!r}，附近：{ctx}")
        self.i += 1
        return tok

    def skip_until(self, tok: str) -> None:
        while self.take() != tok:
            pass

    def skip_balanced(self, open_: str, close: str) -> None:
        depth = 0
        while True:
            t = self.take()
            if t == open_:
                depth += 1
            elif t == close:
                depth -= 1
                if depth == 0:
                    return


class Module:
    def __init__(self, name: str):
        self.name = name
        self.port_order: List[str] = []
        self.decl: Dict[str, Tuple[str, int, int]] = {}      # 名 -> (input/output/wire, msb, lsb)
        self.instances: List[Tuple[str, str, List, Dict[str, List[str]]]] = []
        self.assigns: List[Tuple[List[str], List[str]]] = []


def _range(p: _Parser) -> Tuple[int, int]:
    if p.peek() != "[":
        return 0, 0
    p.take("[")
    msb = int(p.take())
    p.take(":")
    lsb = int(p.take())
    p.take("]")
    return msb, lsb


def _bits_of(mod: Module, name: str) -> List[str]:
    d = mod.decl.get(name)
    if d is None or not _is_bus(mod, name):
        return [name]  # 标量（含隐式声明的 1 位网线）
    _, msb, lsb = d
    step = 1 if msb >= lsb else -1
    return [f"{name}[{i}]" for i in range(lsb, msb + step, step)]  # LSB 在前


def _is_bus(mod: Module, name: str) -> bool:
    return getattr(mod, "_bus", {}).get(name, False)


def _expr(p: _Parser, mod: Module) -> List[str]:
    """解析连接表达式，返回位列表（LSB 在前）。"""
    tok = p.peek()
    if tok == "{":
        p.take("{")
        parts: List[List[str]] = []
        while True:
            if p.peek() and re.fullmatch(r"\d+", p.peek()) and p.peek(1) == "{":  # 复制 {n{...}}
                n = int(p.take())
                p.take("{")
                inner = _expr(p, mod)
                p.take("}")
                parts.append(inner * n)
            else:
                parts.append(_expr(p, mod))
            if p.peek() == ",":
                p.take(",")
                continue
            break
        p.take("}")
        bits: List[str] = []
        for part in reversed(parts):  # 拼接里最左边是 MSB
            bits.extend(part)
        return bits
    tok = p.take()
    if re.match(r"^\d", tok):
        bits = _const_bits(tok)
        return bits
    name = tok
    if p.peek() == "[":
        p.take("[")
        hi = int(p.take())
        if p.peek() == ":":
            p.take(":")
            lo = int(p.take())
            p.take("]")
            step = 1 if hi >= lo else -1
            return [f"{name}[{i}]" for i in range(lo, hi + step, step)] if step == 1 else \
                   [f"{name}[{i}]" for i in range(lo, hi - 1, -1)]
        p.take("]")
        return [f"{name}[{hi}]"]
    return _bits_of(mod, name)


def _parse_module(p: _Parser) -> Module:
    p.take("module")
    mod = Module(p.take())
    mod._bus = {}  # type: ignore[attr-defined]

    def declare(kind: str, rng: Tuple[int, int], names: List[str], explicit_range: bool):
        for n in names:
            if kind in ("input", "output"):
                if n not in mod.port_order:
                    mod.port_order.append(n)
            prev = mod.decl.get(n)
            if prev and prev[0] in ("input", "output") and kind == "wire":
                continue  # "output y; wire y;" 这种重复声明
            mod.decl[n] = (kind, rng[0], rng[1])
            if explicit_range:
                mod._bus[n] = True  # type: ignore[attr-defined]

    if p.peek() == "#":
        p.take("#")
        p.skip_balanced("(", ")")
    if p.peek() == "(":
        p.take("(")
        kind = None
        while p.peek() != ")":
            tok = p.take()
            if tok in ("input", "output", "inout"):
                kind = tok
                if p.peek() in ("wire", "reg", "logic"):
                    p.take()
                explicit = p.peek() == "["
                rng = _range(p)
                name = p.take()
                declare(kind, rng, [name], explicit)
            elif tok == ",":
                continue
            elif kind is not None and tok not in ("wire", "reg"):
                # ANSI 头中省略方向的后续名字："input a, b"
                prev = mod.decl[mod.port_order[-1]]
                declare(kind, (prev[1], prev[2]), [tok], _is_bus(mod, mod.port_order[-1]))
            elif tok not in ("wire", "reg"):
                mod.port_order.append(tok)
        p.take(")")
    p.take(";")

    while p.peek() != "endmodule":
        tok = p.peek()
        if tok in ("input", "output", "inout", "wire", "reg", "supply0", "supply1", "tri"):
            kind = p.take()
            if kind == "inout":
                raise ParseError(f"模块 {mod.name} 有 inout 端口，AQFP 网表不支持双向端口")
            if p.peek() in ("wire", "reg", "logic", "signed"):
                p.take()
            explicit = p.peek() == "["
            rng = _range(p)
            names = []
            while True:
                names.append(p.take())
                if p.peek() == "=":  # wire x = y;  视为 assign
                    p.take("=")
                    rhs = _expr(p, mod)
                    declare("wire" if kind in ("wire", "reg") else kind, rng, [names[-1]], explicit)
                    mod.assigns.append((_bits_of(mod, names[-1]), rhs))
                if p.peek() == ",":
                    p.take(",")
                    continue
                break
            p.take(";")
            declare("wire" if kind in ("wire", "reg", "tri") else kind, rng, names, explicit)
        elif tok == "assign":
            p.take("assign")
            while True:
                lhs = _expr(p, mod)
                p.take("=")
                rhs = _expr(p, mod)
                mod.assigns.append((lhs, rhs))
                if p.peek() == ",":
                    p.take(",")
                    continue
                break
            p.take(";")
        elif tok in ("parameter", "localparam", "defparam", "initial", "always", "function", "task",
                     "generate", "specify"):
            raise ParseError(f"模块 {mod.name} 含行为级语句 `{tok}`；请先用 Yosys 综合（v2v2j 的 --rtl 选项）")
        else:
            cell = p.take()
            if p.peek() == "#":
                p.take("#")
                p.skip_balanced("(", ")")
            inst_name = p.take()
            p.take("(")
            named: Dict[str, List[str]] = {}
            positional: List[List[str]] = []
            while p.peek() != ")":
                if p.peek() == ".":
                    p.take(".")
                    pin = p.take()
                    p.take("(")
                    if p.peek() == ")":
                        named[pin] = []
                    else:
                        named[pin] = _expr(p, mod)
                    p.take(")")
                elif p.peek() == ",":
                    p.take(",")
                else:
                    positional.append(_expr(p, mod))
            p.take(")")
            p.take(";")
            mod.instances.append((cell, inst_name, positional, named))
    p.take("endmodule")
    return mod


def parse_modules(text: str) -> List[Module]:
    p = _Parser(tokenize(text))
    mods = []
    while p.peek() is not None:
        if p.peek() == "module":
            mods.append(_parse_module(p))
        else:
            p.take()  # `timescale 等编译指令的残余
    return mods


def parse_verilog(text: str, lib: Library, top: Optional[str] = None) -> Netlist:
    """解析门级 Verilog，返回位级 Netlist。

    - 多个模块时，取未被其他模块实例化的那个为顶层（或用 top 指定）。
      子模块实例目前不展开（门级综合结果一般已 flatten）。
    - assign 视为导线别名；``assign po = 1'b0`` 记为常量输出。
    """
    mods = parse_modules(text)
    if not mods:
        raise ParseError("没有找到 module")
    by_name = {m.name: m for m in mods}
    if top is None:
        used = {inst[0] for m in mods for inst in m.instances}
        tops = [m for m in mods if m.name not in used]
        top_mod = tops[-1] if tops else mods[-1]
    else:
        if top not in by_name:
            raise ParseError(f"找不到顶层模块 {top}")
        top_mod = by_name[top]

    nl = Netlist(top_mod.name, lib)
    for pname in top_mod.port_order:
        kind, msb, lsb = top_mod.decl.get(pname, ("wire", 0, 0))
        if kind not in ("input", "output"):
            raise ParseError(f"端口 {pname} 没有方向声明")
        nl.ports.append(Port(pname, kind, _bits_of(top_mod, pname), msb, lsb))

    # assign 别名：用并查集把同一根导线的不同名字合并，代表元优先选"有驱动的那一侧"
    parent: Dict[str, str] = {}

    def find(x: str) -> str:
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    po_bits = set(nl.pos)
    pi_bits = set(nl.pis)
    for lhs, rhs in top_mod.assigns:
        if len(lhs) != len(rhs):
            rhs = (rhs + [CONST0] * len(lhs))[: len(lhs)]
        for l, r in zip(lhs, rhs):
            a, b = find(l), find(r)
            if a == b:
                continue
            # 常量/主输入 当代表元；其次是非 PO 名（让 PO 能挂在驱动网线上）
            def rank(x):
                return (x in (CONST0, CONST1)) * 4 + (x in pi_bits) * 2 + (x not in po_bits)
            if rank(a) >= rank(b):
                parent[b] = a
            else:
                parent[a] = b

    for cell, iname, positional, named in top_mod.instances:
        if cell not in lib:
            if cell in by_name:
                raise ParseError(f"实例 {iname} 引用了子模块 {cell}；请先 flatten（Yosys: flatten）")
            raise ParseError(f"实例 {iname} 的单元类型 {cell} 不在单元库里")
        cd = lib[cell]
        if positional and not named:
            named = {pin: bits for pin, bits in zip(cd.ports, positional)}
        conns = {}
        for pin, bits in named.items():
            if pin in cd.inouts:
                continue  # 励磁/偏置脚：插入后统一重新生成
            if pin not in cd.inputs and pin not in cd.outputs:
                raise ParseError(f"单元 {cell} 没有端口 {pin}（实例 {iname}）")
            if not bits:
                continue
            if len(bits) != 1:
                raise ParseError(f"{iname}.{pin} 连接了 {len(bits)} 位，单元端口只有 1 位")
            conns[pin] = find(bits[0])
        nl.add(Inst(iname, cell, conns))

    for po in nl.pos:
        r = find(po)
        if r != po:
            nl.po_drivers[po] = r
    # PI 被 assign 别名后，单元连接里用的就是 PI 名（代表元），无需再处理
    for inst in nl.insts.values():
        for pin in lib[inst.cell].outputs:
            net = inst.conns.get(pin)
            if net in pi_bits:
                raise ParseError(f"{inst.name}.{pin} 驱动了主输入 {net}")
    return nl


def parse_verilog_file(path: str, lib: Library, top: Optional[str] = None) -> Netlist:
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return parse_verilog(f.read(), lib, top)
