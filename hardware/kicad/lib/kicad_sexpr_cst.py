# Adapted from mcp-server-kicad (ProductOfAmerica), MIT License.
# https://github.com/ProductOfAmerica/mcp-server-kicad
# See hardware/kicad/lib/THIRD_PARTY_LICENSES.md for the license text and provenance.
"""Lossless concrete syntax tree for KiCad s-expressions. Stdlib only.

This module preserves bytes exactly: ``serialize(parse(data)) == data`` for
well-formed KiCad-style s-expressions. It is intentionally lightweight so
MycoMIDI can safely inspect, generate, and surgically edit KiCad files without
depending on pcbnew or a live KiCad process.
"""

import re

# Order matters: whitespace first, bare atom last (it is the catch-all).
# An unterminated quote runs to EOF rather than raising, so it still round-trips.
TOKEN = re.compile(
    rb'(\s+)|(\()|(\))|("(?:[^"\\]|\\.)*"?)|([^\s()"]+)',
    re.DOTALL,
)

WS, OPEN, CLOSE, STR, BARE = 1, 2, 3, 4, 5

_UNESC = {b"\\": b"\\", b'"': b'"', b"n": b"\n", b"r": b"\r", b"t": b"\t"}
_ESC = ((b"\\", b"\\\\"), (b'"', b'\\"'), (b"\n", b"\\n"), (b"\r", b"\\r"))
_EMPTY = b""


class Node:
    """Common base for CST nodes."""

    __slots__ = ()

    @property
    def atoms(self):
        return [child for child in self.children or () if child.kind == "atom"]

    @property
    def lists(self):
        return [child for child in self.children or () if child.kind == "list"]

    @property
    def head(self):
        atoms = self.atoms
        return atoms[0].text if atoms else None

    def find_all(self, name):
        return [child for child in self.lists if child.head == name]

    def find(self, name):
        found = self.find_all(name)
        return found[0] if found else None


class Atom(Node):
    __slots__ = ("raw", "sep")

    kind = "atom"
    children = None

    def __init__(self, raw=b"", sep=b""):
        self.raw = raw
        self.sep = sep

    @property
    def text(self):
        """Decoded atom text using KiCad-compatible quoted-string rules."""
        raw = self.raw
        if raw[:1] == b'"':
            raw = raw[1:-1] if raw[-1:] == b'"' and len(raw) > 1 else raw[1:]
            raw = re.sub(
                rb"\\(.)",
                lambda match: _UNESC.get(match.group(1), b"\\" + match.group(1)),
                raw,
                flags=re.DOTALL,
            )
        return raw.decode("utf-8", "surrogateescape")

    def set_text(self, value):
        """Replace atom text, quoting only when the value requires it."""
        raw = value.encode("utf-8")
        if self.raw[:1] == b'"' or re.search(rb'[\s()"\\]', raw) or not raw:
            for needle, replacement in _ESC:
                raw = raw.replace(needle, replacement)
            raw = b'"' + raw + b'"'
        self.raw = raw

    def copy(self):
        return Atom(self.raw, self.sep)

    def __repr__(self):
        return f"Atom({self.text!r})"


class List(Node):
    __slots__ = ("sep", "children", "close_sep")

    kind = "list"
    raw = b"("
    close = b")"

    def __init__(self, sep=b"", children=None, close_sep=b""):
        self.sep = sep
        self.children = children if children is not None else []
        self.close_sep = close_sep

    def insert_after(self, ref, node, sep=None):
        index = self.children.index(ref)
        node.sep = sep if sep is not None else (ref.sep or b"\n")
        self.children.insert(index + 1, node)

    def insert_before(self, ref, node, sep=None):
        index = self.children.index(ref)
        if sep is None:
            sep = ref.sep or b"\n"
        node.sep = ref.sep
        ref.sep = sep
        self.children.insert(index, node)

    def append_child(self, node, sep=b"\n"):
        node.sep = sep
        self.children.append(node)

    def remove_child(self, ref):
        self.children.remove(ref)

    def copy(self):
        return type(self)(self.sep, [child.copy() for child in self.children], self.close_sep)

    def __repr__(self):
        return f"List({self.head!r}, {len(self.atoms)} atoms, {len(self.lists)} lists)"


class Doc(List):
    __slots__ = ()

    kind = "doc"
    raw = _EMPTY
    close = _EMPTY


def parse(data: bytes) -> Doc:
    """Parse bytes into a byte-preserving KiCad s-expression CST."""
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("parse() takes bytes, not str")
    root = Doc()
    stack: list[List] = [root]
    pending = _EMPTY
    interned: dict[bytes, bytes] = {}
    pos = 0
    for match in TOKEN.finditer(data):
        if match.start() != pos:
            raise AssertionError(f"gap at {pos}:{match.start()}")
        pos = match.end()
        group = match.lastindex
        if group == WS:
            token = match.group()
            pending = interned.setdefault(token, token)
            continue
        top = stack[-1]
        if group == OPEN:
            node = List(sep=pending)
            top.children.append(node)
            stack.append(node)
        elif group == CLOSE:
            if len(stack) == 1:
                raise SyntaxError(f"unmatched ')' at byte {match.start()}")
            stack.pop().close_sep = pending
        else:
            token = match.group()
            top.children.append(Atom(interned.setdefault(token, token), pending))
        pending = _EMPTY
    if pos != len(data):
        raise AssertionError(f"trailing {len(data) - pos} bytes unconsumed")
    if len(stack) != 1:
        raise SyntaxError(f"{len(stack) - 1} unclosed '(' at end of input")
    root.close_sep = pending
    return root


def serialize(node: Node) -> bytes:
    """Serialize a CST node back to bytes without changing untouched spans."""
    out = []
    stack: list[Node | bytes] = [node]
    while stack:
        item = stack.pop()
        if isinstance(item, bytes):
            out.append(item)
            continue
        out.append(item.sep)
        out.append(item.raw)
        if item.children is not None:
            stack.append(item.close)
            stack.append(item.close_sep)
            stack.extend(reversed(item.children))
    return b"".join(out)


def _num(value: float) -> str:
    return str(int(value)) if value == int(value) else str(value)


def _numish(text: str) -> float | int:
    value = float(text)
    return int(value) if value == int(value) else value


def _node_text(node) -> str:
    return node.atoms[1].text


def _node_xy(node) -> tuple[float, float]:
    at = node.find("at")
    return float(at.atoms[1].text), float(at.atoms[2].text)


def _fill_at(node, x: float, y: float, rotation: float | None = None) -> None:
    at = node.find("at")
    at.atoms[1].set_text(_num(x))
    at.atoms[2].set_text(_num(y))
    if rotation is not None:
        if len(at.atoms) > 3:
            at.atoms[3].set_text(_num(rotation))
        else:
            rot = at.atoms[2].copy()
            rot.sep = b" "
            rot.set_text(_num(rotation))
            at.children.append(rot)


def demo():
    """Self-check: byte round-trip, escape codec, surgical edits, malformed input."""
    hard = (
        b"(kicad_sch\r\n"
        b'\t(a "say \\"hi\\"" "back\\\\slash" "multi\nline" "nl\\nB" "un\\Bk")\r\n'
        b'\t(b 1.5 -2 ~ \xe2\x84\xa6 "")\r\n)\r\n'
    )
    tree = parse(hard)
    assert serialize(tree) == hard, "byte round-trip broken"
    sch = tree.lists[0]
    a = sch.find("a")
    assert sch.head == "kicad_sch"
    assert a.atoms[1].text == 'say "hi"'
    assert a.atoms[2].text == "back\\slash"
    assert a.atoms[3].text == "multi\nline"
    assert a.atoms[4].text == "nl\nB"
    assert a.atoms[5].text == "un\\Bk"
    assert sch.find("b").atoms[4].text.encode() == b"\xe2\x84\xa6"

    probe = Atom(b"x")
    for value in ['a"b\\c', "line1\nline2", "tab\there", "plain", "", "Ω()"]:
        probe.set_text(value)
        assert probe.text == value, (value, probe.raw, probe.text)
        assert b"\n" not in probe.raw, probe.raw

    sch.find("b").atoms[1].set_text("9.75")
    out = serialize(tree)
    assert out == hard.replace(b"(b 1.5", b"(b 9.75"), out

    tree2 = parse(hard)
    sch2 = tree2.lists[0]
    node = sch2.find("a").copy()
    node.atoms[0].set_text("c")
    sch2.insert_after(sch2.find("b"), node)
    out2 = serialize(tree2)
    assert out2.count(b'"multi\nline"') == 2 and out2.startswith(hard[:20])
    assert serialize(parse(out2)) == out2

    tree_at = parse(b"(x\n\t(at 1 2)\n)")
    _fill_at(tree_at.lists[0], 3, 4, 90)
    assert serialize(tree_at) == b"(x\n\t(at 3 4 90)\n)", serialize(tree_at)

    tree3 = parse(b"(r\n\t(x 1)\n\t(z 3)\n)")
    root = tree3.lists[0]
    y = parse(b"(y 2)").lists[0]
    root.insert_before(root.find("z"), y)
    assert serialize(tree3) == b"(r\n\t(x 1)\n\t(y 2)\n\t(z 3)\n)", serialize(tree3)
    root.remove_child(root.find("y"))
    assert serialize(tree3) == b"(r\n\t(x 1)\n\t(z 3)\n)", serialize(tree3)
    w = parse(b"(w 4)").lists[0]
    root.append_child(w, b"\n\t")
    assert serialize(tree3) == b"(r\n\t(x 1)\n\t(z 3)\n\t(w 4)\n)", serialize(tree3)

    for bad in (b"(a (b)", b"(a))"):
        try:
            parse(bad)
        except SyntaxError:
            pass
        else:
            raise AssertionError(f"{bad!r} should have raised")


__all__ = [
    "Atom",
    "Doc",
    "List",
    "Node",
    "demo",
    "parse",
    "serialize",
]
