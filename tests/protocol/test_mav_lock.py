"""Detect unlocked self.mav.mav.*_send calls.

This is a static-grep test: parse controller.py and assert that every line
calling `self.mav.mav.<x>_send(` lives inside a `with self._mav_lock:` block
(or its equivalent `_send_under_lock` wrapper introduced in Phase 1.7).

A runtime concurrency test would be flaky — this AST/text check is the
honest enforcement mechanism until we extract _connection.py in Phase 3.
"""
import ast
from pathlib import Path

from mavpilot import controller as ctrl_mod


def _send_call_lines() -> list[tuple[int, str]]:
    src = Path(ctrl_mod.__file__).read_text()
    lines = src.splitlines()
    matches: list[tuple[int, str]] = []
    for i, line in enumerate(lines, start=1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue  # skip comment lines
        if "self.mav.mav." in stripped and "_send(" in stripped:
            matches.append((i, stripped))
        if "self.mav.recv_match" in stripped:
            matches.append((i, stripped))
    return matches


def test_every_mav_send_uses_lock_wrapper():
    """Each direct self.mav.mav.*_send call must be inside a
    `with self._mav_lock:` block (or go through `_send_under_lock`)."""
    src = Path(ctrl_mod.__file__).read_text()
    tree = ast.parse(src)

    lock_ranges: list[range] = []

    class V(ast.NodeVisitor):
        def visit_With(self, node):
            for item in node.items:
                ctx = item.context_expr
                if (isinstance(ctx, ast.Attribute)
                        and isinstance(ctx.value, ast.Name)
                        and ctx.value.id == "self"
                        and ctx.attr == "_mav_lock"):
                    lock_ranges.append(range(node.lineno, node.end_lineno + 1))
            self.generic_visit(node)

    V().visit(tree)

    # Lines inside connect() are exempt: they run before the receiver thread starts.
    connect_range: range = range(0)
    for node in ast.walk(tree):
        if isinstance(node, ast.AsyncFunctionDef) and node.name == "connect":
            connect_range = range(node.lineno, (node.end_lineno or node.lineno) + 1)

    unprotected = []
    for lineno, txt in _send_call_lines():
        if lineno in connect_range:
            continue  # exempt: runs before receiver thread exists
        protected = any(lineno in r for r in lock_ranges)
        if not protected:
            unprotected.append((lineno, txt))
    assert not unprotected, (
        "Found self.mav.mav.*_send / recv_match calls NOT protected by "
        f"self._mav_lock:\n" + "\n".join(f"  line {n}: {t}" for n, t in unprotected)
    )


def test_recv_match_timeout_is_50ms_or_less():
    """Receiver thread's recv_match timeout must be <= 0.05 s to bound the
    streamer-starvation window. See spec §4."""
    src = Path(ctrl_mod.__file__).read_text()
    tree = ast.parse(src)
    long_timeouts: list[tuple[int, float]] = []

    class V(ast.NodeVisitor):
        def visit_Call(self, node):
            if (isinstance(node.func, ast.Attribute)
                    and node.func.attr == "recv_match"):
                for kw in node.keywords:
                    if kw.arg == "timeout" and isinstance(kw.value, ast.Constant):
                        if isinstance(kw.value.value, (int, float)) and kw.value.value > 0.05:
                            long_timeouts.append((node.lineno, kw.value.value))
            self.generic_visit(node)

    V().visit(tree)

    flagged: list[tuple[int, float]] = []
    for ln, val in long_timeouts:
        enclosing = None
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if node.lineno <= ln <= (node.end_lineno or ln):
                    if enclosing is None or node.lineno > enclosing.lineno:
                        enclosing = node
        if enclosing is not None and enclosing.name in ("connect",):
            continue
        flagged.append((ln, val))

    assert not flagged, (
        "recv_match() with timeout > 0.05s outside connect():\n"
        + "\n".join(f"  line {n}: timeout={v}" for n, v in flagged)
    )
