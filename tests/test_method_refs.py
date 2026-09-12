"""Every `self.name(...)` call inside a class must resolve to a method or an
attribute assigned somewhere in that class. Catches renamed-method typos that
py_compile cannot see and that only surface when the ROS node starts (which
happened once: localization.py called self._speed_cap() after the method had
been renamed speed_cap, and the car never moved)."""
import ast
import pathlib
import unittest

SRC = pathlib.Path(__file__).resolve().parents[1] / 'src' / 'pit_crew'
FILES = ['localization.py', 'mpc3.py', 'mpc3_core.py', 'mpc2_core.py',
         'follow_the_gap.py', 'vehicle.py', 'throttle2.py', 'mpc.py', 'pf_core.py']


def unresolved(path):
    tree = ast.parse(path.read_text())
    bad = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        defined = {n.name for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
        # attributes assigned anywhere in the class body (self.x = ..., incl. tuple targets)
        for n in ast.walk(cls):
            if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) \
                    and n.value.id == 'self' and isinstance(n.ctx, ast.Store):
                defined.add(n.attr)
        # bases we cannot see (rclpy Node etc.): only flag private/underscore names and
        # names that look like our own (defined nowhere and not obviously inherited)
        for n in ast.walk(cls):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                    and isinstance(n.func.value, ast.Name) and n.func.value.id == 'self':
                name = n.func.attr
                if name not in defined and name.startswith('_'):
                    bad.append(f'{path.name}:{n.lineno} self.{name}() in class {cls.name}')
    return bad


class MethodRefs(unittest.TestCase):
    def test_private_self_calls_resolve(self):
        problems = []
        for f in FILES:
            p = SRC / f
            if p.exists():
                problems += unresolved(p)
        self.assertEqual(problems, [])


if __name__ == '__main__':
    unittest.main()
