"""
calc — evaluate a Python-compatible math expression.

The LLM is unreliable at arithmetic done in its head, so give it a calculator.
Takes a single expression string (Python operator syntax, e.g. ``(3 + 4) * 2``,
``2 ** 10``, ``sqrt(2)``, ``sin(pi / 6)``) and returns the numeric result.

Safety: the expression is parsed with ``ast`` and walked against a whitelist of
node types, so only literals, arithmetic/comparison operators, and a fixed set of
``math`` functions/constants are allowed. There is no ``eval`` of arbitrary code —
names, attribute access, calls to anything outside the whitelist, and comprehensions
all raise a clean error dict rather than executing. This keeps a model-authored
string from reaching the interpreter as code.
"""
import ast
import math
import operator

# Whitelisted names: math functions + constants. No builtins, no attribute access.
_ALLOWED_NAMES = {
    name: getattr(math, name)
    for name in (
        "sqrt", "exp", "log", "log2", "log10", "pow", "fabs", "factorial",
        "floor", "ceil", "trunc", "gcd", "sin", "cos", "tan", "asin", "acos",
        "atan", "atan2", "sinh", "cosh", "tanh", "degrees", "radians", "hypot",
        "copysign", "fmod", "isqrt",
    )
}
_ALLOWED_NAMES.update({
    "pi": math.pi, "e": math.e, "tau": math.tau, "inf": math.inf, "nan": math.nan,
    "abs": abs, "round": round, "min": min, "max": max, "sum": sum,
})

_BIN_OPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.Div: operator.truediv, ast.FloorDiv: operator.floordiv,
    ast.Mod: operator.mod, ast.Pow: operator.pow,
}
_UNARY_OPS = {ast.UAdd: operator.pos, ast.USub: operator.neg}
_CMP_OPS = {
    ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt,
    ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
}


def _eval(node):
    if isinstance(node, ast.Constant):
        if isinstance(node.value, (int, float)):
            return node.value
        raise ValueError(f"disallowed constant: {node.value!r}")
    if isinstance(node, ast.BinOp) and type(node.op) in _BIN_OPS:
        return _BIN_OPS[type(node.op)](_eval(node.left), _eval(node.right))
    if isinstance(node, ast.UnaryOp) and type(node.op) in _UNARY_OPS:
        return _UNARY_OPS[type(node.op)](_eval(node.operand))
    if isinstance(node, ast.Compare):
        # Chained comparisons (a < b < c), evaluated left-to-right like Python.
        left = _eval(node.left)
        for op, comparator in zip(node.ops, node.comparators):
            if type(op) not in _CMP_OPS:
                raise ValueError("disallowed comparison operator")
            right = _eval(comparator)
            if not _CMP_OPS[type(op)](left, right):
                return False
            left = right
        return True
    if isinstance(node, ast.Name):
        if node.id in _ALLOWED_NAMES:
            return _ALLOWED_NAMES[node.id]
        raise ValueError(f"unknown name: {node.id}")
    if isinstance(node, ast.Call):
        if not isinstance(node.func, ast.Name) or node.func.id not in _ALLOWED_NAMES:
            raise ValueError("only whitelisted math functions may be called")
        if node.keywords:
            raise ValueError("keyword arguments are not allowed")
        func = _ALLOWED_NAMES[node.func.id]
        if not callable(func):
            raise ValueError(f"{node.func.id} is not callable")
        return func(*[_eval(a) for a in node.args])
    raise ValueError(f"disallowed expression element: {type(node).__name__}")


def _calculate(params):
    expr = ((params or {}).get("expression") or "").strip()
    if not expr:
        return {"error": "expression is required"}
    try:
        tree = ast.parse(expr, mode="eval")
        result = _eval(tree.body)
    except ZeroDivisionError:
        return {"error": "division by zero"}
    except SyntaxError:
        return {"error": f"could not parse expression: {expr!r}"}
    except Exception as e:
        return {"error": f"math error: {e}"}
    return {"expression": expr, "result": result}


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "calculate",
        "description": (
            "Evaluate a Python-syntax math expression (e.g. 'sqrt(2)*3', "
            "'factorial(5)'). Use for any arithmetic instead of computing it "
            "yourself."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "expression": {"type": "string",
                               "description": "The math expression to evaluate."},
            },
            "required": ["expression"],
        },
    },
]

_HANDLERS = {"calculate": _calculate}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return {"error": f"unknown tool: {name}"}
    return handler(arguments or {})
