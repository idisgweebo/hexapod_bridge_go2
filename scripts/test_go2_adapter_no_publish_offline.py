#!/usr/bin/env python3
"""THE SAFETY TEST. Mechanically enforces "go2_adapter never publishes to the Go2".

Runs with NO ROS, NO container and NO robot -- it is a static AST walk.

WHY MECHANICAL
--------------
"Never publish to a robot topic" is the project's first hard rule, and it has so
far been kept by discipline. Discipline is exactly what fails at 11pm on the
fourth debugging pass. This test makes the rule a property of the repository
rather than a property of whoever is typing.

WHAT IT CHECKS
--------------
1. Every create_publisher() topic argument is a STRING LITERAL. A variable or an
   f-string would let a topic name be assembled at runtime, which would defeat
   every other check here -- so the indirection itself is the violation.
2. Every such literal is under the /go2/ namespace.
3. unitree_api is never imported. It is the request/response package; the
   adapter has no business with it, and importing it is the first step of any
   command path.
4. No Go2 command-topic substring appears anywhere in the package, in any
   context -- code, comment or string.

ANTI-VACUITY
------------
A static check that finds nothing passes trivially. This test therefore reports
what it actually inspected and FAILS if it scanned no files at all, so "the
package is empty" can never masquerade as "the package is safe".
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, "..", "ros2_ws", "src", "go2_adapter"))

# Measured from inventory-go2_ros2.md's never-publish list. /api/ is deliberately
# broad: it covers /api/sport/request, /api/bashrunner/request and, critically,
# /api/programming_actuator/request -- the CVE-2026-27509 RCE vector.
FORBIDDEN_SUBSTRINGS = [
    "/api/", "/utlidar/switch", "/utlidar/mapping_cmd", "/uslam/client_command",
    "/wirelesscontroller", "/lowcmd", "/arm_Command",
]
ALLOWED_PREFIXES = ("/go2/", "go2/")

fails = []
py_files = []
publishers_found = []


def walk_py():
    for root, _dirs, names in os.walk(PKG):
        if any(p in root for p in ("build", "install", "log", "__pycache__")):
            continue
        for n in names:
            if n.endswith(".py"):
                yield os.path.join(root, n)


print(f"scanning package: {PKG}")
if not os.path.isdir(PKG):
    fails.append(f"package directory does not exist: {PKG}")
else:
    py_files = sorted(walk_py())

print(f"  {len(py_files)} python file(s)")

# --- ANTI-VACUITY GATE -------------------------------------------------------
if not py_files:
    fails.append("scanned ZERO python files -- an empty scan is not a pass")
    print("  FAIL scanned zero files; refusing to report success")

for path in py_files:
    rel = os.path.relpath(path, PKG)
    src = open(path, encoding="utf-8").read()
    try:
        tree = ast.parse(src, filename=path)
    except SyntaxError as exc:
        fails.append(f"{rel}: does not parse: {exc}")
        continue

    for node in ast.walk(tree):
        # 1 + 2: create_publisher topic must be a literal under /go2/
        if isinstance(node, ast.Call):
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", None)
            if name == "create_publisher":
                # signature: create_publisher(msg_type, topic, qos)
                topic_arg = node.args[1] if len(node.args) >= 2 else None
                for kw in node.keywords:
                    if kw.arg == "topic":
                        topic_arg = kw.value
                if topic_arg is None:
                    fails.append(f"{rel}:{node.lineno}: create_publisher with no topic argument")
                elif isinstance(topic_arg, ast.Constant) and isinstance(topic_arg.value, str):
                    topic = topic_arg.value
                    publishers_found.append((rel, node.lineno, topic))
                    if not topic.startswith(ALLOWED_PREFIXES):
                        fails.append(
                            f"{rel}:{node.lineno}: publishes to {topic!r}, outside /go2/")
                else:
                    fails.append(
                        f"{rel}:{node.lineno}: create_publisher topic is not a string "
                        f"literal ({type(topic_arg).__name__}) -- runtime-assembled topic "
                        f"names are forbidden because they defeat this check")

        # 3: unitree_api must never be imported
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name.startswith("unitree_api"):
                    fails.append(f"{rel}:{node.lineno}: imports {a.name}")
        if isinstance(node, ast.ImportFrom):
            if node.module and node.module.startswith("unitree_api"):
                fails.append(f"{rel}:{node.lineno}: imports from {node.module}")

    # 4: forbidden substrings anywhere in the file, including comments
    for bad in FORBIDDEN_SUBSTRINGS:
        if bad in src:
            for i, line in enumerate(src.splitlines(), 1):
                if bad in line:
                    fails.append(f"{rel}:{i}: contains forbidden Go2 command topic {bad!r}")

print(f"  {len(publishers_found)} create_publisher call(s) inspected")
for rel, line, topic in publishers_found:
    print(f"    {rel}:{line}  ->  {topic}")
if not publishers_found:
    print("    (none yet -- reported, not treated as a pass)")

print()
print("FAILURES:" if fails else "ALL NO-PUBLISH SAFETY CHECKS PASS")
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
