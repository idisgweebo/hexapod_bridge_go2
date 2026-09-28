#!/usr/bin/env python3
"""One correct main() for every node in this package. Spin, shut down, exit cleanly.

WHY THIS EXISTS
---------------
The first live run of these nodes tracebacked on SIGTERM. Not a crash -- the nodes
had already done their work and were being stopped normally -- but the output was a
Python traceback and a non-zero exit, which is what a crash looks like.

That matters more than cosmetics. `ros2 launch` sends SIGINT and then SIGTERM;
`docker stop` sends SIGTERM; so does systemd. Every ordinary stop of these nodes
would have printed a traceback. The thermal-watchdog fix in session 8 recorded the
principle: a number wrong in the alarming direction trains the operator to ignore
the instrument. A stack trace on every normal shutdown trains an operator to ignore
stack traces, and this project's logs are evidence.

WHY THE EXCEPTION IS IMPORTED DEFENSIVELY
-----------------------------------------
rclpy raises ExternalShutdownException out of spin() when the context is shut down
by a signal. It lives in rclpy.executors in Humble; it does NOT exist in Foxy, which
is this package's deployment target (the Orin bridge container). Importing it
unconditionally would make the package fail to import on Foxy -- trading a cosmetic
bug on one distro for a fatal one on the other.

SIGINT was already handled before this change (rclpy turns it into
KeyboardInterrupt), which is exactly why the defect survived: the interactive test a
developer runs, Ctrl+C, is the one path that worked.
"""
import rclpy

_SHUTDOWN_SIGNALS = [KeyboardInterrupt]
try:  # Humble and later
    from rclpy.executors import ExternalShutdownException
    _SHUTDOWN_SIGNALS.append(ExternalShutdownException)
except ImportError:  # Foxy: no such exception, SIGTERM surfaces differently
    pass
SHUTDOWN_SIGNALS = tuple(_SHUTDOWN_SIGNALS)


def run(node_class, args=None):
    """Construct, spin and tear down a node. Returns a process exit code.

    A failure in the CONSTRUCTOR is deliberately NOT swallowed. Two of these nodes
    refuse to start when the mounted config disagrees with where they actually
    publish, and that refusal has to be loud and fatal -- a node that logged the
    problem and carried on would be worse than one that never started. rclpy is
    still shut down on that path so the process does not leave a participant behind.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
        return 0
    except SHUTDOWN_SIGNALS:
        # A signal is a normal stop, not an error. No traceback, exit 0.
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        # Foxy raises if shutdown() is called on an already-shut-down context and
        # Humble tolerates it; guard so one file behaves the same on both.
        if rclpy.ok():
            rclpy.shutdown()
