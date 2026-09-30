"""Minimal ROS diagnostics for trolley_core nodes.

Publishes standard diagnostic_msgs/DiagnosticArray on /diagnostics once a
second, the same wire format diagnostic_updater produces, without needing
that package on the cart. A node registers named checks; each returns
(level, message, values) when called:

    self.diag = Diagnostics(self)
    self.diag.add('lidar scan', self._diag_scan)

Status names are "<node name>: <check>", matching diagnostic_updater.

ROS levels say something is wrong, not what the cart can still do. Where a
check knows, it says so with a VDA 5050 errorLevel in its values:

    return ERROR, 'scan stale; holding STOP', error_level(CRITICAL, values)
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

# VDA 5050 errorLevel: what the cart can still do.
WARNING = 'WARNING'    # no immediate attention needed; carries on
URGENT = 'URGENT'      # needs attention now; carries on
CRITICAL = 'CRITICAL'  # needs attention now; cannot carry on with this job
FATAL = 'FATAL'        # someone has to intervene; cannot take a job at all


def error_level(level, values=None):
    """Return values with the VDA 5050 errorLevel added."""
    return {**(values or {}), 'errorLevel': level}


class Diagnostics:
    def __init__(self, node, period_s=1.0):
        self._node = node
        self._checks = []
        node.declare_parameter('hardware_id', 'trolley-x')
        self._hardware_id = str(node.get_parameter('hardware_id').value)
        self._pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        node.create_timer(period_s, self._publish)

    def add(self, name, check):
        """Register check() -> (level, message, {key: value})."""
        self._checks.append((name, check))

    def _publish(self):
        arr = DiagnosticArray()
        arr.header.stamp = self._node.get_clock().now().to_msg()
        for name, check in self._checks:
            try:
                level, message, values = check()
            except Exception as e:  # a broken check must not take the node down
                level, message, values = ERROR, f'diagnostic check failed: {e}', {}
            arr.status.append(DiagnosticStatus(
                level=level,
                name=f'{self._node.get_name()}: {name}',
                message=message,
                hardware_id=self._hardware_id,
                values=[KeyValue(key=str(k), value=str(v)) for k, v in values.items()],
            ))
        self._pub.publish(arr)
