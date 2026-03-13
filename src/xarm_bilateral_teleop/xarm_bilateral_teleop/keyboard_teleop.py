"""
Keyboard joystick for bilateral teleop testing.

Publishes WrenchStamped to the master force topic, simulating operator input.
Also publishes optional slave contact force for feedback testing.

Controls:
  Master force (operator):
    W/S  = +/- X       A/D  = +/- Y       Q/E  = +/- Z

  Slave contact force (simulated obstacle):
    I/K  = +/- X       J/L  = +/- Y       U/O  = +/- Z

  Adjustments:
    1/2  = decrease/increase force magnitude
    SPACE = zero all forces
    ESC or Ctrl-C = quit
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

_HELP = """
Keyboard Teleop for Bilateral Force Feedback
---------------------------------------------
Master force (operator push):
  W/S = +/- X    A/D = +/- Y    Q/E = +/- Z

Slave force (simulated contact):
  I/K = +/- X    J/L = +/- Y    U/O = +/- Z

1/2 = decrease/increase force step (current: {step:.1f} N)
SPACE = zero all        ESC = quit
---------------------------------------------"""

_MASTER_KEYS = {
    'w': (0, +1), 's': (0, -1),
    'a': (1, +1), 'd': (1, -1),
    'q': (2, +1), 'e': (2, -1),
}

_SLAVE_KEYS = {
    'i': (0, +1), 'k': (0, -1),
    'j': (1, +1), 'l': (1, -1),
    'u': (2, +1), 'o': (2, -1),
}


def _get_key(settings):
    """Return pressed key, or '' if no key within 20 ms (non-blocking)."""
    tty.setraw(sys.stdin.fileno())
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0.02)
        ch = sys.stdin.read(1) if ready else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self._master_topic = self.declare_parameter(
            'master_force_topic',
            '/teleop/operator_force_direct').value
        self._slave_topic = self.declare_parameter(
            'slave_force_topic',
            '/teleop/slave_force_direct').value
        hz = self.declare_parameter('publish_hz', 50.0).value

        self._pub_master = self.create_publisher(
            WrenchStamped, self._master_topic, 10)
        self._pub_slave = self.create_publisher(
            WrenchStamped, self._slave_topic, 10)

        self._step = 2.0
        self._master_f = [0.0, 0.0, 0.0]
        self._slave_f = [0.0, 0.0, 0.0]

        self._timer = self.create_timer(1.0 / hz, self._publish)

    def _publish(self):
        now = self.get_clock().now().to_msg()

        m = WrenchStamped()
        m.header.stamp = now
        m.header.frame_id = 'link_base'
        m.wrench.force.x = self._master_f[0]
        m.wrench.force.y = self._master_f[1]
        m.wrench.force.z = self._master_f[2]
        self._pub_master.publish(m)

        s = WrenchStamped()
        s.header.stamp = now
        s.header.frame_id = 'link_base'
        s.wrench.force.x = self._slave_f[0]
        s.wrench.force.y = self._slave_f[1]
        s.wrench.force.z = self._slave_f[2]
        self._pub_slave.publish(s)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(_HELP.format(step=self._step))
        self._print_status()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = _get_key(settings)

                if not key:
                    continue  # no key — just keep publishing at 50 Hz

                if key == '\x1b' or key == '\x03':  # ESC or Ctrl-C
                    break
                elif key == ' ':
                    self._master_f = [0.0, 0.0, 0.0]
                    self._slave_f = [0.0, 0.0, 0.0]
                elif key == '1':
                    self._step = max(0.5, self._step - 0.5)
                elif key == '2':
                    self._step = min(20.0, self._step + 0.5)
                elif key in _MASTER_KEYS:
                    axis, sign = _MASTER_KEYS[key]
                    self._master_f[axis] += sign * self._step
                elif key in _SLAVE_KEYS:
                    axis, sign = _SLAVE_KEYS[key]
                    self._slave_f[axis] += sign * self._step

                self._print_status()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _print_status(self):
        mf = self._master_f
        sf = self._slave_f
        print(
            f'\rMaster F: [{mf[0]:+6.1f}, {mf[1]:+6.1f}, {mf[2]:+6.1f}] N  '
            f'| Slave F: [{sf[0]:+6.1f}, {sf[1]:+6.1f}, {sf[2]:+6.1f}] N  '
            f'| Step: {self._step:.1f} N   ',
            end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
