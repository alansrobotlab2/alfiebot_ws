"""ServoTool3 - servo bring-up and diagnostics for gen2 Alfie, in the browser.

Usage:
    ros2 run alfie_tools servotool
    ros2 run alfie_tools servotool --ros-args -p port:=8080 -p host:=127.0.0.1

Then open http://<robot>:7870/.

The default bind is 0.0.0.0, so the UI is reachable from any machine on the
same wifi. The startup log prints the URLs that actually work.

Parameters:
    host              bind address (default 0.0.0.0 - reachable on the LAN)
    port              HTTP port (default 7870)
    stream_rate_hz    state push rate over SSE (default 10)
    robot_namespace   ROS namespace the robot runs in (default "alfie")
    source            command_mux source name to publish as (default "tool")
    forward_rate_hz   command publish rate while holding control (default 20)
    control_timeout   seconds without an operator heartbeat before releasing
                      every held subsystem (default 1.5)
    max_speed         upper bound on commanded joint speed, rad/s (default 6)
    allowed_origins   web origins allowed to call the API cross-origin, for the
                      Foxglove Servo Tool panel (alfie_foxglove_extension).
                      Refused origins are logged once; "*" allows any
                      (default ["https://app.foxglove.dev"])
    urls_topic        where the node advertises its URLs (std_msgs/String,
                      JSON {"urls": [...]}, latched) so the Foxglove panel can
                      find it over the bridge (default /alfie/servotool/urls)
"""

import json
import socket
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from alfie_tools.servotool.ros.servo_bridge import ServoBridge
from alfie_tools.servotool.server.http_server import ServoToolServer


def _lan_address():
    """Best guess at this machine's address on the network it routes over.

    Opening a UDP socket toward an off-link address makes the kernel pick a
    route and bind a source address; nothing is transmitted. Beats
    gethostname() lookups, which return 127.0.1.1 on Debian-family images.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('192.0.2.1', 9))          # TEST-NET-1, never routed
        return probe.getsockname()[0]
    except OSError:
        return None                              # no network at all
    finally:
        probe.close()


def _urls(host: str, port: int):
    """URLs the UI can actually be opened at, best first."""
    if host not in ('0.0.0.0', '::', ''):
        return [f'http://{host}:{port}/']

    urls = []
    lan = _lan_address()
    if lan:
        urls.append(f'http://{lan}:{port}/')
    # avahi publishes <hostname>.local, which survives a DHCP lease change.
    hostname = socket.gethostname()
    if hostname and hostname != 'localhost':
        urls.append(f'http://{hostname}.local:{port}/')
    urls.append(f'http://127.0.0.1:{port}/')
    return urls


def main(args=None):
    """Start the ROS bridge and serve the web UI until interrupted."""
    rclpy.init(args=args)
    node = rclpy.create_node('servotool')
    logger = node.get_logger()

    host = node.declare_parameter('host', '0.0.0.0').value
    port = int(node.declare_parameter('port', 7870).value)
    stream_rate = float(node.declare_parameter('stream_rate_hz', 10.0).value)
    allowed_origins = list(node.declare_parameter(
        'allowed_origins', ['https://app.foxglove.dev']).value)
    urls_topic = node.declare_parameter('urls_topic', '/alfie/servotool/urls').value

    bridge = ServoBridge(node)

    # The bridge's service calls are made from HTTP worker threads and poll their
    # futures, so the node needs an executor that keeps servicing callbacks while
    # those threads wait.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        server = ServoToolServer(host, port, bridge, logger, stream_rate,
                                 allowed_origins)
    except OSError as exc:
        logger.error(f'cannot bind {host}:{port}: {exc}')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    urls = _urls(host, port)
    logger.info(f'servotool web UI: {"  ".join(urls)}')

    # The Foxglove panel can't see which address Foxglove used to reach the
    # bridge, so the robot tells it, over the bridge. Latched for late joiners,
    # and repeated in case the bridge subscribed with volatile durability.
    urls_pub = node.create_publisher(
        String, urls_topic,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    urls_msg = String(data=json.dumps({'urls': urls}))
    urls_pub.publish(urls_msg)
    node.create_timer(5.0, lambda: urls_pub.publish(urls_msg))
    if host in ('0.0.0.0', '::', ''):
        logger.warn('servotool is unauthenticated and reachable from the whole '
                    'network - anyone who can open it can move the arms')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info('shutting down servotool')
    finally:
        # Drop control before the process goes away, so nothing is left holding
        # a subsystem it can no longer refresh.
        bridge.release_all('servotool shutting down')
        server.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
