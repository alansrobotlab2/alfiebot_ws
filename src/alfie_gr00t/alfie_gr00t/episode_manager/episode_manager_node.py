"""Episode manager - review, relabel and delete GR00T training episodes in a browser.

Usage:
    ros2 run alfie_gr00t episode_manager --ros-args -p dataset:=2026-11-20_s02_hw2

Then open http://<robot>:7880/.

Parameters:
    host               bind address (default 0.0.0.0 - reachable on the LAN)
    port               HTTP port (default 7880; groot_server's visualizer has 7860)
    data_root          folder holding one sub-folder per recording session
                       (default $ALFIE_DATA_ROOT or ~/alfie_data/raw)
    dataset            session opened at startup; others can be picked in the UI
                       (default $ALFIE_DATASET, else the newest session)
    tasks_yaml         task registry file (default: the installed config/tasks.yaml)
    workspace_dir      alfiebot git checkout, for the commit written into a
                       session when it is closed (default ~/alfiebot_ws)
    isaac_groot_dir    Isaac-GR00T checkout, likewise (default ~/Isaac-GR00T)
    python             interpreter for rosbag_to_groot.py and parquet edits. Needs
                       pyarrow, pandas and zstandard; system python has none of
                       them (default ~/Isaac-GR00T/.venv/bin/python if present)
    converter_script   rosbag_to_groot.py to run (default: the installed copy)
    num_threads        converter worker threads (default 8)
    use_gpu            pass --use-gpu to the converter (default false)
    recorder_status_topic
                       data_recorder status, shown in the top bar
                       (default /alfie/recording/status)
    allowed_origins    web origins allowed to call the API cross-origin, for the
                       Foxglove Episode Manager panel (alfie_foxglove_extension).
                       Refused origins are logged once; "*" allows any
                       (default ["https://app.foxglove.dev"])
    urls_topic         where the node advertises its URLs (std_msgs/String,
                       JSON {"urls": [...]}, latched) so the Foxglove panel can
                       find it over the bridge (default /alfie/episode_manager/urls)
"""

import json
import os
import signal
import socket
import sys
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from alfie_gr00t.dataset_paths import data_root
from alfie_gr00t.recording_context import CONTEXT_TOPIC, SET_CONTEXT_TOPIC
from alfie_gr00t.episode_manager.dataset import default_python
from alfie_gr00t.episode_manager.http_server import EpisodeManagerServer
from alfie_gr00t.episode_manager.manager import EpisodeManager


def _lan_address():
    """Best guess at this machine's address on the network it routes over.

    Opening a UDP socket toward an off-link address makes the kernel pick a
    route and bind a source address; nothing is transmitted.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('192.0.2.1', 9))          # TEST-NET-1, never routed
        return probe.getsockname()[0]
    except OSError:
        return None
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
    hostname = socket.gethostname()
    if hostname and hostname != 'localhost':
        urls.append(f'http://{hostname}.local:{port}/')
    urls.append(f'http://127.0.0.1:{port}/')
    return urls


def main(args=None):
    """Start the node and serve the web UI until interrupted."""
    # rclpy's own SIGINT handler shuts the context down without interrupting
    # serve_forever, which leaves the HTTP server running forever. Take the
    # signals ourselves so Ctrl-C and ros2 launch's SIGTERM both land as a
    # KeyboardInterrupt in this thread.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    node = rclpy.create_node('episode_manager')
    logger = node.get_logger()

    host = node.declare_parameter('host', '0.0.0.0').value
    port = int(node.declare_parameter('port', 7880).value)
    root = node.declare_parameter('data_root', str(data_root())).value
    dataset = node.declare_parameter('dataset', os.environ.get('ALFIE_DATASET', '')).value
    tasks_yaml = node.declare_parameter('tasks_yaml', '').value
    workspace_dir = node.declare_parameter('workspace_dir', '~/alfiebot_ws').value
    isaac_groot_dir = node.declare_parameter('isaac_groot_dir', '~/Isaac-GR00T').value
    python = node.declare_parameter('python', default_python()).value
    converter = node.declare_parameter('converter_script', '').value or os.path.join(
        get_package_share_directory('alfie_gr00t'), 'scripts', 'rosbag_to_groot.py')
    num_threads = int(node.declare_parameter('num_threads', 8).value)
    use_gpu = bool(node.declare_parameter('use_gpu', False).value)
    status_topic = node.declare_parameter(
        'recorder_status_topic', '/alfie/recording/status').value
    allowed_origins = list(node.declare_parameter(
        'allowed_origins', ['https://app.foxglove.dev']).value)
    urls_topic = node.declare_parameter('urls_topic', '/alfie/episode_manager/urls').value

    manager = EpisodeManager(root, dataset, python, converter, num_threads, use_gpu, logger,
                             tasks_yaml=tasks_yaml,
                             workspace_dir=workspace_dir, isaac_groot_dir=isaac_groot_dir)
    dataset = manager.dataset.name
    if not dataset:
        logger.info(f'no session under {root} yet; one is made on the first recording, '
                    f'or with New session in the UI')

    def on_recorder_status(msg: String) -> None:
        manager.recorder_status = msg.data

    node.create_subscription(String, status_topic, on_recorder_status, 10)

    # Recording context: data_recorder owns it. Its latched state comes in
    # here; changes made in the UI go out as JSON updates.
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_context(msg: String) -> None:
        try:
            manager.context = json.loads(msg.data)
        except ValueError:
            logger.warn(f'unreadable recording context: {msg.data!r}')

    node.create_subscription(String, CONTEXT_TOPIC, on_context, latched)
    set_context_pub = node.create_publisher(String, SET_CONTEXT_TOPIC, 10)
    manager.send_context = lambda update: set_context_pub.publish(
        String(data=json.dumps(update)))

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        server = EpisodeManagerServer(host, port, manager, logger, allowed_origins)
    except OSError as exc:
        logger.error(f'cannot bind {host}:{port}: {exc}')
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    urls = _urls(host, port)
    logger.info(f'episode manager [{dataset or "no session yet"}]: {"  ".join(urls)}')

    # The Foxglove panel can't see which address Foxglove used to reach the
    # bridge, so the robot tells it, over the bridge. Latched for late joiners,
    # and repeated in case the bridge subscribed with volatile durability.
    urls_pub = node.create_publisher(
        String, urls_topic,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    urls_msg = String(data=json.dumps({'urls': urls}))
    urls_pub.publish(urls_msg)
    node.create_timer(5.0, lambda: urls_pub.publish(urls_msg))
    logger.info(f'converter: {python} {converter}')
    if host in ('0.0.0.0', '::', ''):
        logger.warn('episode manager is unauthenticated and reachable from the whole '
                    'network - anyone who can open it can delete episodes')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        logger.info('shutting down episode manager')
        if manager.job is not None:
            manager.job.cancel()
        server.server_close()
        # Let the spin thread leave rcl before the node goes away under it.
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
