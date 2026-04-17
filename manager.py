# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import ntcore
from output.ManagerServer import ManagerServer


class InstanceProcess:
    """Manages a single northstar instance subprocess."""

    def __init__(
        self, name: str, config_path: str, calibration_path: str, instance_config: dict
    ):
        self.name = name
        self.config_path = config_path
        self.calibration_path = calibration_path
        self.instance_config = instance_config
        self.process: subprocess.Popen = None
        self.status = "stopped"  # stopped, starting, running, crashed
        self.exit_code = None
        self._should_run = False
        self._monitor_thread: threading.Thread = None

    @property
    def stream_port(self) -> int:
        return self.instance_config.get("apriltags_stream_port", 8000)

    @property
    def device_id(self) -> str:
        return self.instance_config.get("device_id", "unknown")

    def start(self):
        if self.process and self.process.poll() is None:
            return  # Already running
        self._should_run = True
        self.status = "starting"
        self.exit_code = None
        self._spawn()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_thread = threading.Thread(target=self._monitor, daemon=True)
            self._monitor_thread.start()

    def stop(self):
        self._should_run = False
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.status = "stopped"

    def restart(self):
        self.stop()
        time.sleep(0.5)
        self.start()

    def _spawn(self):
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "__init__.py")
        self.process = subprocess.Popen(
            [
                sys.executable,
                script,
                "--config",
                self.config_path,
                "--calibration",
                self.calibration_path,
            ],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self.status = "running"
        # Drain stdout in background so the pipe doesn't block
        threading.Thread(target=self._drain_output, daemon=True).start()

    def _drain_output(self):
        proc = self.process
        for line in proc.stdout:
            text = line.decode("utf-8", errors="replace").rstrip()
            print(f"[{self.name}] {text}")

    def _monitor(self):
        while self._should_run:
            if self.process and self.process.poll() is not None:
                self.exit_code = self.process.returncode
                if self._should_run:
                    self.status = "crashed"
                    print(
                        f"[{self.name}] Process exited with code {self.exit_code}, restarting in 3s..."
                    )
                    time.sleep(3)
                    if self._should_run:
                        self._spawn()
            time.sleep(1)


class Manager:
    """Manages multiple northstar instances and serves the unified dashboard."""

    def __init__(self, config_path: str):
        with open(config_path, "r") as f:
            self._config = json.load(f)

        self._manager_port = self._config.get("manager_port", 5800)
        self._publish_streams = self._config.get("publish_streams", False)
        self._server_ip = self._config.get("server_ip", "")
        self._instances: list[InstanceProcess] = []
        self._nt_publishers = []  # Keep references so they aren't garbage collected
        self._base_dir = os.path.dirname(os.path.abspath(__file__))

        for inst_def in self._config.get("instances", []):
            config_file = inst_def["config"]
            # Resolve relative paths against the northstar directory
            if not os.path.isabs(config_file):
                config_file = os.path.join(self._base_dir, config_file)

            calibration_file = inst_def["calibration"]
            if not os.path.isabs(calibration_file):
                calibration_file = os.path.join(self._base_dir, calibration_file)

            # Read the instance's config.json to get port info
            with open(config_file, "r") as f:
                instance_config = json.load(f)

            self._instances.append(
                InstanceProcess(
                    name=inst_def["name"],
                    config_path=config_file,
                    calibration_path=calibration_file,
                    instance_config=instance_config,
                )
            )

    def start(self):
        if not self._instances:
            print("No instances configured.")
            return

        print(f"Starting Northstar Manager with {len(self._instances)} camera(s):")
        for inst in self._instances:
            print(
                f"  - {inst.name} (device={inst.device_id}, stream_port={inst.stream_port})"
            )

        # Start all instance processes
        for inst in self._instances:
            inst.start()

        # Publish stream URLs to NetworkTables CameraPublisher so dashboards (e.g. Elastic) can discover them
        if self._publish_streams:
            server_ip = self._server_ip
            if not server_ip:
                # Fall back to first instance's server_ip
                server_ip = self._instances[0].instance_config.get("server_ip", "")
            if not server_ip:
                print(
                    "Warning: publish_streams enabled but no server_ip configured, skipping NT publish"
                )
            else:
                nt_inst = ntcore.NetworkTableInstance.getDefault()
                nt_inst.setServer(server_ip)
                nt_inst.startClient4("northstar_manager")

                # Wait for NT connection before publishing
                print(f"  Connecting to NT server at {server_ip}...")
                connected = False
                for _ in range(50):  # Wait up to 5 seconds
                    if nt_inst.isConnected():
                        connected = True
                        break
                    time.sleep(0.1)
                if not connected:
                    print(
                        "  Warning: Could not connect to NT server, publishing anyway..."
                    )

                # Determine our own IP for stream URLs
                local_ip = self._config.get("stream_ip", "")
                if not local_ip:
                    # Try to resolve our IP on the robot network
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        s.connect((server_ip, 1))
                        local_ip = s.getsockname()[0]
                        s.close()
                    except Exception:
                        local_ip = "localhost"

                for inst in self._instances:
                    cam_name = inst.name.replace(" ", "_")
                    table = nt_inst.getTable(f"/CameraPublisher/{cam_name}")
                    stream_url = (
                        f"mjpg:http://{local_ip}:{inst.stream_port}/stream.mjpg"
                    )
                    # Use Topic API so NT4 type metadata is set correctly
                    streams_pub = table.getStringArrayTopic("streams").publish()
                    streams_pub.set([stream_url])
                    source_pub = table.getStringTopic("source").publish()
                    source_pub.set(f"ip:{local_ip}:{inst.stream_port}")
                    desc_pub = table.getStringTopic("description").publish()
                    desc_pub.set(cam_name)
                    connected_pub = table.getBooleanTopic("connected").publish()
                    connected_pub.set(True)
                    mode_pub = table.getStringTopic("mode").publish()
                    mode_pub.set("mjpg")
                    modes_pub = table.getStringArrayTopic("modes").publish()
                    modes_pub.set(["mjpg"])
                    # Keep publishers alive
                    self._nt_publishers.extend(
                        [
                            streams_pub,
                            source_pub,
                            desc_pub,
                            connected_pub,
                            mode_pub,
                            modes_pub,
                        ]
                    )
                    print(
                        f"  Published {cam_name} stream to CameraPublisher: {stream_url}"
                    )

        # Build dashboard instance list from live process info
        dashboard_instances = [
            {"name": inst.name, "host": "localhost", "port": inst.stream_port}
            for inst in self._instances
        ]

        # Start the dashboard server
        server = ManagerServer(dashboard_instances)
        server.set_instance_processes(self._instances)
        server.start(self._manager_port)

    def stop(self):
        print("\nStopping all instances...")
        for inst in self._instances:
            inst.stop()
        print("All instances stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Northstar multi-camera manager")
    parser.add_argument(
        "--config", default="manager_config.json", help="Path to manager config JSON"
    )
    args = parser.parse_args()

    manager = Manager(args.config)

    def signal_handler(sig, frame):
        manager.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    manager.start()

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        manager.stop()
