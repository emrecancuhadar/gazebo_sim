#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import tempfile
import os
import random
import time
import sys
import subprocess
import signal
import socket
import argparse
from subprocess import CalledProcessError, TimeoutExpired
from datetime import datetime

class GazeboServiceTester(Node):
    def __init__(self, reset_frequency=50, operation_interval=2.0, world_name='default'):
        super().__init__('gazebo_service_tester')

        self.reset_frequency = reset_frequency
        self.operation_interval = operation_interval
        self.world_name = world_name

        self.operation_count = 0
        self.total_operations = 0
        self.spawn_count = 0
        self.delete_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.model_base = 'test_box'
        self.spawned_models = []
        self.next_spawn = True
        self.test_running = False

        socket.setdefaulttimeout(5.0)

        self.temp_dir = tempfile.mkdtemp()
        self.log_path = os.path.join(self.temp_dir, 'gazebo_service_test.log')
        self.log_operation(f"TEST STARTED - Reset every {self.reset_frequency}, interval {self.operation_interval}s")

        self.create_timer(30.0, self.report_status)

    def run_cmd_with_retry(self, cmd, timeout=4.0, retries=3):
        """Run a subprocess command, retrying up to `retries` times on failure."""
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                result = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=timeout
                )
                if result.returncode == 0:
                    return result
                else:
                    last_exc = CalledProcessError(result.returncode, cmd, output=result.stdout, stderr=result.stderr)
                    self.get_logger().warn(
                        f"Command failed (code {result.returncode}) on attempt {attempt}/{retries}: {result.stderr.strip()}"
                    )
            except TimeoutExpired as e:
                last_exc = e
                self.get_logger().warn(f"Command timeout on attempt {attempt}/{retries}: {e}")
            except CalledProcessError as e:
                last_exc = e
                self.get_logger().warn(f"Command error on attempt {attempt}/{retries}: {e}")

            if attempt < retries:
                time.sleep(0.5)

        # all attempts failed
        raise last_exc

    def start_test(self):
        if self.test_running:
            self.get_logger().warning("Test already running")
            return
        self.test_running = True
        self.get_logger().info("Starting Gazebo spawn/delete test")
        self.operation_timer = self.create_timer(self.operation_interval, self.execute_operation)

    def stop_test(self):
        if not self.test_running:
            return
        self.test_running = False
        self.operation_timer.cancel()
        self.log_operation("Test execution stopped")
        self.report_status(final=True)

    def execute_operation(self):
        if not self.test_running:
            return

        if self.operation_count >= self.reset_frequency:
            self.restart_ros2_daemon()
            self.operation_count = 0

        if self.next_spawn:
            success = self.spawn_model()
        else:
            if not self.spawned_models:
                success = self.spawn_model()
            else:
                success = self.delete_model()

        self.operation_count += 1
        self.total_operations += 1
        if success:
            self.success_count += 1
        else:
            self.failure_count += 1

        self.next_spawn = not self.next_spawn

    def restart_ros2_daemon(self):
        self.get_logger().info(f"Restarting ROS2 daemon after {self.total_operations} ops")
        self.log_operation(f"ROS2 daemon restart triggered")
        try:
            subprocess.run(['ros2', 'daemon', 'restart'], check=True)
            self.log_operation("ROS2 daemon restarted successfully")
        except Exception as e:
            self.get_logger().error(f"Daemon restart failed: {e}")
            self.log_operation(f"ROS2 daemon restart ERROR: {e}")

    def spawn_model(self):
        name = f"{self.model_base}_{self.spawn_count}"
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        z = 0.5

        self.get_logger().info(f"Spawning {name} at ({x:.2f},{y:.2f},{z:.2f})")
        sdf_content = f"""<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='link'>
      <collision name='col'>
        <geometry><box><size>1 1 1</size></box></geometry>
      </collision>
      <visual name='vis'>
        <geometry><box><size>1 1 1</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>"""

        sdf_file = os.path.join(self.temp_dir, f"{name}.sdf")
        with open(sdf_file, 'w') as f:
            f.write(sdf_content)

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            f"-{self.world_name}", self.world_name,
            "-file", sdf_file,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-name", name
        ]
        try:
            result = self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            self.spawned_models.append(name)
            self.log_operation(f"SPAWN SUCCESS: {name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Spawn exception after retries: {e}")
            self.log_operation(f"SPAWN FAILURE: {name}")
            return False

    def delete_model(self):
        name = self.spawned_models.pop(0)
        self.get_logger().info(f"Deleting model: {name}")

        cmd = [
            "ros2", "run", "ros_gz_sim", "remove",
            "--ros-args", "-p", f"entity_name:={name}"
        ]
        try:
            result = self.run_cmd_with_retry(cmd)
            self.delete_count += 1
            self.log_operation(f"DELETE SUCCESS: {name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Delete exception after retries: {e}")
            self.log_operation(f"DELETE FAILURE: {name}")
            # put it back so cleanup can remove it
            self.spawned_models.insert(0, name)
            return False

    def report_status(self, final=False):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rate = (self.success_count / self.total_operations * 100) if self.total_operations else 0.0
        status = (
            f"\n===== STATUS @ {now} =====\n"
            f"Total ops: {self.total_operations}\n"
            f"Success:   {self.success_count}\n"
            f"Failure:   {self.failure_count}\n"
            f"Rate:      {rate:.2f}%\n"
            f"Spawned:   {self.spawn_count}\n"
            f"Deleted:   {self.delete_count}\n"
            f"Alive:     {len(self.spawned_models)}\n"
            f"Since reset: {self.operation_count}/{self.reset_frequency}\n"
            f"Log:       {self.log_path}\n"
            "============================"
        )
        self.get_logger().info(status)
        if final:
            self.log_operation(f"FINAL: ops={self.total_operations}, rate={rate:.2f}%")

    def log_operation(self, msg):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}\n"
        try:
            with open(self.log_path, 'a') as f:
                f.write(line)
        except Exception as e:
            self.get_logger().error(f"Log write error: {e}")

    def cleanup(self):
        self.get_logger().info("Cleaning up spawned models...")
        for name in list(self.spawned_models):
            self.delete_model()
        self.report_status(final=True)
        self.log_operation("TEST ENDED - cleanup done")
        self.get_logger().info(f"Log at {self.log_path}")

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--reset-frequency', type=int, default=50)
    parser.add_argument('--interval', type=float, default=2.0)
    parser.add_argument('--world', type=str, default='default')
    parser.add_argument('--duration', type=int, default=0,
                        help='Seconds to run (0 = infinite)')
    raw_args = parser.parse_args(args[1:] if args else None)

    tester = GazeboServiceTester(
        reset_frequency=raw_args.reset_frequency,
        operation_interval=raw_args.interval,
        world_name=raw_args.world
    )
    tester.start_test()

    def shutdown(signum, frame):
        print("Shutting down...")
        tester.stop_test()
        tester.cleanup()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    if raw_args.duration > 0:
        import threading
        t = threading.Timer(raw_args.duration, shutdown, args=(None,None))
        t.daemon = True
        t.start()

    print("Running. Ctrl‑C to stop.")
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
