#!/usr/bin/env python3
import os
import sys
import time
import signal
import rosbag
import psutil
import subprocess
from pathlib import Path

class BagProcessor:
    def __init__(self, input_bag, output_bag, package, node):
        self.input_bag = Path(input_bag).absolute()
        self.output_bag = Path(output_bag).absolute()
        self.package = 'testing_pkg'
        self.node = 'imu_fft_node.py'
        self.processes = []

    def _reindex(self, bag_path):
        """Reindex bag file and clean up temporary files"""
        print(f"Reindexing {bag_path.name}...")
        result = subprocess.run(
            f"rosbag reindex {bag_path}",
            shell=True,
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print(f"Reindex failed: {result.stderr}")
            raise RuntimeError("Bag reindexing failed")
        
        # Clean up temporary reindexing files
        for temp_file in [f"{bag_path.stem}.orig.bag", f"{bag_path.stem}.bag"]:
            temp_path = bag_path.with_name(temp_file)
            if temp_path.exists():
                try:
                    temp_path.unlink()
                except Exception as e:
                    print(f"Warning: Could not remove {temp_path}: {e}")
        
        return bag_path

    def _finalize_output(self):
        """Handle active file and ensure proper output"""
        active_path = self.output_bag.with_name(self.output_bag.name + '.active')
        
        if active_path.exists():
            print(f"Finalizing {active_path.name}...")
            try:
                # Properly close the active file before renaming
                subprocess.run(f"rosbag reindex {active_path}", shell=True, check=True)
                active_path.rename(self.output_bag)
                # Verify the output bag
                subprocess.run(f"rosbag check {self.output_bag}", shell=True, check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error finalizing bag: {e}")
                raise RuntimeError("Failed to finalize output bag")

        if not self.output_bag.exists():
            raise FileNotFoundError("Output bag was not created")
            
        print(f"Verifying {self.output_bag.name}...")
        try:
            # Use rosbag check for more thorough validation
            subprocess.run(f"rosbag check {self.output_bag}", shell=True, check=True)
            return self.output_bag
        except subprocess.CalledProcessError:
            print("Output needs reindexing...")
            return self._reindex(self.output_bag)

    def run_fft_node(self):
        """Launch the FFT processing node"""
        cmd = f"rosrun {self.package} {self.node}"
        proc = subprocess.Popen(
            cmd,
            shell=True,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.processes.append(proc)
        time.sleep(1)
        return proc.pid

    def process(self):
        try:
            # 1. Verify input bag
            try:
                subprocess.run(f"rosbag check {self.input_bag}", shell=True, check=True)
                with rosbag.Bag(self.input_bag) as bag:
                    topics = list(bag.get_type_and_topic_info()[1].keys())
            except subprocess.CalledProcessError:
                print(f"{self.input_bag.name} needs reindexing...")
                self._reindex(self.input_bag)
                with rosbag.Bag(self.input_bag) as bag:
                    topics = list(bag.get_type_and_topic_info()[1].keys())

            # 2. Start recording with buffer flushing
            fft_topics = [
                '/imu/fft/x',
                '/imu/fft/y',
                '/imu/fft/z',
                '/imu/fft/frequencies'
            ]
            record_cmd = (
                f"rosbag record --buffsize=0 --chunksize=768 "
                f"-O {self.output_bag} {' '.join(topics + fft_topics)}"
            )
            recorder = subprocess.Popen(record_cmd, shell=True)
            self.processes.append(recorder)
            time.sleep(1)

            # 3. Start FFT node
            self.run_fft_node()

            # 4. Play bag with rate control
            play_cmd = (
                f"rosbag play {self.input_bag} "
                "--clock --wait-for-subscribers --rate=1.0"
            )
            player = subprocess.Popen(play_cmd, shell=True)
            player.wait()

            # 5. Finalize output
            final_path = self._finalize_output()
            print(f"\n✅ Success! Final output saved to:\n{final_path}")

        except Exception as e:
            print(f"\n❌ Error: {str(e)}", file=sys.stderr)
            return 1
        finally:
            self._cleanup()
        return 0

    def _cleanup(self):
        """Terminate all processes gracefully"""
        # First try SIGINT for graceful shutdown
        for p in self.processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        
        # Wait for processes to terminate
        time.sleep(1)
        
        # Force kill any remaining processes
        for p in self.processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Process ROS bag with FFT node',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('input_bag', help='Input bag file')
    parser.add_argument('-o', '--output', default='output.bag', help='Output bag file')
    parser.add_argument('-p', '--package', default='your_package', help='ROS package name')
    parser.add_argument('-n', '--node', default='imu_fft_node.py', help='Node script name')
    
    args = parser.parse_args()

    processor = BagProcessor(
        input_bag=args.input_bag,
        output_bag=args.output,
        package=args.package,
        node=args.node
    )
    sys.exit(processor.process())