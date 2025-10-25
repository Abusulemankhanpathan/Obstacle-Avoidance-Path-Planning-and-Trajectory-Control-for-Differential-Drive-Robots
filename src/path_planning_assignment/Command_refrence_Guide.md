# Command Reference Guide

Complete command reference for running, testing, and plotting results.

---

## 📦 Installation Commands

```bash
# Create and setup workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-url> path_planning_assignment
cd ~/ros2_ws

# Install dependencies
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
pip3 install numpy scipy matplotlib pandas pytest pytest-cov

# Build package
colcon build --packages-select path_planning_assignment
source install/setup.bash

# Set environment variables (add to ~/.bashrc)
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Running the System

### Method 1: Complete Launch File
```bash
# Terminal 1: Launch Gazebo + Full System
cd ~/ros2_ws
source install/setup.bash
ros2 launch path_planning_assignment full_system.launch.py
```

### Method 2: Manual Launch (Step-by-Step)
```bash
# Terminal 1: Start Gazebo simulation
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Run follower node
cd ~/ros2_ws
source install/setup.bash
ros2 run path_planning_assignment follower_node

# Terminal 3: Run visualizer
ros2 run path_planning_assignment visualizer

# Terminal 4: Start RViz
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/path_planning_assignment/path_planning_assignment/rviz/path_visualizer_config.rviz
```

---

## 🧪 Testing Commands

### Run All Tests
```bash
cd ~/ros2_ws
source install/setup.bash

# Run all unit tests
python3 -m pytest src/path_planning_assignment/test/ -v

# Run with detailed output
python3 -m pytest src/path_planning_assignment/test/ -v -s
```

### Run Individual Test Suites
```bash
# Path smoothing tests
python3 -m pytest src/path_planning_assignment/test/test_path_smoothing.py -v

# Trajectory generation tests
python3 -m pytest src/path_planning_assignment/test/test_trajectory_generator.py -v

# Controller tests
python3 -m pytest src/path_planning_assignment/test/test_controller.py -v

# Integration tests
python3 -m pytest src/path_planning_assignment/test/test_integration.py -v
```

### Run Specific Test
```bash
# Run single test function
python3 -m pytest src/path_planning_assignment/test/test_path_smoothing.py::TestPathSmoothing::test_circular_arc_curvature -v
```

### Test Coverage
```bash
# Generate coverage report
pip3 install pytest-cov
python3 -m pytest src/path_planning_assignment/test/ --cov=path_planning_assignment --cov-report=html

# View coverage report
firefox htmlcov/index.html  # or: xdg-open htmlcov/index.html
```

### ROS2 Tests
```bash
# Build with tests
colcon build --packages-select path_planning_assignment

# Run ROS2 tests
colcon test --packages-select path_planning_assignment
colcon test-result --verbose
```

---

## 📊 Data Recording & Plotting

### Record Trajectory Data

#### Option 1: ROS2 Bag Recording
```bash
# Start recording (in new terminal while system is running)
cd ~/ros2_ws
source install/setup.bash
ros2 bag record /odom /cmd_vel /path_visualization -o trajectory_data

# Stop recording with Ctrl+C
# Files will be saved as trajectory_data/
```

#### Option 2: Topic Echo to CSV
```bash
# Record odometry to CSV
ros2 topic echo /odom --csv > odom_data.csv

# Record commands to CSV (in separate terminal)
ros2 topic echo /cmd_vel --csv > cmd_data.csv
```

### Generate Plots

#### Using the Complete Analysis Script
```bash
cd ~/ros2_ws/src/path_planning_assignment

# From ROS2 bag file
python3 analyze_and_plot.py --bag trajectory_data --output plots/

# From CSV files
python3 analyze_and_plot.py --odom-csv odom_data.csv --cmd-csv cmd_data.csv --output plots/

# With custom waypoints
python3 analyze_and_plot.py --bag trajectory_data --waypoints waypoints.json --output plots/

# Generate demo plots (no data required)
python3 analyze_and_plot.py --output plots/
```

#### Quick Plotting Scripts

**Simple Trajectory Plot:**
```bash
python3 << 'EOF'
import matplotlib.pyplot as plt
import pandas as pd

# Read CSV data
df = pd.read_csv('odom_data.csv')

# Extract x, y coordinates
x = df['field.pose.pose.position.x']
y = df['field.pose.pose.position.y']

# Plot
plt.figure(figsize=(10, 8))
plt.plot(x, y, 'b-', linewidth=2)
plt.scatter(x.iloc[0], y.iloc[0], c='green', s=200, label='Start', zorder=5)
plt.scatter(x.iloc[-1], y.iloc[-1], c='red', s=200, label='End', zorder=5)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.savefig('trajectory.png', dpi=300)
print("Saved: trajectory.png")
EOF
```

**Velocity Profile Plot:**
```bash
python3 << 'EOF'
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('cmd_data.csv')

# Extract time and velocities
time = df['sec'] + df['nanosec'] * 1e-9
time = time - time.iloc[0]  # Normalize to start at 0
v = df['linear.x']
w = df['angular.z']

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

ax1.plot(time, v, 'b-', linewidth=2)
ax1.set_ylabel('Linear Velocity (m/s)')
ax1.set_title('Velocity Profiles')
ax1.grid(True)

ax2.plot(time, w, 'r-', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.grid(True)

plt.tight_layout()
plt.savefig('velocities.png', dpi=300)
print("Saved: velocities.png")
EOF
```

---

## 🔍 Debugging Commands

### Check System Status
```bash
# List all active nodes
ros2 node list

# Check if topics are publishing
ros2 topic list

# Monitor topic frequency
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# Echo topic data
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once

# Check node information
ros2 node info /trajectory_follower
ros2 node info /path_visualizer
```

### Inspect TF Frames
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_footprint

# Monitor TF broadcasts
ros2 topic echo /tf
```

### RViz Configuration
```bash
# Manual RViz setup (if config file doesn't work)
ros2 run rviz2 rviz2

# Then add:
# 1. Set Fixed Frame: "odom"
# 2. Add → RobotModel
# 3. Add → MarkerArray → Topic: /path_visualization
# 4. Add → Odometry → Topic: /odom
# 5. Save Config → Save to rviz/custom_config.rviz
```

### Performance Profiling
```bash
# Monitor CPU usage
ros2 topic hz /odom --window 100
top -p $(pgrep -f follower_node)

# Check message latency
ros2 topic delay /cmd_vel

# Analyze computational cost
python3 -m cProfile -o profile.stats src/path_planning_assignment/path_planning_assignment/follower_node.py
python3 -c "import pstats; p = pstats.Stats('profile.stats'); p.sort_stats('cumulative').print_stats(20)"
```

---

## 📹 Recording Demo Video

### Record Screen with SimpleScreenRecorder
```bash
# Install
sudo apt install simplescreenrecorder

# Run
simplescreenrecorder

# Settings:
# - Area: Select RViz window
# - FPS: 30
# - Output: demo_video.mp4
```

### Record with ROS2 Tools
```bash
# Record rosbag with compressed images (if using camera)
ros2 bag record -a --compression-mode file -o full_demo

# Convert bag to video (requires image_view)
ros2 run image_view video_recorder image:=/camera/image_raw _filename:=demo.avi
```

### Create GIF from Video
```bash
# Install ffmpeg
sudo apt install ffmpeg

# Convert video to GIF
ffmpeg -i demo_video.mp4 -vf "fps=10,scale=800:-1:flags=lanczos" -loop 0 demo.gif

# Optimize GIF size
ffmpeg -i demo_video.mp4 -vf "fps=5,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" demo_optimized.gif
```

---

## 📈 Generate Performance Metrics

### Create Metrics Report
```bash
cd ~/ros2_ws/src/path_planning_assignment

python3 << 'EOF'
import json
import numpy as np
import pandas as pd

# Load data
odom_df = pd.read_csv('odom_data.csv')
cmd_df = pd.read_csv('cmd_data.csv')

# Compute metrics
x = odom_df['field.pose.pose.position.x'].values
y = odom_df['field.pose.pose.position.y'].values

# Total distance
distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
total_distance = np.sum(distances)

# Time
time = odom_df['sec'].values + odom_df['nanosec'].values * 1e-9
total_time = time[-1] - time[0]

# Velocities
v_cmd = cmd_df['linear.x'].values
w_cmd = cmd_df['angular.z'].values

metrics = {
    "Total Distance": f"{total_distance:.3f} m",
    "Total Time": f"{total_time:.2f} s",
    "Average Speed": f"{total_distance/total_time:.3f} m/s",
    "Max Linear Velocity": f"{np.max(v_cmd):.3f} m/s",
    "Max Angular Velocity": f"{np.max(np.abs(w_cmd)):.3f} rad/s",
    "Mean Linear Velocity": f"{np.mean(v_cmd):.3f} m/s"
}

# Save
with open('metrics.json', 'w') as f:
    json.dump(metrics, f, indent=4)

print("Performance Metrics:")
print(json.dumps(metrics, indent=2))
EOF
```

---

## 🔧 Parameter Tuning

### Modify Controller Parameters
```bash
# Edit follower_node.py parameters
cd ~/ros2_ws/src/path_planning_assignment/path_planning_assignment

# Open in editor
nano follower_node.py

# Adjust parameters (lines 18-24):
# - lookahead_distance: 0.1 → 0.15 (smoother but less accurate)
# - max_linear_vel: 0.15 → 0.22 (faster)
# - max_angular_vel: 1.5 → 2.0 (quicker turns)
# - kp_ang: 2.0 → 1.5 (less aggressive turning)
# - goal_tolerance: 0.15 → 0.10 (more precise)

# Rebuild
cd ~/ros2_ws
colcon build --packages-select path_planning_assignment
source install/setup.bash
```

### Live Parameter Adjustment (Advanced)
```bash
# Use ROS2 param commands
ros2 param set /trajectory_follower lookahead_distance 0.15
ros2 param set /trajectory_follower max_linear_vel 0.20
ros2 param set /trajectory_follower kp_ang 1.8

# List all parameters
ros2 param list /trajectory_follower

# Get current value
ros2 param get /trajectory_follower lookahead_distance
```

---

## 📝 Generate Documentation

### Create API Documentation
```bash
# Install pdoc
pip3 install pdoc3

# Generate HTML docs
cd ~/ros2_ws/src/path_planning_assignment
pdoc --html --output-dir docs path_planning_assignment

# View documentation
firefox docs/path_planning_assignment/index.html
```

### Export Test Results
```bash
# Run tests with JUnit XML output
python3 -m pytest src/path_planning_assignment/test/ --junitxml=test_results.xml

# Generate HTML test report
pip3 install pytest-html
python3 -m pytest src/path_planning_assignment/test/ --html=test_report.html --self-contained-html

# View report
firefox test_report.html
```

---

## 🐛 Common Issues & Solutions

### Issue: Robot doesn't move
```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Verify TurtleBot3 model is set
echo $TURTLEBOT3_MODEL

# Check Gazebo physics
gz physics -i

# Restart Gazebo
killall gzserver gzclient
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Issue: RViz shows nothing
```bash
# Verify Fixed Frame is "odom"
ros2 run rviz2 rviz2
# Global Options → Fixed Frame → "odom"

# Check if markers are published
ros2 topic echo /path_visualization --once

# Verify TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Issue: Tests fail with import errors
```bash
# Set PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/path_planning_assignment

# Or install package in development mode
cd ~/ros2_ws/src/path_planning_assignment
pip3 install -e .
```

### Issue: Bag file won't open
```bash
# Check bag info
ros2 bag info trajectory_data

# Convert bag format (if needed)
ros2 bag convert -i trajectory_data -o trajectory_data_converted

# Verify SQLite database
sqlite3 trajectory_data/trajectory_data_0.db3 "SELECT COUNT(*) FROM messages;"
```

---

## 📤 Submission Preparation

### Create Submission Package
```bash
cd ~/ros2_ws/src

# Create archive
tar -czf path_planning_assignment.tar.gz \
    path_planning_assignment/ \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='build' \
    --exclude='*.bag' \
    --exclude='plots'

# Or create zip
zip -r path_planning_assignment.zip \
    path_planning_assignment/ \
    -x '*.pyc' '*__pycache__*' '*build*' '*.bag' '*plots*'
```

### Generate All Required Files
```bash
cd ~/ros2_ws/src/path_planning_assignment

# 1. Run tests and generate report
python3 -m pytest test/ --html=test_report.html --self-contained-html

# 2. Generate plots
python3 analyze_and_plot.py --bag ~/trajectory_data --output plots/

# 3. Record demo video (manually with SimpleScreenRecorder)

# 4. Create video from images (if you took screenshots)
ffmpeg -framerate 10 -pattern_type glob -i 'screenshots/*.png' -c:v libx264 demo_video.mp4

# 5. Generate documentation
pdoc --html --output-dir docs path_planning_assignment

# 6. Create submission structure
mkdir -p submission/{code,documentation,videos,plots,tests}
cp -r path_planning_assignment submission/code/
cp README.md COMMANDS_REFERENCE.md submission/documentation/
cp demo_video.mp4 submission/videos/
cp -r plots/* submission/plots/
cp test_report.html submission/tests/
```

### Verify Submission Checklist
```bash
# Run verification script
cat > verify_submission.sh << 'EOF'
#!/bin/bash
echo "Checking submission requirements..."

# Check code structure
[ -d "path_planning_assignment" ] && echo "✓ Code directory exists" || echo "✗ Code directory missing"

# Check README
[ -f "README.md" ] && echo "✓ README.md exists" || echo "✗ README.md missing"

# Check test files
[ -d "test" ] && echo "✓ Test directory exists" || echo "✗ Test directory missing"

# Check for required modules
[ -f "path_planning_assignment/path_smoothing.py" ] && echo "✓ path_smoothing.py exists" || echo "✗ path_smoothing.py missing"
[ -f "path_planning_assignment/trajectory_generator.py" ] && echo "✓ trajectory_generator.py exists" || echo "✗ trajectory_generator.py missing"
[ -f "path_planning_assignment/controller.py" ] && echo "✓ controller.py exists" || echo "✗ controller.py missing"
[ -f "path_planning_assignment/follower_node.py" ] && echo "✓ follower_node.py exists" || echo "✗ follower_node.py missing"

# Check launch file
[ -f "path_planning_assignment/launch/full_system.launch.py" ] && echo "✓ Launch file exists" || echo "✗ Launch file missing"

echo "Verification complete!"
EOF

chmod +x verify_submission.sh
./verify_submission.sh
```

---

## 🎓 Quick Reference: Complete Workflow

```bash
# ==== SETUP (One-time) ====
cd ~/ros2_ws
colcon build --packages-select path_planning_assignment
source install/setup.bash

# ==== RUN SYSTEM ====
# Terminal 1:
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2:
ros2 launch path_planning_assignment full_system.launch.py

# Terminal 3:
ros2 run rviz2 rviz2 -d src/path_planning_assignment/path_planning_assignment/rviz/path_visualizer_config.rviz

# ==== RECORD DATA ====
# Terminal 4:
ros2 bag record /odom /cmd_vel -o trajectory_data

# ==== TEST ====
python3 -m pytest src/path_planning_assignment/test/ -v

# ==== ANALYZE & PLOT ====
python3 src/path_planning_assignment/analyze_and_plot.py --bag trajectory_data --output plots/

# ==== VIEW RESULTS ====
xdg-open plots/trajectory_comparison.png
xdg-open plots/velocity_profiles.png
xdg-open plots/tracking_error.png
```

---

## 📚 Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **TurtleBot3 Manual**: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Matplotlib Gallery**: https://matplotlib.org/stable/gallery/
- **pytest Documentation**: https://docs.pytest.org/

---

**Need Help?** Open an issue on GitHub or contact the maintainer.
