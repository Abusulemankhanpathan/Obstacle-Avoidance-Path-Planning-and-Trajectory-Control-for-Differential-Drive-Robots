# Command Reference Guide

Complete command reference for running, testing, and plotting results with YAML-based configuration.

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

# Build package with symlink-install for YAML hot-reload
colcon build --packages-select path_planning_assignment --symlink-install
source install/setup.bash

# Set environment variables (add to ~/.bashrc)
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Running the System

### Method 1: Complete Launch File (Recommended)
```bash
# Launch everything: Gazebo + Follower + Visualizer + RViz
cd ~/ros2_ws
source install/setup.bash
ros2 launch path_planning_assignment full_system.launch.py
```

**What This Launches:**
- Gazebo with TurtleBot3 World
- RViz with pre-configured visualization
- Follower node (with YAML parameters)
- Visualizer node (with YAML parameters)

### Method 2: Manual Launch (Step-by-Step with YAML)

**Terminal 1: Start Gazebo simulation**
```bash
cd ~/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2: Run follower node with YAML parameters**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run path_planning_assignment follower_node --ros-args \
  --params-file install/path_planning_assignment/share/path_planning_assignment/config/controller_params.yaml
```

**Terminal 3: Run visualizer with YAML parameters**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run path_planning_assignment visualizer --ros-args \
  --params-file install/path_planning_assignment/share/path_planning_assignment/config/controller_params.yaml
```

**Terminal 4: Start RViz with pre-configured settings**
```bash
cd ~/ros2_ws
source install/setup.bash
rviz2 -d install/path_planning_assignment/share/path_planning_assignment/rviz/path_planner_config.rviz
```

---

## ⚙️ YAML Configuration Commands

### View Current Configuration
```bash
# View the YAML config file
cd ~/ros2_ws/src/path_planning_assignment/path_planning_assignment/config
cat controller_params.yaml

# Or with syntax highlighting
batcat controller_params.yaml  # Install: sudo apt install bat
```

### Edit Configuration (No Rebuild Required!)
```bash
# Edit YAML parameters
cd ~/ros2_ws/src/path_planning_assignment/path_planning_assignment/config
nano controller_params.yaml

# After editing, just relaunch - NO colcon build needed!
cd ~/ros2_ws
source install/setup.bash
ros2 launch path_planning_assignment full_system.launch.py
```

### Common Parameter Modifications

**Change Trajectory (Edit waypoints):**
```bash
nano path_planning_assignment/config/controller_params.yaml

# Modify waypoints_flat section:
waypoints_flat: [
  0.0, 0.0,    # Start
  2.0, 0.0,    # Point 1
  2.0, 2.0,    # Point 2
  0.0, 2.0,    # Point 3
  0.0, 0.0     # Back to start
]
```

**Tune Controller Performance:**
```bash
# For smoother motion (slower):
max_linear_vel: 0.10           # Reduce from 0.15
lookahead_distance: 0.15       # Increase from 0.1

# For faster motion (less smooth):
max_linear_vel: 0.22           # Increase from 0.15
lookahead_distance: 0.08       # Decrease from 0.1

# For sharper turns:
max_angular_vel: 2.0           # Increase from 1.5
kp_ang: 2.5                    # Increase from 2.0

# For smoother turns:
max_angular_vel: 1.0           # Decrease from 1.5
kp_ang: 1.5                    # Decrease from 2.0
```

**Adjust Visualization:**
```bash
# Make path lines thicker/thinner:
planned_path_width: 0.10       # Increase from 0.07
actual_path_width: 0.20        # Increase from 0.15

# Make waypoint spheres larger/smaller:
waypoint_radius: 0.12          # Increase from 0.08

# Increase actual path resolution:
trail_min_distance: 0.01       # Decrease from 0.02 (more points)
```

### Backup and Restore Configurations
```bash
# Backup current config
cd ~/ros2_ws/src/path_planning_assignment/path_planning_assignment/config
cp controller_params.yaml controller_params.yaml.backup

# Create different configurations
cp controller_params.yaml fast_mode.yaml
cp controller_params.yaml precise_mode.yaml

# Restore backup
cp controller_params.yaml.backup controller_params.yaml
```

### Use Alternative Config Files
```bash
# Launch with custom config file
ros2 run path_planning_assignment follower_node --ros-args \
  --params-file ~/my_custom_params.yaml

# Or modify launch file to use different config
nano path_planning_assignment/launch/full_system.launch.py
# Change: config_file = os.path.join(pkg_dir, "config", "my_custom_config.yaml")
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
python3 -m pytest src/path_planning_assignment/test/test_smoothing.py -v

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
python3 -m pytest src/path_planning_assignment/test/test_smoothing.py::TestPathSmoothing::test_circular_arc_curvature -v
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
ros2 bag record /odom /cmd_vel /path_visualization /path_to_visualize -o trajectory_data

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
plt.plot(x, y, 'b-', linewidth=2, label='Actual Path')
plt.scatter(x.iloc[0], y.iloc[0], c='green', s=200, label='Start', zorder=5)
plt.scatter(x.iloc[-1], y.iloc[-1], c='red', s=200, label='End', zorder=5)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.savefig('trajectory.png', dpi=300, bbox_inches='tight')
print("✓ Saved: trajectory.png")
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
ax1.set_ylabel('Linear Velocity (m/s)', fontsize=12)
ax1.set_title('Velocity Profiles', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.set_xlim(time.iloc[0], time.iloc[-1])

ax2.plot(time, w, 'r-', linewidth=2)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(time.iloc[0], time.iloc[-1])

plt.tight_layout()
plt.savefig('velocities.png', dpi=300, bbox_inches='tight')
print("✓ Saved: velocities.png")
EOF
```

---

## 🔍 Debugging Commands

### Check System Status
```bash
# List all active nodes
ros2 node list

# Should show:
# /gazebo
# /trajectory_follower
# /path_visualizer
# /rviz2
# /robot_state_publisher

# Check if topics are publishing
ros2 topic list

# Monitor topic frequency
ros2 topic hz /odom
ros2 topic hz /cmd_vel
ros2 topic hz /path_visualization
ros2 topic hz /path_to_visualize

# Echo topic data
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /path_visualization --once

# Check node information
ros2 node info /trajectory_follower
ros2 node info /path_visualizer
```

### Check YAML Parameters at Runtime
```bash
# List all parameters for follower node
ros2 param list /trajectory_follower

# Get specific parameter value
ros2 param get /trajectory_follower lookahead_distance
ros2 param get /trajectory_follower max_linear_vel
ros2 param get /trajectory_follower waypoints_flat

# Dump all parameters to YAML file
ros2 param dump /trajectory_follower > current_params.yaml
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
# 3. Add → TF
# 4. Add → MarkerArray → Topic: /path_visualization
# 5. Add → MarkerArray → Topic: /path_to_visualize
# 6. Enable all namespaces in each MarkerArray
# 7. Save Config → Save to rviz/path_planner_config.rviz
```

### Performance Profiling
```bash
# Monitor CPU usage
ros2 topic hz /odom --window 100
top -p $(pgrep -f follower_node)

# Check message latency
ros2 topic delay /cmd_vel

# Analyze computational cost
python3 -m cProfile -o profile.stats -m path_planning_assignment.follower_node
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
    "Mean Linear Velocity": f"{np.mean(v_cmd):.3f} m/s",
    "Configuration": "YAML-based parameters"
}

# Save
with open('metrics.json', 'w') as f:
    json.dump(metrics, f, indent=4)

print("Performance Metrics:")
print(json.dumps(metrics, indent=2))
print("\n✓ Saved to: metrics.json")
EOF
```

---

## 🐛 Common Issues & Solutions

### Issue: Robot doesn't move
```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Verify TurtleBot3 model is set
echo $TURTLEBOT3_MODEL

# Check if follower node is running
ros2 node list | grep trajectory_follower

# Check if waypoints are loaded from YAML
ros2 param get /trajectory_follower waypoints_flat

# Restart system
pkill -f gazebo
ros2 launch path_planning_assignment full_system.launch.py
```

### Issue: YAML parameters not loading
```bash
# Check if YAML file exists
ls -la install/path_planning_assignment/share/path_planning_assignment/config/

# Verify YAML syntax
cd ~/ros2_ws/src/path_planning_assignment/path_planning_assignment/config
python3 -c "import yaml; yaml.safe_load(open('controller_params.yaml'))"

# If syntax error, it will show line number
# Fix the YAML and relaunch (no rebuild needed with --symlink-install)

# Check current parameters at runtime
ros2 param dump /trajectory_follower
```

### Issue: RViz shows nothing / missing path visualization
```bash
# Verify Fixed Frame is "odom"
# In RViz: Global Options → Fixed Frame → "odom"

# Check if both marker topics are added
# Left panel should show TWO MarkerArray displays:
# - /path_to_visualize
# - /path_visualization

# Check if namespaces are enabled
# Expand each MarkerArray → Namespaces → Enable all checkboxes

# Verify topics are publishing
ros2 topic echo /path_visualization --once
ros2 topic echo /path_to_visualize --once

# Check message rate
ros2 topic hz /path_visualization
ros2 topic hz /path_to_visualize

# Restart visualizer
pkill -f visualizer
ros2 run path_planning_assignment visualizer --ros-args \
  --params-file install/path_planning_assignment/share/path_planning_assignment/config/controller_params.yaml
```

### Issue: Waypoint colors not showing correctly
```bash
# This is a RViz namespace issue
# In RViz left panel:
# 1. Find MarkerArray (/path_visualization)
# 2. Expand "Namespaces" section
# 3. Enable ALL namespaces:
#    ☑ planned_path
#    ☑ actual_path
#    ☑ start_point
#    ☑ end_point
#    ☑ waypoints

# Save the configuration
# File → Save Config As → path_planner_config.rviz
```

### Issue: Tests fail with import errors
```bash
# Set PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/path_planning_assignment

# Or install package in development mode
cd ~/ros2_ws/src/path_planning_assignment
pip3 install -e .

# Or rebuild
cd ~/ros2_ws
colcon build --packages-select path_planning_assignment --symlink-install
source install/setup.bash
```

### Issue: Build fails after YAML changes
```bash
# YAML changes don't require rebuild if you used --symlink-install
# Just relaunch!

# But if you changed Python code:
cd ~/ros2_ws
colcon build --packages-select path_planning_assignment --symlink-install
source install/setup.bash
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
pip3 install pdoc3
pdoc --html --output-dir docs path_planning_assignment

# 6. Create submission structure
mkdir -p submission/{code,documentation,videos,plots,tests,config}
cp -r path_planning_assignment submission/code/
cp README.md Command_reference_Guide.md submission/documentation/
cp demo_video.mp4 submission/videos/
cp -r plots/* submission/plots/
cp test_report.html submission/tests/
cp path_planning_assignment/config/controller_params.yaml submission/config/
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
[ -f "path_planning_assignment/follower_node.py" ] && echo "✓ follower_node.py exists" || echo "✗ follower_node.py missing"
[ -f "path_planning_assignment/visualizer.py" ] && echo "✓ visualizer.py exists" || echo "✗ visualizer.py missing"

# Check YAML config
[ -f "path_planning_assignment/config/controller_params.yaml" ] && echo "✓ YAML config exists" || echo "✗ YAML config missing"

# Check launch file
[ -f "path_planning_assignment/launch/full_system.launch.py" ] && echo "✓ Launch file exists" || echo "✗ Launch file missing"

# Check RViz config
[ -f "path_planning_assignment/rviz/path_planner_config.rviz" ] && echo "✓ RViz config exists" || echo "✗ RViz config missing"

echo ""
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
colcon build --packages-select path_planning_assignment --symlink-install
source install/setup.bash

# ==== CONFIGURE (Edit YAML - No rebuild needed!) ====
nano src/path_planning_assignment/path_planning_assignment/config/controller_params.yaml

# ==== RUN SYSTEM (Single Command) ====
ros2 launch path_planning_assignment full_system.launch.py

# ==== OR RUN INDIVIDUALLY ====
# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Follower with YAML
ros2 run path_planning_assignment follower_node --ros-args \
  --params-file install/path_planning_assignment/share/path_planning_assignment/config/controller_params.yaml

# Terminal 3: Visualizer with YAML
ros2 run path_planning_assignment visualizer --ros-args \
  --params-file install/path_planning_assignment/share/path_planning_assignment/config/controller_params.yaml

# Terminal 4: RViz
rviz2 -d install/path_planning_assignment/share/path_planning_assignment/rviz/path_planner_config.rviz

# ==== RECORD DATA ====
ros2 bag record /odom /cmd_vel /path_visualization /path_to_visualize -o trajectory_data

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
- **ROS2 Parameters Guide**: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html
- **TurtleBot3 Manual**: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Matplotlib Gallery**: https://matplotlib.org/stable/gallery/
- **pytest Documentation**: https://docs.pytest.org/
- **YAML Syntax**: https://yaml.org/spec/1.2/spec.html

---

## 🎯 Pro Tips

1. **Use `--symlink-install`** when building - YAML changes won't need rebuild
2. **Save RViz configs** after setting up displays correctly
3. **Create multiple YAML profiles** for different scenarios (fast, precise, demo)
4. **Monitor topics** with `ros2 topic hz` to ensure everything is publishing
5. **Check namespaces** in RViz MarkerArray - they must all be enabled
6. **Use `ros2 param dump`** to backup current running parameters
7. **Test YAML syntax** with Python before launching: `python3 -c "import yaml; yaml.safe_load(open('config.yaml'))"`

---

**Need Help?** Open an issue on GitHub or contact the maintainer.
