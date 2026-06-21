# TurtleBot4 Hospital Logistics Robot

Autonomous indoor delivery system for hospitals. Addresses NHS staff shortages by automating logistics tasks (specimen transport, supply delivery). Built on TurtleBot4 in ROS 2 Humble with Nav2 navigation stack.

**Problem:** NHS England has ~100,700 vacancies (6.4% nursing rate). RCN 2024 survey shows only 1/3 of nursing shifts adequately staffed, diverting 15-20% of nurse time to repetitive logistics work.

**Solution:** Autonomous robot that navigates a hospital, reaches 18 named destinations (wards, labs, pharmacy, ER), executes multi-stop queues, and handles recovery from obstacles.

---

## Quick Start

**Prerequisites:** ROS 2 Humble, Nav2, TurtleBot4 packages, Ignition Gazebo, ros_gz_bridge

**Step 1: Build the package**
```bash
cd autonomous_ws
colcon build --symlink-install
source install/setup.bash
```

**Step 2: Launch Gazebo + Nav2 + RViz**
Open a **new terminal (Terminal 1)** and run:
```bash
# Terminal 1
ros2 launch autonomous_bot final_navigation.launch.py
```

**Step 3: Set initial pose in RViz**
Open a **second new terminal (Terminal 2)** and do this in RViz:
- Click "2D Pose Estimate" tool in RViz
- Click at robot spawn position (4.6, -9.6 on the map)
- Drag the arrow to set yaw orientation
- This initialises AMCL particles at the correct location

**Step 4: Launch mission dashboard**
Open a **third new terminal (Terminal 3)** and run:
```bash
# Terminal 3
ros2 run autonomous_bot room_navigatorup
```

**Step 5: Execute missions**
- Click room buttons in the GUI to add destinations to the queue
- Use drag arrows (↑ ↓) to reorder destinations
- Click "Start" to begin executing the mission queue
- Watch live ETA, recovery count, and status updates in real-time
- Click "Pause / Resume" to hold execution; "Cancel Current" to skip a goal

---

## Architecture

**3 layers:**
1. **Sensing** — Gazebo → ros_gz_bridge → /scan_raw
2. **Nav2** — AMCL localisation + NavFn planner + Regulated Pure Pursuit controller (0.26 m/s)
3. **Dashboard** — Python/Tkinter GUI (652 lines) with queue, scheduler, looping delivery, live ETA

**Files:**
- `room_navigatorup.py` (652 lines) — mission dashboard with multi-threading
- `room_navigator.py` (98 lines) — v1 prototype
- `final_navigation.launch.py` (195 lines) — production launch
- `nav2_params.yaml` — Nav2 tuning (AMCL, planner, controller)
- `room_poses.yaml` — 18 room definitions (x, y, yaw)

---

## References

- NHS Workforce Statistics (Dec 2024): https://digital.nhs.uk/data-and-information/publications/statistical/nhs-workforce-statistics/december-2024
- NHS Vacancy Statistics: https://digital.nhs.uk/data-and-information/publications/statistical/nhs-vacancies-survey
- The King's Fund (Staff Shortages): https://www.kingsfund.org.uk/insight-and-analysis/data-and-charts/staff-shortages
- RCN "Last Shift" 2024: https://www.rcn.org.uk/news-and-events/Press-Releases/critical-nursing-shortages-leave-patients-unsafe-010724
- Lord Darzi Investigation: https://www.gov.uk/government/publications/independent-investigation-of-the-nhs-in-england
- ROS 2 Nav2: https://docs.nav2.org/
- TurtleBot4: https://clearpathrobotics.com/turtlebot-4/
