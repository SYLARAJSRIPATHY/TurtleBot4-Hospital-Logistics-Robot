#!/usr/bin/env python3

import os
import csv
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOM_YAML_PATH = os.path.normpath(os.path.join(_THIS_DIR, '..', 'config', 'room_poses.yaml'))
LOG_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', 'runs'))
LOG_PATH = os.path.join(LOG_DIR, 'mission_log.csv')

NAV2_ACTION_NAME = 'navigate_to_pose'


NOMINAL_CRUISE_SPEED = 0.26

def _yaw_to_quat(yaw: float) -> tuple:

    if abs(yaw) > 2 * math.pi:
        yaw = math.radians(yaw)
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def load_rooms(yaml_path: str) -> dict:

    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"room_poses.yaml not found at: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    node = data.get('rooms', data) if isinstance(data, dict) else {}
    if not isinstance(node, dict):
        raise ValueError('Unsupported room_poses.yaml format')
    out = {}
    for name, entry in node.items():
        if isinstance(entry, dict) and 'x' in entry and 'y' in entry:
            x = float(entry.get('x', 0.0))
            y = float(entry.get('y', 0.0))
            z = float(entry.get('z', 0.0))
            yaw = float(entry.get('yaw', 0.0))
            qx, qy, qz, qw = _yaw_to_quat(yaw)
        else:
            pos = entry.get('position', {})
            ori = entry.get('orientation', {})
            x = float(pos.get('x', 0.0))
            y = float(pos.get('y', 0.0))
            z = float(pos.get('z', 0.0))
            if 'yaw' in entry:
                qx, qy, qz, qw = _yaw_to_quat(float(entry['yaw']))
            else:
                qx = float(ori.get('x', 0.0))
                qy = float(ori.get('y', 0.0))
                qz = float(ori.get('z', 0.0))
                qw = float(ori.get('w', 1.0))
        out[str(name)] = {
            'position': {'x': x, 'y': y, 'z': z},
            'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
        }
    if not out:
        raise ValueError('No rooms parsed from YAML')
    return out


def make_pose_stamped(x: float, y: float, qz: float = 0.0, qw: float = 1.0, frame: str = 'map') -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


class NavClient(Node):
    def __init__(self) -> None:
        super().__init__('room_navigator_gui')
        self.cli: ActionClient = ActionClient(self, NavigateToPose, NAV2_ACTION_NAME)
        self.current_goal_handle = None
        self.current_result_status = None
        self._result_event = threading.Event()
        self._feedback_cb = None
        self._cancel_lock = threading.Lock()
        self.get_logger().info('Waiting for Nav2 action server...')
        self.cli.wait_for_server()
        self.get_logger().info('Nav2 action server connected.')

    def send_goal(self, pose: PoseStamped, feedback_cb=None) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._feedback_cb = feedback_cb
        self._result_event.clear()
        send_future = self.cli.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        def _on_sent(fut):
            goal_handle = fut.result()
            self.current_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by server.')
                self.current_result_status = GoalStatus.STATUS_ABORTED
                self._result_event.set()
                return
            self.get_logger().info('Goal accepted.')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)
        send_future.add_done_callback(_on_sent)

    def _on_feedback(self, feedback_msg):
        if self._feedback_cb:
            try:
                self._feedback_cb(feedback_msg.feedback)
            except Exception as exc:
                self.get_logger().warn(f'Feedback callback error: {exc}')

    def _on_result(self, fut):
        try:
            res = fut.result()
            self.current_result_status = res.status
        except Exception as exc:
            self.get_logger().error(f'Result error: {exc}')
            self.current_result_status = GoalStatus.STATUS_ABORTED
        finally:
            self._result_event.set()

    def wait_for_result(self, timeout: float | None = None) -> int:
        self._result_event.wait(timeout=timeout)
        return self.current_result_status

    def cancel(self) -> None:
        with self._cancel_lock:
            if self.current_goal_handle is None:
                return
            self.get_logger().info('Canceling current goal...')
            fut = self.current_goal_handle.cancel_goal_async()
            fut.add_done_callback(lambda _: self.get_logger().info('Cancel request sent.'))


class App(tk.Tk):
    """Tkinter-based dashboard for sending and monitoring Nav2 missions."""
    def __init__(self, node: NavClient, rooms: dict) -> None:
        super().__init__()
        self.title('Room Navigator – Hospital Demo')
        self.geometry('960x680')
        self.node = node
        self.rooms = rooms
        self.successes: int = 0
        self.attempts: int = 0
        self.replans: int = 0
        self.recoveries: int = 0
        self.cancels: int = 0
        self.queue: list[str] = []
        self.running: bool = False
        self.paused: bool = False
        self._paused_room: str | None = None
        self._current_room: str | None = None
        self._goal_start_ts: float | None = None
        self._last_dist: float | None = None
        self.scheduled_jobs: list[dict] = []   
        self.loop_running: bool = False
        self.clock_var = tk.StringVar(value=time.strftime("%H:%M:%S"))
        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._ros_thread.start()
        self._build_ui()
        self._ensure_logger()

    def _spin_ros(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            self.node.destroy_node()

    def _ensure_logger(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        if not os.path.exists(LOG_PATH):
            with open(LOG_PATH, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'room', 'success', 'duration_sec', 'replans', 'recoveries', 'canceled'])


    def _build_ui(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text='clock').pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.clock_var, width=8).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(top, text='Destination:').pack(side=tk.LEFT)
        self.room_var = tk.StringVar()
        self.room_combo = ttk.Combobox(top, textvariable=self.room_var, width=46)
        self._refresh_room_combo()
        self.room_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text='Add to Queue', command=self._add_to_queue).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text='Reload Rooms', command=self._reload_rooms).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text='Clear Queue', command=self._clear_queue).pack(side=tk.LEFT, padx=4)
        mid = ttk.Frame(self, padding=8)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        left = ttk.Frame(mid)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Label(left, text='Mission Queue').pack(anchor='w')
        self.queue_list = tk.Listbox(left, height=10)
        self.queue_list.pack(fill=tk.BOTH, expand=True, pady=4)
        qbtns = ttk.Frame(left)
        qbtns.pack(fill=tk.X, pady=4)
        ttk.Button(qbtns, text='↑', width=3, command=lambda: self._move_queue(-1)).pack(side=tk.LEFT)
        ttk.Button(qbtns, text='↓', width=3, command=lambda: self._move_queue(1)).pack(side=tk.LEFT, padx=4)
        ttk.Button(qbtns, text='Remove', command=self._remove_selected).pack(side=tk.LEFT, padx=4)
        ctrls = ttk.Frame(left)
        ctrls.pack(fill=tk.X, pady=6)
        ttk.Button(ctrls, text='Start', command=self._start_queue).pack(side=tk.LEFT)
        ttk.Button(ctrls, text='Pause / Resume', command=self._pause_queue).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrls, text='Cancel Current', command=self._cancel_current).pack(side=tk.LEFT, padx=4)
        sched = ttk.LabelFrame(left, text='Mission Scheduler (HH:MM → Room)', padding=8)
        sched.pack(fill=tk.X, pady=(12, 6))
        self.sched_time = tk.StringVar(value=time.strftime("%H:%M"))
        self.sched_room = tk.StringVar()
        ttk.Button(sched, text='Add', command=self._add_schedule).pack(side=tk.LEFT)
        ttk.Entry(sched, textvariable=self.sched_time, width=7).pack(side=tk.RIGHT, padx=6)
        ttk.Label(sched, text='at').pack(side=tk.RIGHT, padx=4)
        self.sched_room_combo = ttk.Combobox(sched, textvariable=self.sched_room, width=24,values=sorted(self.rooms.keys()))
        self.sched_room_combo.pack(side=tk.RIGHT)
        self.sched_list = tk.Listbox(sched, height=4)
        self.sched_list.pack(fill=tk.X, pady=6)
        loop = ttk.LabelFrame(left, text='Looping Delivery (From ↔ To, count)', padding=8)
        loop.pack(fill=tk.X, pady=(6, 6))
        self.loop_from = tk.StringVar()
        self.loop_to = tk.StringVar()
        self.loop_count = tk.IntVar(value=2)
        ttk.Label(loop, text='From').grid(row=0, column=0, sticky='w')
        ttk.Combobox(loop, textvariable=self.loop_from, width=18, values=sorted(self.rooms.keys())).grid(row=0, column=1, padx=4)
        ttk.Label(loop, text='To').grid(row=0, column=2, sticky='w')
        ttk.Combobox(loop, textvariable=self.loop_to, width=18, values=sorted(self.rooms.keys())).grid(row=0, column=3, padx=4)
        ttk.Label(loop, text='Loops').grid(row=0, column=4, sticky='w')
        ttk.Spinbox(loop, from_=1, to=999, textvariable=self.loop_count, width=5).grid(row=0, column=5, padx=4)
        ttk.Button(loop, text='Start Loops', command=self._start_loops).grid(row=0, column=6, padx=6)
        right = ttk.Frame(mid, padding=(12, 0))
        right.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(right, text='Status').grid(row=0, column=0, sticky='w')
        self.status_var = tk.StringVar(value='Idle')
        ttk.Label(right, textvariable=self.status_var, foreground='blue').grid(row=0, column=1, sticky='w', padx=8)
        ttk.Label(right, text='ETA').grid(row=1, column=0, sticky='w')
        self.eta_var = tk.StringVar(value='-')
        ttk.Label(right, textvariable=self.eta_var).grid(row=1, column=1, sticky='w', padx=8)
        ttk.Label(right, text='Current Room').grid(row=2, column=0, sticky='w')
        self.curr_room_var = tk.StringVar(value='-')
        ttk.Label(right, textvariable=self.curr_room_var).grid(row=2, column=1, sticky='w', padx=8)
        ttk.Separator(right, orient='horizontal').grid(row=3, column=0, columnspan=2, sticky='ew', pady=6)
        ttk.Label(right, text='Metrics (session)').grid(row=4, column=0, sticky='w')
        self.sr_var = tk.StringVar(value='0 / 0 (0%)')
        ttk.Label(right, text='Success Rate').grid(row=5, column=0, sticky='w')
        ttk.Label(right, textvariable=self.sr_var).grid(row=5, column=1, sticky='w', padx=8)
        self.replan_var = tk.StringVar(value='0')
        ttk.Label(right, text='Replans').grid(row=6, column=0, sticky='w')
        ttk.Label(right, textvariable=self.replan_var).grid(row=6, column=1, sticky='w', padx=8)
        self.recovery_var = tk.StringVar(value='0')
        ttk.Label(right, text='Recoveries').grid(row=7, column=0, sticky='w')
        ttk.Label(right, textvariable=self.recovery_var).grid(row=7, column=1, sticky='w', padx=8)
        self.cancel_var = tk.StringVar(value='0')
        ttk.Label(right, text='Cancels').grid(row=8, column=0, sticky='w')
        ttk.Label(right, textvariable=self.cancel_var).grid(row=8, column=1, sticky='w', padx=8)
        ttk.Separator(right, orient='horizontal').grid(row=9, column=0, columnspan=2, sticky='ew', pady=6)
        ttk.Button(right, text='Export CSV…', command=self._export_csv).grid(
            row=10, column=0, columnspan=2, pady=8, sticky='ew'
        )
        right.grid_columnconfigure(1, weight=1)
        bottom = ttk.Frame(self, padding=8)
        bottom.pack(side=tk.BOTTOM, fill=tk.BOTH)
        ttk.Label(bottom, text='Mission Log').pack(anchor='w')
        cols = ('time', 'room', 'success', 'duration')
        self.log_tree = ttk.Treeview(bottom, columns=cols, show='headings', height=7)
        for c in cols:
            self.log_tree.heading(c, text=c.capitalize())
            self.log_tree.column(c, stretch=True, width=160)
        self.log_tree.pack(fill=tk.BOTH, expand=True)
        self.after(200, self._tick)
        self.after(1000, self._clock_tick)
        self.after(1000, self._scheduler_tick)
    def _refresh_room_combo(self) -> None:
        names = sorted(self.rooms.keys())
        self.room_combo['values'] = names
        if names:
            self.room_combo.current(0)
    def _add_to_queue(self) -> None:
        name = self.room_var.get()
        if not name:
            return
        self.queue.append(name)
        self.queue_list.insert(tk.END, name)

    def _clear_queue(self) -> None:
        self.queue.clear()
        self.queue_list.delete(0, tk.END)

    def _move_queue(self, delta: int) -> None:
        sel = self.queue_list.curselection()
        if not sel:
            return
        i = sel[0]
        j = max(0, min(len(self.queue) - 1, i + delta))
        if i == j:
            return
        self.queue[i], self.queue[j] = self.queue[j], self.queue[i]
        self.queue_list.delete(0, tk.END)
        for r in self.queue:
            self.queue_list.insert(tk.END, r)
        self.queue_list.selection_set(j)

    def _remove_selected(self) -> None:
        sel = self.queue_list.curselection()
        if not sel:
            return
        i = sel[0]
        self.queue.pop(i)
        self.queue_list.delete(i)
    def _start_queue(self) -> None:
        if self.running:
            messagebox.showinfo('Info', 'Queue already running.')
            return
        if not self.queue:
            messagebox.showwarning('Warning', 'Queue is empty.')
            return
        self.running = True
        self.paused = False
        threading.Thread(target=self._run_queue, daemon=True).start()

    def _pause_queue(self) -> None:
        if not self.running:
            return
        self.paused = not self.paused
        if self.paused:
            if self.node.current_goal_handle is not None and self._current_room:
                self.node.cancel()
                self.queue.insert(0, self._current_room)
                self.queue_list.insert(0, self._current_room)
                self._paused_room = self._current_room
            self.status_var.set('Paused')
        else:
            if self._paused_room:
                if not self.queue or self.queue[0] != self._paused_room:
                    self.queue.insert(0, self._paused_room)
                    self.queue_list.insert(0, self._paused_room)
                self._paused_room = None
            self.status_var.set('Navigating')
            if not self.running:
                self.running = True
                threading.Thread(target=self._run_queue, daemon=True).start()

    def _cancel_current(self) -> None:
        if self.node.current_goal_handle is not None:
            self.node.cancel()
            self.cancels += 1
            self.cancel_var.set(str(self.cancels))

    def _run_queue(self) -> None:
        while self.queue and self.running:
            if self.paused:
                time.sleep(0.1)
                continue
            room = self.queue.pop(0)
            self.queue_list.delete(0)
            self._send_room_goal(room)
        self.running = False
        if not self.queue:
            self.status_var.set('Idle')

    def _add_schedule(self) -> None:
        tstr = self.sched_time.get().strip()
        room = (self.sched_room.get() or self.sched_room_combo.get() or "").strip()
        if not tstr or not room:
            messagebox.showwarning('Missing', 'Provide both time and room.')
            return
        try:
            time.strptime(tstr, "%H:%M") 
        except ValueError:
            messagebox.showerror('Invalid time', 'Use 24h HH:MM (e.g., 14:35).')
            return
        job = {"time": tstr, "room": room, "dispatched": False}
        self.scheduled_jobs.append(job)
        self.sched_list.insert(tk.END, f"{tstr} → {room}")

    def _scheduler_tick(self) -> None:
        now_hm = time.strftime("%H:%M")
        for job in self.scheduled_jobs:
            if not job["dispatched"] and job["time"] <= now_hm:
                self.queue.append(job["room"])
                self.queue_list.insert(tk.END, job["room"])
                job["dispatched"] = True
                if not self.running:
                    self._start_queue()
        self.after(1000, self._scheduler_tick)

    def _start_loops(self) -> None:
        if self.loop_running:
            messagebox.showinfo('Info', 'Loop is already running.')
            return
        frm = (self.loop_from.get() or "").strip()
        to = (self.loop_to.get() or "").strip()
        try:
            count = int(self.loop_count.get() or 0)
        except Exception:
            count = 0
        if not frm or not to or count <= 0:
            messagebox.showwarning('Missing', 'Select From, To and set Loops ≥ 1.')
            return
        seq = []
        for _ in range(count):
            seq.extend([frm, to])
        for r in seq:
            self.queue.append(r)
            self.queue_list.insert(tk.END, r)
        if not self.running:
            self._start_queue()
        self.loop_running = True

        def _watch_loop():
            while self.running or self.queue:
                time.sleep(0.2)
            self.loop_running = False
        threading.Thread(target=_watch_loop, daemon=True).start()

    def _send_room_goal(self, room_name: str) -> None:
        pose = self.rooms[room_name]
        x = pose['position']['x']
        y = pose['position']['y']
        qz = pose['orientation']['z']
        qw = pose['orientation']['w']
        goal = make_pose_stamped(x, y, qz, qw)
        self._current_room = room_name
        self.curr_room_var.set(room_name)
        self.status_var.set(f'Navigating → {room_name}')
        self.eta_var.set('…')
        self.replans = 0
        self.recoveries = 0
        self.replan_var.set('0')
        self.recovery_var.set('0')
        self._last_dist = None
        self._goal_start_ts = time.time()
        self.attempts += 1
        self._update_success_rate()
        self.node.send_goal(goal, feedback_cb=self._on_feedback)
        status = self.node.wait_for_result()
        duration = time.time() - self._goal_start_ts if self._goal_start_ts else 0.0
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            self.successes += 1
            self._toast(f'Reached {room_name} in {duration:.1f}s ✔')
        else:
            self._toast(f'Failed to reach {room_name} ✖')
        self._update_success_rate()
        self._append_log(room_name, success, duration, self.replans, self.recoveries, canceled=False)
        self._current_room = None
        self.curr_room_var.set('-')
        self.status_var.set('Idle')
        self.eta_var.set('-')

    def _on_feedback(self, fb: NavigateToPose.Feedback) -> None:
        try:
            dist = float(getattr(fb, 'distance_remaining', float('nan')))
        except Exception:
            dist = float('nan')
        try:
            rec = int(getattr(fb, 'number_of_recoveries', 0))
        except Exception:
            rec = 0
        if rec > self.recoveries:
            self.recoveries = rec
            self.recovery_var.set(str(self.recoveries))
        if not math.isnan(dist):
            speed = max(NOMINAL_CRUISE_SPEED * 0.6, 0.05)
            eta = dist / speed
            self.eta_var.set(f'{eta:.0f}s')
        if self._last_dist is not None and not math.isnan(dist):
            if dist - self._last_dist > 0.8:
                self.replans += 1
                self.replan_var.set(str(self.replans))
        self._last_dist = dist

    def _update_success_rate(self) -> None:
        pct = (self.successes / self.attempts * 100.0) if self.attempts else 0.0
        self.sr_var.set(f'{self.successes} / {self.attempts} ({pct:.0f}%)')

    def _append_log(self, room: str, success: bool, duration: float, replans: int, recoveries: int, canceled: bool) -> None:
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        self.log_tree.insert('', tk.END, values=(ts, room, 'yes' if success else 'no', f'{duration:.1f}s'))
        with open(LOG_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([ts, room, int(success), f'{duration:.2f}', replans, recoveries, int(canceled)])

    def _export_csv(self) -> None:
        if not os.path.exists(LOG_PATH):
            messagebox.showwarning('No log', 'No log file yet.')
            return
        dest = filedialog.asksaveasfilename(defaultextension='.csv', initialfile='mission_log.csv')
        if not dest:
            return
        with open(LOG_PATH, 'rb') as src, open(dest, 'wb') as dst:
            dst.write(src.read())
        messagebox.showinfo('Exported', f'Saved to {dest}')

    def _reload_rooms(self) -> None:
        try:
            new_rooms = load_rooms(ROOM_YAML_PATH)
            self.rooms = new_rooms
            self._refresh_room_combo()
            messagebox.showinfo('Rooms reloaded', f'Loaded {len(self.rooms)} rooms.')
        except Exception as exc:
            messagebox.showerror('Reload failed', str(exc))

    def _tick(self) -> None:
        self.after(200, self._tick)

    def _clock_tick(self) -> None:
        self.clock_var.set(time.strftime("%H:%M:%S"))
        self.after(1000, self._clock_tick)

    def _toast(self, text: str) -> None:
        self.status_var.set(text)
        self.after(2500, lambda: self.status_var.set('Navigating' if self.running and not self.paused else 'Idle'))

def main() -> None:
    rclpy.init()
    rooms = load_rooms(ROOM_YAML_PATH)
    node = NavClient()
    app = App(node, rooms)
    app.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
