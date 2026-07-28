#!/usr/bin/env python3

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import math
import os
from pathlib import Path

PI = math.pi
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI

BASE_RADIUS = 150.0
TOP_RADIUS = 100.0
BASE_ANGLE_OFFSET = 0.0
TOP_ANGLE_OFFSET = PI / 6.0
MIN_LEG_LENGTH = 80.0
MAX_LEG_LENGTH = 200.0
HOME_Z = 150.0

OUTPUT_DIR = Path(__file__).parent / "output"
OUTPUT_DIR.mkdir(exist_ok=True)


def compute_rotation_matrix(phi, theta, psi):
    cp, sp = math.cos(phi), math.sin(phi)
    ct, st = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi), math.sin(psi)
    return np.array([
        [ct * cpsi,  sp * st * cpsi - cp * spsi,  cp * st * cpsi + sp * spsi],
        [ct * spsi,  sp * st * spsi + cp * cpsi,  cp * st * spsi - sp * cpsi],
        [-st,        sp * ct,                     cp * ct]
    ])


def calculate_attachment_points(radius, angle_offset):
    points = np.zeros((6, 3))
    for i in range(6):
        angle = angle_offset + i * PI / 3.0
        points[i, 0] = radius * math.cos(angle)
        points[i, 1] = radius * math.sin(angle)
        points[i, 2] = 0.0
    return points


def solve_ik(pose, base_points, top_points):
    x, y, z, phi, theta, psi = pose
    R = compute_rotation_matrix(phi, theta, psi)
    T = np.array([x, y, z])

    leg_lengths = np.zeros(6)
    for i in range(6):
        top_global = R @ top_points[i] + T
        leg_vec = top_global - base_points[i]
        leg_lengths[i] = np.linalg.norm(leg_vec)
    return leg_lengths


def s_curve(t, duration):
    if duration <= 0:
        return 1.0
    tau = max(0.0, min(1.0, t / duration))
    return tau * tau * (3.0 - 2.0 * tau)


def generate_trajectory(start_pose, end_pose, duration, num_steps):
    poses = []
    for step in range(num_steps + 1):
        t = step * duration / num_steps
        s = s_curve(t, duration)
        pose = np.array([
            start_pose[0] + (end_pose[0] - start_pose[0]) * s,
            start_pose[1] + (end_pose[1] - start_pose[1]) * s,
            start_pose[2] + (end_pose[2] - start_pose[2]) * s,
            start_pose[3] + (end_pose[3] - start_pose[3]) * s,
            start_pose[4] + (end_pose[4] - start_pose[4]) * s,
            start_pose[5] + (end_pose[5] - start_pose[5]) * s,
        ])
        poses.append(pose)
    return poses


def create_3d_animation_gif():
    print("[1/4] Creating 3D animated GIF...")

    base_points = calculate_attachment_points(BASE_RADIUS, BASE_ANGLE_OFFSET)
    top_points = calculate_attachment_points(TOP_RADIUS, TOP_ANGLE_OFFSET)

    num_frames = 120
    fps = 20
    duration = 3.0

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    def animate(frame):
        ax.clear()

        t = frame / num_frames * duration
        x = 40 * math.sin(2 * PI * t / duration)
        y = 30 * math.sin(4 * PI * t / duration)
        z = HOME_Z + 20 * math.sin(2 * PI * t / duration)
        phi = 5 * DEG_TO_RAD * math.sin(2 * PI * t / duration)
        theta = 8 * DEG_TO_RAD * math.sin(4 * PI * t / duration)
        psi = 3 * DEG_TO_RAD * math.sin(2 * PI * t / duration)

        pose = np.array([x, y, z, phi, theta, psi])
        R = compute_rotation_matrix(phi, theta, psi)
        T = np.array([x, y, z])

        top_global = np.array([R @ top_points[i] + T for i in range(6)])

        base_closed = np.vstack([base_points, base_points[0:1]])
        ax.plot(base_closed[:, 0], base_closed[:, 1], base_closed[:, 2],
                'b-', linewidth=2.5, label='Base')

        top_closed = np.vstack([top_global, top_global[0:1]])
        ax.plot(top_closed[:, 0], top_closed[:, 1], top_closed[:, 2],
                'r-', linewidth=2.5, label='Top Platform')

        for i in range(6):
            leg_len = np.linalg.norm(top_global[i] - base_points[i])
            mid = (MIN_LEG_LENGTH + MAX_LEG_LENGTH) / 2
            color = 'g' if abs(leg_len - mid) < 20 else 'orange'
            ax.plot([base_points[i, 0], top_global[i, 0]],
                    [base_points[i, 1], top_global[i, 1]],
                    [base_points[i, 2], top_global[i, 2]],
                    color=color, linewidth=1.5, alpha=0.8)

        ax.scatter(base_points[:, 0], base_points[:, 1], base_points[:, 2],
                   c='blue', s=40, marker='o')
        ax.scatter(top_global[:, 0], top_global[:, 1], top_global[:, 2],
                   c='red', s=40, marker='^')

        ax.set_xlabel('X [mm]', fontsize=11)
        ax.set_ylabel('Y [mm]', fontsize=11)
        ax.set_zlabel('Z [mm]', fontsize=11)
        ax.set_title(f'Stewart Platform - Figure-8 Trajectory\n'
                     f'Pose: ({x:.1f}, {y:.1f}, {z:.1f}) mm | '
                     f'Roll: {phi*RAD_TO_DEG:.1f}° Pitch: {theta*RAD_TO_DEG:.1f}° Yaw: {psi*RAD_TO_DEG:.1f}°',
                     fontsize=10)

        limit = 200
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(0, 300)
        ax.view_init(elev=25, azim=30 + frame * 0.5)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)

    anim = FuncAnimation(fig, animate, frames=num_frames, interval=1000/fps)
    gif_path = OUTPUT_DIR / "stewart_platform_3d_animation.gif"
    anim.save(gif_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"  Saved: {gif_path}")


def create_leg_lengths_plot():
    print("[2/4] Creating leg lengths plot...")

    base_points = calculate_attachment_points(BASE_RADIUS, BASE_ANGLE_OFFSET)
    top_points = calculate_attachment_points(TOP_RADIUS, TOP_ANGLE_OFFSET)

    num_steps = 200
    duration = 4.0
    times = np.linspace(0, duration, num_steps + 1)

    leg_lengths_history = np.zeros((num_steps + 1, 6))

    for i, t in enumerate(times):
        s = s_curve(t, duration)
        x = 50 * math.sin(2 * PI * t / duration)
        y = 50 * math.cos(2 * PI * t / duration)
        z = HOME_Z + 30 * math.sin(4 * PI * t / duration)
        phi = 10 * DEG_TO_RAD * math.sin(2 * PI * t / duration)
        theta = 10 * DEG_TO_RAD * math.cos(2 * PI * t / duration)
        psi = 5 * DEG_TO_RAD * math.sin(4 * PI * t / duration)

        pose = np.array([x, y, z, phi, theta, psi])
        leg_lengths_history[i] = solve_ik(pose, base_points, top_points)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    colors = plt.cm.viridis(np.linspace(0, 1, 6))
    for i in range(6):
        ax1.plot(times, leg_lengths_history[:, i],
                 color=colors[i], linewidth=1.8, label=f'Leg {i+1}')

    ax1.axhline(y=MIN_LEG_LENGTH, color='r', linestyle='--', alpha=0.5, label='Min limit')
    ax1.axhline(y=MAX_LEG_LENGTH, color='r', linestyle='--', alpha=0.5, label='Max limit')
    ax1.set_ylabel('Leg Length [mm]', fontsize=12)
    ax1.set_title('Actuator Leg Lengths During Circular + Tilt Trajectory', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', ncol=2, fontsize=9)
    ax1.set_ylim(MIN_LEG_LENGTH - 10, MAX_LEG_LENGTH + 10)

    dt = times[1] - times[0]
    velocities = np.gradient(leg_lengths_history, dt, axis=0)
    for i in range(6):
        ax2.plot(times, velocities[:, i],
                 color=colors[i], linewidth=1.5, label=f'Leg {i+1}')

    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.2)
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('Leg Velocity [mm/s]', fontsize=12)
    ax2.set_title('Actuator Velocities', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', ncol=2, fontsize=9)

    plt.tight_layout()
    png_path = OUTPUT_DIR / "leg_lengths_velocities.png"
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {png_path}")


def create_workspace_map():
    print("[3/4] Creating workspace reachability map...")

    base_points = calculate_attachment_points(BASE_RADIUS, BASE_ANGLE_OFFSET)
    top_points = calculate_attachment_points(TOP_RADIUS, TOP_ANGLE_OFFSET)

    resolution = 40
    z_levels = [120, 150, 180]
    x_range = np.linspace(-80, 80, resolution)
    y_range = np.linspace(-80, 80, resolution)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5), subplot_kw={'projection': '3d'})

    for idx, z_home in enumerate(z_levels):
        ax = axes[idx]
        reachable = np.zeros((resolution, resolution))

        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                pose = np.array([x, y, z_home, 0.0, 0.0, 0.0])
                lengths = solve_ik(pose, base_points, top_points)
                if np.all(lengths >= MIN_LEG_LENGTH) and np.all(lengths <= MAX_LEG_LENGTH):
                    reachable[j, i] = 1.0

        X, Y = np.meshgrid(x_range, y_range)
        Z = np.full_like(X, z_home)

        mask = reachable > 0.5
        ax.scatter(X[mask], Y[mask], Z[mask],
                   c='green', s=8, alpha=0.6, label='Reachable')

        mask_inv = ~mask
        ax.scatter(X[mask_inv], Y[mask_inv], Z[mask_inv],
                   c='red', s=8, alpha=0.3, label='Unreachable')

        ax.set_xlabel('X [mm]', fontsize=10)
        ax.set_ylabel('Y [mm]', fontsize=10)
        ax.set_zlabel('Z [mm]', fontsize=10)
        ax.set_title(f'Workspace at Z = {z_home} mm', fontsize=11, fontweight='bold')
        ax.set_xlim(-80, 80)
        ax.set_ylim(-80, 80)
        ax.set_zlim(100, 200)
        ax.view_init(elev=30, azim=-60)
        ax.grid(True, alpha=0.2)

    fig.suptitle('Stewart Platform Workspace Reachability (XY at Various Heights)',
                 fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    png_path = OUTPUT_DIR / "workspace_reachability.png"
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {png_path}")


def create_pose_trajectory_plot():
    print("[4/4] Creating pose trajectory plot...")

    base_points = calculate_attachment_points(BASE_RADIUS, BASE_ANGLE_OFFSET)
    top_points = calculate_attachment_points(TOP_RADIUS, TOP_ANGLE_OFFSET)

    num_steps = 300
    duration = 5.0
    times = np.linspace(0, duration, num_steps + 1)

    positions = np.zeros((num_steps + 1, 3))
    orientations = np.zeros((num_steps + 1, 3))
    leg_lengths = np.zeros((num_steps + 1, 6))

    for i, t in enumerate(times):
        s = s_curve(t, duration)
        x = 40 * math.sin(2 * PI * t / duration)
        y = 40 * math.cos(2 * PI * t / duration)
        z = HOME_Z + 40 * (1 - math.cos(2 * PI * t / duration)) / 2
        phi = 8 * DEG_TO_RAD * math.sin(2 * PI * t / duration)
        theta = 8 * DEG_TO_RAD * math.cos(2 * PI * t / duration)
        psi = 4 * DEG_TO_RAD * math.sin(4 * PI * t / duration)

        positions[i] = [x, y, z]
        orientations[i] = [phi, theta, psi]
        pose = np.array([x, y, z, phi, theta, psi])
        leg_lengths[i] = solve_ik(pose, base_points, top_points)

    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(2, 3, 1)
    ax1.plot(times, positions[:, 0], 'r-', linewidth=1.8, label='X')
    ax1.plot(times, positions[:, 1], 'g-', linewidth=1.8, label='Y')
    ax1.plot(times, positions[:, 2], 'b-', linewidth=1.8, label='Z')
    ax1.set_xlabel('Time [s]', fontsize=11)
    ax1.set_ylabel('Position [mm]', fontsize=11)
    ax1.set_title('Position vs Time', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)

    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(times, orientations[:, 0] * RAD_TO_DEG, 'r-', linewidth=1.8, label='Roll (φ)')
    ax2.plot(times, orientations[:, 1] * RAD_TO_DEG, 'g-', linewidth=1.8, label='Pitch (θ)')
    ax2.plot(times, orientations[:, 2] * RAD_TO_DEG, 'b-', linewidth=1.8, label='Yaw (ψ)')
    ax2.set_xlabel('Time [s]', fontsize=11)
    ax2.set_ylabel('Angle [°]', fontsize=11)
    ax2.set_title('Orientation vs Time', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    ax3 = fig.add_subplot(2, 3, 3)
    colors = plt.cm.viridis(np.linspace(0, 1, 6))
    for i in range(6):
        ax3.plot(times, leg_lengths[:, i], color=colors[i], linewidth=1.5, label=f'Leg {i+1}')
    ax3.axhline(y=MIN_LEG_LENGTH, color='r', linestyle='--', alpha=0.4)
    ax3.axhline(y=MAX_LEG_LENGTH, color='r', linestyle='--', alpha=0.4)
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_ylabel('Leg Length [mm]', fontsize=11)
    ax3.set_title('Leg Lengths vs Time', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, ncol=2)

    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    ax4.plot(positions[:, 0], positions[:, 1], positions[:, 2],
             'b-', linewidth=2, alpha=0.8)
    ax4.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
                c='green', s=80, marker='o', label='Start')
    ax4.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
                c='red', s=80, marker='^', label='End')
    ax4.set_xlabel('X [mm]', fontsize=10)
    ax4.set_ylabel('Y [mm]', fontsize=10)
    ax4.set_zlabel('Z [mm]', fontsize=10)
    ax4.set_title('3D Trajectory Path', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.2)

    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
    ax5.scatter(positions[0, 0], positions[0, 1], c='green', s=60, marker='o', label='Start')
    ax5.scatter(positions[-1, 0], positions[-1, 1], c='red', s=60, marker='^', label='End')
    ax5.set_xlabel('X [mm]', fontsize=11)
    ax5.set_ylabel('Y [mm]', fontsize=11)
    ax5.set_title('XY Projection', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.axis('equal')
    ax5.legend(fontsize=9)

    ax6 = fig.add_subplot(2, 3, 6)
    leg_min = np.min(leg_lengths, axis=0)
    leg_max = np.max(leg_lengths, axis=0)
    leg_avg = np.mean(leg_lengths, axis=0)
    x_pos = np.arange(6)
    ax6.bar(x_pos - 0.2, leg_max - leg_avg, 0.2, bottom=leg_avg,
            color='orange', alpha=0.7, label='Above avg')
    ax6.bar(x_pos, leg_avg - leg_min, 0.2, bottom=leg_min,
            color='steelblue', alpha=0.7, label='Below avg')
    ax6.axhline(y=MIN_LEG_LENGTH, color='r', linestyle='--', alpha=0.4)
    ax6.axhline(y=MAX_LEG_LENGTH, color='r', linestyle='--', alpha=0.4)
    ax6.set_xlabel('Leg Number', fontsize=11)
    ax6.set_ylabel('Length [mm]', fontsize=11)
    ax6.set_title('Leg Length Range', fontsize=12, fontweight='bold')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels([f'L{i+1}' for i in range(6)])
    ax6.grid(True, alpha=0.3, axis='y')
    ax6.legend(fontsize=9)

    plt.tight_layout()
    png_path = OUTPUT_DIR / "pose_trajectory_analysis.png"
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {png_path}")


def create_circular_motion_gif():
    print("[BONUS] Creating circular motion GIF...")

    base_points = calculate_attachment_points(BASE_RADIUS, BASE_ANGLE_OFFSET)
    top_points = calculate_attachment_points(TOP_RADIUS, TOP_ANGLE_OFFSET)

    num_frames = 100
    fps = 20
    duration = 3.0

    fig = plt.figure(figsize=(14, 6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1])

    ax3d = fig.add_subplot(gs[0], projection='3d')
    ax_leg = fig.add_subplot(gs[1])

    def animate(frame):
        ax3d.clear()
        ax_leg.clear()

        t = frame / num_frames * duration
        x = 50 * math.sin(2 * PI * t / duration)
        y = 50 * math.cos(2 * PI * t / duration)
        z = HOME_Z + 15 * math.sin(4 * PI * t / duration)
        phi = 5 * DEG_TO_RAD * math.sin(2 * PI * t / duration)
        theta = 5 * DEG_TO_RAD * math.cos(2 * PI * t / duration)
        psi = 0.0

        pose = np.array([x, y, z, phi, theta, psi])
        R = compute_rotation_matrix(phi, theta, psi)
        T = np.array([x, y, z])
        top_global = np.array([R @ top_points[i] + T for i in range(6)])
        lengths = solve_ik(pose, base_points, top_points)

        base_closed = np.vstack([base_points, base_points[0:1]])
        ax3d.plot(base_closed[:, 0], base_closed[:, 1], base_closed[:, 2],
                  'b-', linewidth=2)
        top_closed = np.vstack([top_global, top_global[0:1]])
        ax3d.plot(top_closed[:, 0], top_closed[:, 1], top_closed[:, 2],
                  'r-', linewidth=2)

        for i in range(6):
            ax3d.plot([base_points[i, 0], top_global[i, 0]],
                      [base_points[i, 1], top_global[i, 1]],
                      [base_points[i, 2], top_global[i, 2]],
                      color='green', linewidth=1.5, alpha=0.7)

        ax3d.scatter(base_points[:, 0], base_points[:, 1], base_points[:, 2],
                     c='blue', s=30)
        ax3d.scatter(top_global[:, 0], top_global[:, 1], top_global[:, 2],
                     c='red', s=30)

        ax3d.set_xlim(-200, 200)
        ax3d.set_ylim(-200, 200)
        ax3d.set_zlim(0, 300)
        ax3d.set_xlabel('X [mm]')
        ax3d.set_ylabel('Y [mm]')
        ax3d.set_zlabel('Z [mm]')
        ax3d.set_title(f'3D View (t={t:.2f}s)', fontsize=11, fontweight='bold')
        ax3d.view_init(elev=25, azim=30 + frame * 0.8)
        ax3d.grid(True, alpha=0.2)

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        bars = ax_leg.bar(range(6), lengths, color=colors, alpha=0.8, edgecolor='black', linewidth=0.5)
        ax_leg.axhline(y=MIN_LEG_LENGTH, color='r', linestyle='--', alpha=0.5, label='Min')
        ax_leg.axhline(y=MAX_LEG_LENGTH, color='r', linestyle='--', alpha=0.5, label='Max')
        ax_leg.set_ylim(MIN_LEG_LENGTH - 10, MAX_LEG_LENGTH + 10)
        ax_leg.set_xlabel('Leg Number', fontsize=11)
        ax_leg.set_ylabel('Length [mm]', fontsize=11)
        ax_leg.set_title('Actuator Lengths', fontsize=11, fontweight='bold')
        ax_leg.set_xticks(range(6))
        ax_leg.set_xticklabels([f'L{i+1}' for i in range(6)])
        ax_leg.grid(True, alpha=0.3, axis='y')
        ax_leg.legend(fontsize=9)

        for bar, length in zip(bars, lengths):
            ax_leg.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                        f'{length:.1f}', ha='center', va='bottom', fontsize=8)

        plt.tight_layout()

    anim = FuncAnimation(fig, animate, frames=num_frames, interval=1000/fps)
    gif_path = OUTPUT_DIR / "circular_motion_leg_lengths.gif"
    anim.save(gif_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"  Saved: {gif_path}")


def main():
    print("=" * 60)
    print("Stewart Platform Simulation - Generating Visualizations")
    print("=" * 60)
    print()

    create_3d_animation_gif()
    print()
    create_leg_lengths_plot()
    print()
    create_workspace_map()
    print()
    create_pose_trajectory_plot()
    print()
    create_circular_motion_gif()
    print()
    print("=" * 60)
    print(f"All visualizations saved to: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
