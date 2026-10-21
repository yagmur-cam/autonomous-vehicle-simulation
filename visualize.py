#Scientific plotting, error fiagnostics, 2-sigma covariance ellipses, and animation.

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse #to draw the 2D confidence regions (uncertainty ellipses) directly onto the trejectory plot. Orientation angle is derived from the Kalman covariance matrix.
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation, PillowWriter
#FuncAnimation: drives the frame-by-frame loop for dynamic vehicle simulation, updating vehicle position, the trailing trajectory line, radar pings, the pulsing covariance ellipses at each step.
#PillowWriter: file-writing engine encoding + compressing the rendered animation frames into a .gif file.

def compute_covariance_ellipse(P_pos: np.ndarray, n_sigma: float = 2.0) -> tuple[float, float, float]:
    #Decomposes the 2x2 position block of covariance matrix P into ellipse parameters.
    #np.linalg.eigh - provides eigenvalues + eigenvectors.
    #Semi-axis lengths are: 2 * sqrt(eigenvalue) for a 2-sigma confidence region.
    #orientation angle theta is derived from the primary eigenvector.

    #Returns:
        # (width, height, angle_degrees)
    
    #Guard against slight numerical negative zero drift

    P_pos = np.asarray(P_pos, dtype = float)
    eigenvalues, eigenvectors = np.linalg.eigh(P_pos)
    eigenvalues = np.maximum(eigenvalues, 1e-12)

    #Sort eigenvalues and eigenvectors in DESCENDING order:
    
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    #Rotation angle in degrees for Matplotlib Ellipse patch
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

    #Matplotlib Ellipse expects full width and full height (diameter = 2 * semi-axis)
    width = 2.0 * n_sigma * np.sqrt(eigenvalues[0])
    height = 2.0 * n_sigma * np.sqrt(eigenvalues[1])

    return float(width), float(height), float(angle)

def plot_static_study(
        truth: np.ndarray,
        measurements: np.ndarray,
        estimates: np.ndarray,
        covariances: np.ndarray,
        dt: float,
        dropouts: list[tuple[int, int]]= None,
        filename: str = "figures/kalman_study.png"
):
        # Producing the 3-panel static study:
        # 1. Trajectory - truth, noisy dots, filter line, covaraince ellipse
        # 2. Error against time - raw error vs filter error with shaded dropouts
        # 3. Uncertainty evolution over time: Trace(P_pos).

        n_steps = len(truth)
        time = np.arange(n_steps) * dt
        valid = ~np.isnan(measurements[:, 0])

        #Euclidean position error norms
        #Raw Error and KF Error represent the Euclidean distance between what's actually happening in the simulation (i.e. ground truth) and what's being reported by either the bare sensor or the filter.
        raw_err = np.linalg.norm(measurements[valid] - truth[valid, :2], axis=1)
        kf_err = np.linalg.norm(estimates[:, :2]- truth[:, :2], axis=1)

        raw_rmse = np.sqrt(np.mean(raw_err**2))
        kf_rmse = np.sqrt(np.mean(kf_err**2))

        fig = plt.figure(figsize=(15, 8.5), constrained_layout=True)
        gs = GridSpec(2, 2, figure=fig, width_ratios=[1.2, 1.0])

        ax_traj = fig.add_subplot(gs[:, 0])
        ax_err = fig.add_subplot(gs[0, 1])
        ax_unc = fig.add_subplot(gs[1, 1])

        # -------- Part 1 - Trajectory Panel (large 2D map on the left side of the static figure)--------

        ax_traj.set_title("2D Vehicle Trajectory & Sensor Coverage", fontweight= "bold", fontsize=11)
        ax_traj.plot(truth[:, 0], truth[:, 1], "k-", lw=1.8, label="Ground Truth")
        ax_traj.scatter(measurements[valid, 0], measurements[valid, 1],
                        color = "crimson", alpha=0.35, s=12, label="Radar Measurements")
        ax_traj.plot(estimates[:, 0], estimates[:, 1], "royalblue", lw=2.0, label="Kalman Estimate")
        # "crimson" = Radar Hits (red scatter dots) - raw position pings recorder by the onboard radar sensor (Z_x, Z_y)
        # "royalblue" = Kalman Estimate Line - represents the state vector's filtered position coordinates.

        # Overlay 2-sigma uncertainty ellipses every 20 frames:
        for k in range (0, n_steps, 20):
              w, h, ang = compute_covariance_ellipse(covariances[k, :2, :2], n_sigma=2.0)
              ell = Ellipse(xy=estimates[k, :2], width=w, height=h, angle=ang,
                            edgecolor="royalblue", fc="none", lw=1.2, alpha=0.7)
              ax_traj.add_patch(ell)

        ax_traj.set_xlabel("X Position (m)")
        ax_traj.set_ylabel("Y Position (m)")
        ax_traj.legend(loc="upper left")
        ax_traj.grid(True, linestyle= ":", alpha=0.6)
        ax_traj.axis("equal")

        # ------- Part 2 -  Error vs Time panel ---------
        ax_err.set_title(f"Position Error (Raw RMSE: {raw_rmse: .2f}m | KF RMSE: {kf_rmse: .2f}m)",
                         fontweight="bold", fontsize=11)
        ax_err.plot(time[valid], raw_err, "r.", alpha =0.35, label="Raw Sensor Error")
        ax_err.plot(time, kf_err, "royalblue", lw=1.8, label="Kalman Filter Error")

        if dropouts:
              for s, e in dropouts:
                    ax_err.axvspan(s * dt, e * dt, color="orange", alpha=0.2, label="Sensor Dropout")
        
        ax_err.set_xlabel("Time (s)")
        ax_err.set_ylabel("Error norm (m)")
        ax_err.grid(True, linestyle= ":", alpha=0.6)
        ax_err.legend(loc="upper right")

        # -------- Part 3 - uncertainty Trace Panel - evaluating quantitative accuracy of both raw sensor + kalman filter across the entire timeline of the run --------
        # Trace(P_pos) represents total positional variance: Var(x) + Var(y)
        trace_P_pos = np.trace(covariances[:, :2, :2], axis1=1, axis2=2)
        ax_unc.set_title("Filter Uncertainty Evolution: Trace(P_pos)", fontweight="bold", fontsize=11)
        ax_unc.plot(time, trace_P_pos, color="darkviolet", lw=1.8, label="Trace(P_pos)")
    
        if dropouts:
         for s, e in dropouts:
            ax_unc.axvspan(s * dt, e * dt, color="orange", alpha=0.2)
            
        ax_unc.set_xlabel("Time (s)")
        ax_unc.set_ylabel("Trace(P) (m²)")
        ax_unc.grid(True, linestyle=":", alpha=0.6)
        ax_unc.legend(loc="upper right")
    
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else ".", exist_ok=True)
        plt.savefig(filename, dpi=300)
        plt.close()
        print(f"[OK] Static study figure saved to: {filename}")

def create_animation(
          truth: np.ndarray,
          measurements: np.ndarray,
          estimates: np.ndarray,
          covariances: np.ndarray,
          filename: str = "figures/tracking.gif",
          fps: int = 15
):
     #render the moving-camera tracking reel with the breathing 2-sigma covariance ellipse as a GIF for the README file.
     fig, ax = plt.subplots(figsize=(8, 6))
     ax.set_aspect("equal")
     ax.set_facecolor("#12121a")
     fig.patch.set_facecolor("#12121a")

     ax.set_title("Kalman Tracking with 2σ Uncertainty", color="white", fontweight="bold")
     ax.set_xlabel("X Position (m)", color="white")
     ax.set_ylabel("Y Position (m)", color="white")

     line_true, = ax.plot([], [], color="#dfe6e9", lw=2.0, alpha=0.85, label="Ground Truth")
     line_est, = ax.plot([], [], color="#00d2d3", lw=2.0, label="Kalman Estimate")
     scat_meas = ax.scatter([], [], color="#ff6b6b", s=20, alpha=0.6, label="Radar Hits")
    
     car_dot, = ax.plot([], [], "wo", markersize=8)
     est_dot, = ax.plot([], [], "o", color="#00d2d3", markersize=6)
    
     ellipse_patch = Ellipse(xy=(0, 0), width=0, height=0, angle=0,
                            edgecolor="#00d2d3", fc="none", lw=1.5, linestyle="--")
     ax.add_patch(ellipse_patch)

     ax.legend(loc="upper left", facecolor="#1e272e", edgecolor="none", labelcolor="white")
     ax.grid(True, color="#2d3436", linestyle=":")
     ax.tick_params(colors="white")

     def init():
          line_true.set_data([], [])
          line_est.set_data([], [])
          scat_meas.set_offsets(np.empty((0,2)))
          car_dot.set_data([], [])
          est_dot.set_data([], [])
          return line_true, line_est, scat_meas, car_dot, est_dot, ellipse_patch
     
     def update (frame):
          # Update trailing path lines:
          line_true.set_data(truth[:frame, 0], truth[:frame, 1])
          line_est.set_data(estimates[:frame, 0], estimates[:frame, 1])

          # Vehicle + estimate position heads:
          car_dot.set_data([truth[frame, 0]], [truth[frame, 1]])
          est_dot.set_data([estimates[frame, 0]], [estimates[frame, 1]])

          # Drawing recent 12 radar hits:
          start_ping = max(0, frame - 12)
          recent_meas = measurements[start_ping:frame]
          valid_meas = recent_meas[~np.isnan(recent_meas[:, 0])]
          scat_meas.set_offsets(valid_meas if len(valid_meas) > 0 else np.empty((0, 2)))

          # Updating dynamic 2_sigma uncertainty ellipse:
          w, h, ang = compute_covariance_ellipse(covariances[frame, :2, :2], n_sigma=2.0)
          ellipse_patch.set_center(estimates[frame, :2])
          ellipse_patch.width = w
          ellipse_patch.height = h
          ellipse_patch.angle = ang

          # Moving camera centered on the vehicle:
          cx, cy = truth[frame, :2]
          ax.set_xlim(cx - 30, cx + 30)
          ax.set_ylim(cy - 22.5, cy + 22.5)
          return line_true, line_est, scat_meas, car_dot, est_dot, ellipse_patch
     
     anim = FuncAnimation(fig, update, frames=len(truth), init_func=init, blit=False)

     os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else ".", exist_ok = True)
     writer = PillowWriter(fps=fps)
     anim.save(filename, writer=writer)
     plt.close()
     print(f"[OK] Animation GIF saved to: {filename}")


    

        
        





    