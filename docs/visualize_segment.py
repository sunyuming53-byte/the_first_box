#!/usr/bin/env python3
"""
Interactive 3D visualization of a line segment on a rotating rectangle.

The rectangle rotates by angle θ around the z-axis (its left edge). A line
segment on the rectangle rotates by angle φ around an axis through its top
endpoint with direction (cosθ, sinθ, 0). The track of the segment's center
point is shown.

Controls:
    θ slider — rectangle rotation angle
    φ slider — segment rotation angle
    r slider — distance from segment to z-axis
    L slider — half-length of the segment
    h slider — height of segment center point
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def center_point(theta, phi, r, L, h):
    """Segment center after both rotations.

    At θ=0 the segment center is at (r, 0, h), distance r from the z-axis.
    Rectangle rotates CCW about z-axis, then segment rotates by φ about its
    top-endpoint axis u = (cosθ, sinθ, 0).
    """
    x = r * np.cos(theta) - L * np.sin(theta) * np.sin(phi)
    y = r * np.sin(theta) + L * np.cos(theta) * np.sin(phi)
    z = h + L * (1 - np.cos(phi))
    return np.array([x, y, z])


def top_endpoint(theta, r, L, h):
    """Top endpoint — the segment's own rotation axis passes through here."""
    return np.array([r * np.cos(theta), r * np.sin(theta), h + L])


def rotation_axis_endpoints(theta, r, L, h, axis_length=2.0):
    """Endpoints of the segment's rotation axis through its top endpoint."""
    T = top_endpoint(theta, r, L, h)
    u = np.array([np.cos(theta), np.sin(theta), 0])
    return T - axis_length * u, T + axis_length * u


def segment_endpoints(theta, phi, r, L, h):
    """Both endpoints of the segment after rotation."""
    C = center_point(theta, phi, r, L, h)
    T = top_endpoint(theta, r, L, h)
    B = 2 * C - T
    return B, T


def target_axes(theta, phi):
    """Target frame axes in world coordinates.

    x̂: along segment (bottom→top), ŷ: parallel to u, ẑ: completes right-hand frame.
    Returns (x_hat, y_hat, z_hat) as unit vectors.
    """
    x_hat = np.array([ np.sin(theta) * np.sin(phi),
                      -np.cos(theta) * np.sin(phi),
                       np.cos(phi)])
    y_hat = np.array([np.cos(theta), np.sin(theta), 0])
    z_hat = np.array([-np.cos(phi) * np.sin(theta),
                       np.cos(phi) * np.cos(theta),
                       np.sin(phi)])
    return x_hat, y_hat, z_hat


def rectangle_vertices(theta, r, h, L, rect_width_factor=1.8):
    """Vertices of the rectangle with its left edge pinned to the z-axis."""
    w = r * rect_width_factor
    corners = np.array([
        [0, 0, h - L],    # bottom at axis
        [w, 0, h - L],    # bottom far
        [w, 0, h + L],    # top far
        [0, 0, h + L],    # top at axis
    ])
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    for i in range(4):
        x0, y0 = corners[i, 0], corners[i, 1]
        corners[i, 0] = x0 * cos_t - y0 * sin_t
        corners[i, 1] = x0 * sin_t + y0 * cos_t
    return corners


def main():
    plt.rcParams['font.size'] = 11

    r_init, L_init, h_init = 2.0, 1.5, 0.0
    theta_init, phi_init = 34.0, 24.0
    r_val, L_val, h_val = r_init, L_init, h_init

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title("Line Segment Rotation Visualization")

    ax = fig.add_axes([0.05, 0.20, 0.65, 0.75], projection='3d')

    ax_theta = fig.add_axes([0.75, 0.80, 0.20, 0.03])
    ax_phi   = fig.add_axes([0.75, 0.72, 0.20, 0.03])
    ax_r     = fig.add_axes([0.75, 0.64, 0.20, 0.03])
    ax_L     = fig.add_axes([0.75, 0.56, 0.20, 0.03])
    ax_h     = fig.add_axes([0.75, 0.48, 0.20, 0.03])

    s_theta = Slider(ax_theta, r'$\theta$ (°)', 0, 360, valinit=theta_init)
    s_phi   = Slider(ax_phi,   r'$\phi$ (°)',   0, 360, valinit=phi_init)
    s_r     = Slider(ax_r,     r'$r$',       0.5, 5.0,    valinit=r_init)
    s_L     = Slider(ax_L,     r'$L$',       0.5, 4.0,    valinit=L_init)
    s_h     = Slider(ax_h,     r'$h$',      -3.0, 3.0,    valinit=h_init)

    view_radius = 6.0

    def update(val=None):
        nonlocal r_val, L_val, h_val
        r_val = s_r.val
        L_val = s_L.val
        h_val = s_h.val
        theta = np.deg2rad(s_theta.val)
        phi   = np.deg2rad(s_phi.val)

        ax.cla()
        ax.set_xlim(-view_radius, view_radius)
        ax.set_ylim(-view_radius, view_radius)
        ax.set_zlim(h_val - 2 * L_val - 1, h_val + 2 * L_val + 1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        axis_len = view_radius * 0.6
        ax.quiver(0, 0, 0, axis_len, 0, 0, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, axis_len, 0, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, 0, axis_len, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.text(axis_len + 0.2, 0, 0, 'X', color='#333333', fontsize=10)
        ax.text(0, axis_len + 0.2, 0, 'Y', color='#333333', fontsize=10)
        ax.text(0, 0, axis_len + 0.2, 'Z', color='#333333', fontsize=10)

        edge_z_min = h_val - L_val - 1.0
        edge_z_max = h_val + L_val + 1.0
        ax.plot([0, 0], [0, 0], [edge_z_min, edge_z_max],
                '#666666', linewidth=1.2, linestyle='--', label='Z-axis (edge)')

        rect_corners = rectangle_vertices(theta, r_val, h_val, L_val, rect_width_factor=2.2)
        verts = [rect_corners]
        rect = Poly3DCollection(verts, alpha=0.18, facecolor='steelblue',
                                edgecolor='#4a90d9', linewidth=0.8)
        ax.add_collection3d(rect)

        a1, a2 = rotation_axis_endpoints(theta, r_val, L_val, h_val, axis_length=3.0)
        ax.plot([a1[0], a2[0]], [a1[1], a2[1]], [a1[2], a2[2]],
                '#e07000', linewidth=1.5, label='Seg. rot. axis')

        B, T = segment_endpoints(theta, phi, r_val, L_val, h_val)
        C = center_point(theta, phi, r_val, L_val, h_val)
        ax.plot([B[0], T[0]], [B[1], T[1]], [B[2], T[2]],
                '#cc2244', linewidth=2.0, label='Segment')

        ax.scatter(*C, color='#cc2244', s=80, edgecolors='#881122', linewidth=1.2, zorder=5)
        ax.scatter(*T, color='#e07000', s=50, edgecolors='#663300', linewidth=1.0, zorder=5)

        x_hat, y_hat, z_hat = target_axes(theta, phi)
        scale = L_val * 0.65
        ax.quiver(*C, *(scale * x_hat), color='#cc2244',   linewidth=1.2, arrow_length_ratio=0.15)
        ax.quiver(*C, *(scale * y_hat), color='#2ca02c',   linewidth=1.2, arrow_length_ratio=0.15)
        ax.quiver(*C, *(scale * z_hat), color='#1f77b4',   linewidth=1.2, arrow_length_ratio=0.15)
        ax.text(*(C + scale * 1.15 * x_hat), r'$\hat{\mathbf{x}}_T$', color='#cc2244',   fontsize=10)
        ax.text(*(C + scale * 1.15 * y_hat), r'$\hat{\mathbf{y}}_T$', color='#2ca02c',   fontsize=10)
        ax.text(*(C + scale * 1.15 * z_hat), r'$\hat{\mathbf{z}}_T$', color='#1f77b4',   fontsize=10)

        phi_vals = np.linspace(0, 2 * np.pi, 200)
        trail = np.array([
            center_point(theta, p, r_val, L_val, h_val) for p in phi_vals
        ])
        ax.plot(trail[:, 0], trail[:, 1], trail[:, 2],
                '#8855aa', linewidth=0.6, alpha=0.3, linestyle=':',
                label='Center trajectory (φ sweep)')

        theta_vals = np.linspace(0, 2 * np.pi, 200)
        trail2 = np.array([
            center_point(t, phi, r_val, L_val, h_val) for t in theta_vals
        ])
        ax.plot(trail2[:, 0], trail2[:, 1], trail2[:, 2],
                '#8855aa', linewidth=0.6, alpha=0.3, linestyle=':',
                label='Center trajectory (θ sweep)')

        info = (
            rf'$C(\theta,\phi)$ = ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})' + '\n'
            f'θ = {s_theta.val:.1f}°  |  φ = {s_phi.val:.1f}°'
        )
        if hasattr(update, 'info_text') and update.info_text is not None:
            update.info_text.set_text(info)
        else:
            update.info_text = fig.text(0.75, 0.38, info,
                                        fontsize=10, fontfamily='monospace',
                                        verticalalignment='top',
                                        bbox=dict(boxstyle='round,pad=0.4',
                                                  facecolor='white', alpha=0.9))
        ax.legend(loc='upper left', fontsize=8)

        fig.canvas.draw_idle()

    update.info_text = None
    s_theta.on_changed(update)
    s_phi.on_changed(update)
    s_r.on_changed(update)
    s_L.on_changed(update)
    s_h.on_changed(update)

    update()
    ax.view_init(elev=np.rad2deg(np.arcsin(1/np.sqrt(3))), azim=45)
    plt.show()


if __name__ == '__main__':
    main()
