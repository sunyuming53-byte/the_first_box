#!/usr/bin/env python3
"""
Interactive 3D visualization of a two-axis gimbal segment on a rotating rectangle.

The rectangle rotates by angle θ around the z-axis (its left edge).  A line
segment on the rectangle undergoes a two-axis gimbal rotation: φ about axis
u₁ = (cosθ, sinθ, 0) through its top endpoint, then ω about the perpendicular
axis u₂ = (-sinθ, cosθ, 0) through the same top endpoint.
ω ∈ (0, π/2) — second gimbal restricted to the first quadrant.

Controls:
    θ  slider — rectangle rotation angle        [0°, 360°]
    φ  slider — first  gimbal rotation (about u₁) [0°, 360°]
    ω  slider — second gimbal rotation (about u₂) [0°, 90°]
    r  slider — distance from segment to z-axis
    L  slider — half-length of the segment
    h  slider — height of segment center point
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ---- Rotation helpers ------------------------------------------------

def rotate_vector(v, axis, angle):
    """Rotate vector v by angle (rad) around unit axis using Rodrigues."""
    c = np.cos(angle)
    s = np.sin(angle)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def rotation_matrix(axis, angle):
    """3×3 rotation matrix via Rodrigues (axis must be unit vector)."""
    c = np.cos(angle)
    s = np.sin(angle)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


# ---- Kinematic model -------------------------------------------------

def center_point(theta, phi, omega, r, L, h):
    """Segment center after rectangle rotation + two-axis gimbal rotation.

    C(θ,φ,ω) = T - L · R₂(ω) · R₁(φ) · e_z
    where T = (r cosθ, r sinθ, h+L).
    """
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)
    cw, sw = np.cos(omega), np.sin(omega)

    x = r * ct - L * st * sp - L * ct * cp * sw
    y = r * st + L * ct * sp - L * st * cp * sw
    z = h + L * (1 - cp * cw)
    return np.array([x, y, z])


def top_endpoint(theta, r, L, h):
    """Top endpoint — both gimbal axes pass through here."""
    return np.array([r * np.cos(theta), r * np.sin(theta), h + L])


def gimbal_axes(theta, r, L, h):
    """World-frame unit vectors for the two gimbal axes."""
    u1 = np.array([np.cos(theta), np.sin(theta), 0])
    u2 = np.array([-np.sin(theta), np.cos(theta), 0])
    T = top_endpoint(theta, r, L, h)
    return T, u1, u2


def segment_endpoints(theta, phi, omega, r, L, h):
    """Both endpoints of the segment after all rotations."""
    C = center_point(theta, phi, omega, r, L, h)
    T = top_endpoint(theta, r, L, h)
    B = 2 * C - T        # bottom = center - (top - center) = 2·center - top
    return B, T


def target_axes(theta, phi, omega):
    """Target frame axes in world coordinates.

    x̂_T: along segment (bottom→top) = R₂(ω)·R₁(φ)·e_z
    ŷ_T: second gimbal axis after the φ rotation = R₂(ω)·u₁
    ẑ_T: x̂_T × ŷ_T
    Returns (x_hat, y_hat, z_hat) as unit vectors.
    """
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)
    cw, sw = np.cos(omega), np.sin(omega)

    x_hat = np.array([st * sp + ct * cp * sw,
                      -ct * sp + st * cp * sw,
                      cp * cw])

    u1 = np.array([ct, st, 0])
    y_hat = rotate_vector(u1, np.array([-st, ct, 0]), omega)

    z_hat = np.cross(x_hat, y_hat)
    return x_hat, y_hat, z_hat


# ---- Geometry --------------------------------------------------------

def rectangle_vertices(theta, r, h, L, rect_width_factor=1.8):
    """Vertices of the rectangle with its left edge pinned to the z-axis."""
    w = r * rect_width_factor
    corners = np.array([
        [0, 0, h - L],
        [w, 0, h - L],
        [w, 0, h + L],
        [0, 0, h + L],
    ])
    ct, st = np.cos(theta), np.sin(theta)
    for i in range(4):
        x0, y0 = corners[i, 0], corners[i, 1]
        corners[i, 0] = x0 * ct - y0 * st
        corners[i, 1] = x0 * st + y0 * ct
    return corners


# ---- Main ------------------------------------------------------------

def main():
    plt.rcParams['font.size'] = 11

    r_init, L_init, h_init = 2.0, 1.5, 0.0
    theta_init, phi_init, omega_init = 34.0, 24.0, 30.0

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title("Two-Axis Gimbal Segment Visualization")

    # 3D axes takes left 65% of figure
    ax = fig.add_axes([0.05, 0.20, 0.65, 0.75], projection='3d')

    # Slider positions (right column)
    ax_theta = fig.add_axes([0.75, 0.84, 0.20, 0.03])
    ax_phi   = fig.add_axes([0.75, 0.78, 0.20, 0.03])
    ax_omega = fig.add_axes([0.75, 0.72, 0.20, 0.03])
    ax_r     = fig.add_axes([0.75, 0.64, 0.20, 0.03])
    ax_L     = fig.add_axes([0.75, 0.56, 0.20, 0.03])
    ax_h     = fig.add_axes([0.75, 0.48, 0.20, 0.03])

    s_theta = Slider(ax_theta, r'$\theta$ (°)',   0, 360, valinit=theta_init)
    s_phi   = Slider(ax_phi,   r'$\phi$ (°)',     0, 360, valinit=phi_init)
    s_omega = Slider(ax_omega, r'$\omega$ (°)',    0,  90, valinit=omega_init)
    s_r     = Slider(ax_r,     r'$r$',          0.5, 5.0, valinit=r_init)
    s_L     = Slider(ax_L,     r'$L$',          0.5, 4.0, valinit=L_init)
    s_h     = Slider(ax_h,     r'$h$',         -3.0, 3.0, valinit=h_init)

    view_radius = 6.0

    def update(val=None):
        r_val = s_r.val
        L_val = s_L.val
        h_val = s_h.val
        theta = np.deg2rad(s_theta.val)
        phi   = np.deg2rad(s_phi.val)
        omega = np.deg2rad(s_omega.val)

        ax.cla()
        ax.set_xlim(-view_radius, view_radius)
        ax.set_ylim(-view_radius, view_radius)
        ax.set_zlim(h_val - 2 * L_val - 1, h_val + 2 * L_val + 1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # ---- World axes ----
        axis_len = view_radius * 0.6
        ax.quiver(0, 0, 0, axis_len, 0, 0, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, axis_len, 0, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.quiver(0, 0, 0, 0, 0, axis_len, color='#333333', arrow_length_ratio=0.1, linewidth=1.2)
        ax.text(axis_len + 0.2, 0, 0, 'X', color='#333333', fontsize=10)
        ax.text(0, axis_len + 0.2, 0, 'Y', color='#333333', fontsize=10)
        ax.text(0, 0, axis_len + 0.2, 'Z', color='#333333', fontsize=10)

        # ---- Z-axis (rectangle rotation edge) ----
        edge_z_min = h_val - L_val - 1.0
        edge_z_max = h_val + L_val + 1.0
        ax.plot([0, 0], [0, 0], [edge_z_min, edge_z_max],
                '#666666', linewidth=1.2, linestyle='--', label='Z-axis (edge)')

        # ---- Rectangle ----
        rect_corners = rectangle_vertices(theta, r_val, h_val, L_val, rect_width_factor=2.2)
        verts = [rect_corners]
        rect = Poly3DCollection(verts, alpha=0.18, facecolor='steelblue',
                                edgecolor='#4a90d9', linewidth=0.8)
        ax.add_collection3d(rect)

        # ---- Gimbal axes ----
        T_, u1, u2 = gimbal_axes(theta, r_val, L_val, h_val)
        ax_len = 3.0

        # u₁ axis (first gimbal, orange)
        p1a = T_ - ax_len * u1
        p1b = T_ + ax_len * u1
        ax.plot([p1a[0], p1b[0]], [p1a[1], p1b[1]], [p1a[2], p1b[2]],
                '#e07000', linewidth=1.8, label=r'Gimbal axis $\mathbf{u}_1$')

        # u₂ axis (second gimbal, purple)
        p2a = T_ - ax_len * u2
        p2b = T_ + ax_len * u2
        ax.plot([p2a[0], p2b[0]], [p2a[1], p2b[1]], [p2a[2], p2b[2]],
                '#8855aa', linewidth=1.8, linestyle='--',
                label=r'Gimbal axis $\mathbf{u}_2$')

        # ---- Segment ----
        B, T_pt = segment_endpoints(theta, phi, omega, r_val, L_val, h_val)
        C = center_point(theta, phi, omega, r_val, L_val, h_val)
        ax.plot([B[0], T_pt[0]], [B[1], T_pt[1]], [B[2], T_pt[2]],
                '#cc2244', linewidth=2.5, label='Segment')

        # ---- Key points ----
        ax.scatter(*C, color='#cc2244', s=80, edgecolors='#881122',
                   linewidth=1.2, zorder=5)
        ax.scatter(*T_pt, color='#e07000', s=50, edgecolors='#663300',
                   linewidth=1.0, zorder=5, label='Top endpoint')

        # ---- Target frame axes at center ----
        x_hat, y_hat, z_hat = target_axes(theta, phi, omega)
        scale = L_val * 0.65
        ax.quiver(*C, *(scale * x_hat), color='#cc2244', linewidth=1.2,
                  arrow_length_ratio=0.15)
        ax.quiver(*C, *(scale * y_hat), color='#2ca02c', linewidth=1.2,
                  arrow_length_ratio=0.15)
        ax.quiver(*C, *(scale * z_hat), color='#1f77b4', linewidth=1.2,
                  arrow_length_ratio=0.15)
        ax.text(*(C + scale * 1.15 * x_hat),
                r'$\hat{\mathbf{x}}_T$', color='#cc2244', fontsize=10)
        ax.text(*(C + scale * 1.15 * y_hat),
                r'$\hat{\mathbf{y}}_T$', color='#2ca02c', fontsize=10)
        ax.text(*(C + scale * 1.15 * z_hat),
                r'$\hat{\mathbf{z}}_T$', color='#1f77b4', fontsize=10)

        # ---- Trajectory: φ sweep at current (θ, ω) ----
        phi_vals = np.linspace(0, 2 * np.pi, 200)
        pts = np.array([center_point(theta, p, omega, r_val, L_val, h_val)
                        for p in phi_vals])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                '#e07000', linewidth=0.6, alpha=0.3, linestyle=':',
                label='Center trajectory (φ sweep)')

        # ---- Trajectory: ω sweep at current (θ, φ) ----
        omega_vals = np.linspace(0, np.pi / 2, 100)
        pts2 = np.array([center_point(theta, phi, w, r_val, L_val, h_val)
                         for w in omega_vals])
        ax.plot(pts2[:, 0], pts2[:, 1], pts2[:, 2],
                '#8855aa', linewidth=0.6, alpha=0.3, linestyle=':',
                label='Center trajectory (ω sweep)')

        # ---- Trajectory: θ sweep at current (φ, ω) ----
        theta_vals = np.linspace(0, 2 * np.pi, 200)
        pts3 = np.array([center_point(t, phi, omega, r_val, L_val, h_val)
                         for t in theta_vals])
        ax.plot(pts3[:, 0], pts3[:, 1], pts3[:, 2],
                '#4a90d9', linewidth=0.6, alpha=0.3, linestyle=':',
                label='Center trajectory (θ sweep)')

        # ---- Info text ----
        info = (
            rf'$C(\theta,\phi,\omega)$ = ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f})'
            + '\n'
            rf'$\theta$ = {s_theta.val:.1f}°  |  $\phi$ = {s_phi.val:.1f}°  '
            rf'|  $\omega$ = {s_omega.val:.1f}°'
        )
        if hasattr(update, 'info_text') and update.info_text is not None:
            update.info_text.set_text(info)
        else:
            update.info_text = fig.text(0.75, 0.38, info,
                                        fontsize=10, fontfamily='monospace',
                                        verticalalignment='top',
                                        bbox=dict(boxstyle='round,pad=0.4',
                                                  facecolor='white', alpha=0.9))

        ax.legend(loc='upper left', fontsize=7)

        fig.canvas.draw_idle()

    update.info_text = None
    s_theta.on_changed(update)
    s_phi.on_changed(update)
    s_omega.on_changed(update)
    s_r.on_changed(update)
    s_L.on_changed(update)
    s_h.on_changed(update)

    update()
    ax.view_init(elev=np.rad2deg(np.arcsin(1 / np.sqrt(3))), azim=45)
    plt.show()


if __name__ == '__main__':
    main()
