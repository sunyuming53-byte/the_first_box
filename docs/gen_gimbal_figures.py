#!/usr/bin/env python3
"""
Generate figures for gimbal kinematics sequential motion analysis.

Motion model (θ ∈ (0, 2π/3)):
  Stage 1: θ=0, φ: 0 → π/4, ω=0
  Stage 2: θ=0, φ=π/4, ω: 0 → π/2
  Stage 3: θ: 0 → 2π/3, φ=π/4, ω=π/2

r=2, L=1.5, h=0 (matching line-segment-rotation.tex parameters).
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUTDIR = os.path.dirname(os.path.abspath(__file__))

R, L, H = 2.0, 1.5, 0.0

# ---- Kinematic functions --------------------------------------------

def rotate_vector(v, axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    return v*c + np.cross(axis, v)*s + axis*np.dot(axis, v)*(1-c)

def center(theta, phi, omega, r=None, Lv=None, h=None):
    """Segment center.  Uses module-level R,L,H unless overrides given."""
    rv = R if r is None else r
    Ln = L if Lv is None else Lv
    hn = H if h is None else h
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)
    cw, sw = np.cos(omega), np.sin(omega)
    return np.array([rv*ct - Ln*st*sp - Ln*ct*cp*sw,
                     rv*st + Ln*ct*sp - Ln*st*cp*sw,
                     hn + Ln*(1 - cp*cw)])

def top_ep(theta):
    return np.array([R*np.cos(theta), R*np.sin(theta), H+L])

def target_axes(theta, phi, omega):
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)
    cw, sw = np.cos(omega), np.sin(omega)
    xh = np.array([st*sp + ct*cp*sw, -ct*sp + st*cp*sw, cp*cw])
    u1 = np.array([ct, st, 0])
    u2 = np.array([-st, ct, 0])
    yh = rotate_vector(u1, u2, omega)
    zh = np.cross(xh, yh)
    return xh, yh, zh

def gimbal_axes(theta):
    u1 = np.array([np.cos(theta), np.sin(theta), 0])
    u2 = np.array([-np.sin(theta), np.cos(theta), 0])
    return u1, u2

def segment_ends(theta, phi, omega):
    C = center(theta, phi, omega)
    T = top_ep(theta)
    B = 2*C - T
    return B, T

def rect_verts(theta):
    w = R * 2.2
    corners = np.array([[0,0,H-L], [w,0,H-L], [w,0,H+L], [0,0,H+L]])
    ct, st = np.cos(theta), np.sin(theta)
    for i in range(4):
        x0, y0 = corners[i,0], corners[i,1]
        corners[i,0] = x0*ct - y0*st
        corners[i,1] = x0*st + y0*ct
    return corners

# ---- Plotting helpers -----------------------------------------------

def setup_3d(ax, title, view_radius=5.0):
    ax.set_xlim(-view_radius, view_radius)
    ax.set_ylim(-view_radius, view_radius)
    ax.set_zlim(H - 2*L - 1, H + 2*L + 1)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(title, fontsize=11, fontweight='bold')
    # world axes
    al = view_radius*0.5
    ax.quiver(0,0,0, al,0,0, color='#333', arrow_length_ratio=0.1, lw=0.8)
    ax.quiver(0,0,0, 0,al,0, color='#333', arrow_length_ratio=0.1, lw=0.8)
    ax.quiver(0,0,0, 0,0,al, color='#333', arrow_length_ratio=0.1, lw=0.8)
    # z-axis edge
    ax.plot([0,0],[0,0],[H-L-1, H+L+1], '#666', lw=0.8, ls='--')

def draw_scene(ax, theta, phi, omega, show_rect=True, show_axes=True,
               frame_scale=None, label_axes=True):
    if show_rect:
        rc = rect_verts(theta)
        rect = Poly3DCollection([rc], alpha=0.12, facecolor='steelblue',
                                edgecolor='#4a90d9', lw=0.5)
        ax.add_collection3d(rect)

    if show_axes:
        T = top_ep(theta)
        u1, u2 = gimbal_axes(theta)
        al = 3.0
        for u, c, ls in [(u1, '#e07000', '-'), (u2, '#8855aa', '--')]:
            p1, p2 = T - al*u, T + al*u
            ax.plot([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]], c, lw=1.2, ls=ls)

    B, T = segment_ends(theta, phi, omega)
    C = center(theta, phi, omega)
    ax.plot([B[0],T[0]], [B[1],T[1]], [B[2],T[2]], '#cc2244', lw=2.2)
    ax.scatter(*C, color='#cc2244', s=60, edgecolors='#881122', lw=1, zorder=5)
    ax.scatter(*T, color='#e07000', s=35, edgecolors='#663300', lw=0.8, zorder=5)

    xh, yh, zh = target_axes(theta, phi, omega)
    sc = frame_scale if frame_scale else L*0.55
    for v, c in [(xh, '#cc2244'), (yh, '#2ca02c'), (zh, '#1f77b4')]:
        ax.quiver(*C, *(sc*v), color=c, lw=1.0, arrow_length_ratio=0.15)
    if label_axes:
        ax.text(*(C + sc*1.15*xh), r'$\hat{\mathbf{x}}_T$', color='#cc2244', fontsize=8)
        ax.text(*(C + sc*1.15*yh), r'$\hat{\mathbf{y}}_T$', color='#2ca02c', fontsize=8)
        ax.text(*(C + sc*1.15*zh), r'$\hat{\mathbf{z}}_T$', color='#1f77b4', fontsize=8)

    return C

# =====================================================================
# Figure 1 — Four key poses (start + end of each stage)
# =====================================================================
def fig_sequence():
    poses = [
        (0,           0,         0,         r'Start: $\theta{=}0,\phi{=}0,\omega{=}0$'),
        (0,           np.pi/4,   0,         r'Stage 1 end: $\phi{=}\pi/4$'),
        (0,           np.pi/4,   np.pi/2,   r'Stage 2 end: $\omega{=}\pi/2$'),
        (2*np.pi/3,   np.pi/4,   np.pi/2,   r'Stage 3 end: $\theta{=}2\pi/3$'),
    ]

    fig = plt.figure(figsize=(14, 11))
    for idx, (th, ph, om, title) in enumerate(poses):
        ax = fig.add_subplot(2, 2, idx+1, projection='3d')
        setup_3d(ax, title, view_radius=5.2)
        C = draw_scene(ax, th, ph, om, frame_scale=L*0.5)
        ax.text2D(0.02, 0.95,
                  rf'$C=({C[0]:.2f},\,{C[1]:.2f},\,{C[2]:.2f})$',
                  transform=ax.transAxes, fontsize=9, fontfamily='monospace',
                  verticalalignment='top',
                  bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.85))
        ax.view_init(elev=35.26, azim=45)

    fig.suptitle('Sequential Motion — Four Key Poses', fontsize=14, fontweight='bold', y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(OUTDIR, 'fig_sequence.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('  fig_sequence.png')

# =====================================================================
# Figure 2 — 3D trajectory of center point through full motion
# =====================================================================
def fig_trajectory():
    n1, n2, n3 = 60, 60, 80
    phi_vals = np.linspace(0, np.pi/4, n1)
    omega_vals = np.linspace(0, np.pi/2, n2)
    theta_vals = np.linspace(0, 2*np.pi/3, n3)

    pts1 = np.array([center(0, p, 0) for p in phi_vals])
    pts2 = np.array([center(0, np.pi/4, w) for w in omega_vals])
    pts3 = np.array([center(t, np.pi/4, np.pi/2) for t in theta_vals])
    all_pts = np.vstack([pts1, pts2, pts3])

    fig = plt.figure(figsize=(14, 8))
    gs = GridSpec(2, 2, figure=fig, height_ratios=[3, 1],
                  width_ratios=[2, 1], hspace=0.35, wspace=0.30)

    # 3D trajectory (spans top left + top right)
    ax = fig.add_subplot(gs[0, :], projection='3d')
    setup_3d(ax, '3D Trajectory', view_radius=5.5)

    for pts, c, label in [(pts1, '#e07000', 'Stage 1: $\phi$'),
                           (pts2, '#8855aa', 'Stage 2: $\omega$'),
                           (pts3, '#4a90d9', 'Stage 3: $\theta$')]:
        ax.plot(pts[:,0], pts[:,1], pts[:,2], c, lw=1.8, label=label)

    ax.scatter(*all_pts[0],  color='#2ca02c', s=80, marker='o', zorder=6, label='Start')
    ax.scatter(*all_pts[-1], color='#cc2244', s=80, marker='s', zorder=6, label='End')

    for label, idx in [(r'$\phi{=}\pi/4$', n1-1), (r'$\omega{=}\pi/2$', n1+n2-1)]:
        pt = all_pts[idx]
        ax.scatter(*pt, color='#333333', s=30, marker='x', zorder=5)
        ax.text(pt[0]+0.15, pt[1]+0.15, pt[2], label, fontsize=8, color='#333')

    draw_scene(ax, 2*np.pi/3, np.pi/4, np.pi/2, show_rect=False,
               show_axes=False, frame_scale=L*0.4, label_axes=False)
    ax.legend(loc='upper left', fontsize=8)
    ax.view_init(elev=35.26, azim=45)

    # Bottom left: component plots
    ax_c = fig.add_subplot(gs[1, 0])
    x_axis = np.linspace(0, 3, n1+n2+n3)
    ax_c.plot(x_axis[:n1], pts1[:,0], '#e07000', lw=1.2, label=r'$C_x$')
    ax_c.plot(x_axis[n1:n1+n2], pts2[:,0], '#8855aa', lw=1.2)
    ax_c.plot(x_axis[n1+n2:], pts3[:,0], '#4a90d9', lw=1.2)
    ax_c.plot(x_axis[:n1], pts1[:,1], '#e07000', lw=1.2, ls='--', label=r'$C_y$')
    ax_c.plot(x_axis[n1:n1+n2], pts2[:,1], '#8855aa', lw=1.2, ls='--')
    ax_c.plot(x_axis[n1+n2:], pts3[:,1], '#4a90d9', lw=1.2, ls='--')
    ax_c.plot(x_axis[:n1], pts1[:,2], '#e07000', lw=1.2, ls='-.', label=r'$C_z$')
    ax_c.plot(x_axis[n1:n1+n2], pts2[:,2], '#8855aa', lw=1.2, ls='-.')
    ax_c.plot(x_axis[n1+n2:], pts3[:,2], '#4a90d9', lw=1.2, ls='-.')
    ax_c.set_xlabel('Stage'); ax_c.set_ylabel('Component value')
    ax_c.legend(fontsize=7, ncol=3); ax_c.grid(alpha=0.3)
    ax_c.set_xticks([0.5, 1.5, 2.5])
    ax_c.set_xticklabels(['Stage 1', 'Stage 2', 'Stage 3'])

    # Bottom right: XY projection
    ax_xy = fig.add_subplot(gs[1, 1])
    ax_xy.plot(pts1[:,0], pts1[:,1], '#e07000', lw=1.2)
    ax_xy.plot(pts2[:,0], pts2[:,1], '#8855aa', lw=1.2)
    ax_xy.plot(pts3[:,0], pts3[:,1], '#4a90d9', lw=1.2)
    ax_xy.scatter(*all_pts[0,:2],  color='#2ca02c', s=40, zorder=5)
    ax_xy.scatter(*all_pts[-1,:2], color='#cc2244', s=40, zorder=5)
    ax_xy.set_xlabel(r'$C_x$'); ax_xy.set_ylabel(r'$C_y$')
    ax_xy.set_title('XY Projection', fontsize=9)
    ax_xy.grid(alpha=0.3); ax_xy.set_aspect('equal')

    fig.suptitle('Center Point Trajectory — Sequential Motion', fontsize=13, fontweight='bold')
    fig.savefig(os.path.join(OUTDIR, 'fig_trajectory.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('  fig_trajectory.png')

# =====================================================================
# Figure 3 — Transform matrix entries as functions of r, L, h
# =====================================================================
def fig_transform_func():
    r0, L0, h0 = 2.0, 1.5, 0.0  # baseline

    def t_x(rv, Lv, hv):
        return Lv - np.sqrt(2)/2 * rv
    def t_y(rv, Lv, hv):
        return Lv + hv
    def t_z(rv, Lv, hv):
        return -np.sqrt(2)/2 * rv

    r_vals = np.linspace(0.5, 5.0, 80)
    L_vals = np.linspace(0.5, 4.0, 80)
    h_vals = np.linspace(-2.0, 2.0, 80)

    fig = plt.figure(figsize=(14, 9))
    gs = GridSpec(2, 3, figure=fig, hspace=0.40, wspace=0.35)

    titles = [r'$t_x = L - \frac{\sqrt{2}}{2}r$',
              r'$t_y = L + h$',
              r'$t_z = -\frac{\sqrt{2}}{2}r$']
    colors = ['#cc2244', '#2ca02c', '#1f77b4']
    funcs = [t_x, t_y, t_z]

    # Top row: components vs each parameter in isolation
    for col, (title, fn, c) in enumerate(zip(titles, funcs, colors)):
        ax = fig.add_subplot(gs[0, col])

        tx_r = fn(r_vals, L0, h0)
        tx_L = fn(r0, L_vals, h0)
        tx_h = fn(r0, L0, h_vals)

        if np.isscalar(tx_r) or tx_r.shape != r_vals.shape:
            tx_r = np.full_like(r_vals, tx_r)
        if np.isscalar(tx_L) or tx_L.shape != L_vals.shape:
            tx_L = np.full_like(L_vals, tx_L)
        if np.isscalar(tx_h) or tx_h.shape != h_vals.shape:
            tx_h = np.full_like(h_vals, tx_h)

        ax.plot(r_vals, tx_r, '#e07000', lw=1.8, label='vary $r$')
        ax.plot(L_vals, tx_L, '#8855aa', lw=1.8, label='vary $L$')
        ax.plot(h_vals, tx_h, '#4a90d9', lw=1.8, label='vary $h$')

        # Mark baseline
        baseline_val = fn(r0, L0, h0)
        ax.axhline(baseline_val, color='#333', lw=0.6, ls='--', alpha=0.5)

        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_xlabel('Parameter value'); ax.set_ylabel('Component')
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)

    # Bottom left: 3D view — C_f positions as r varies
    ax3d = fig.add_subplot(gs[1, 0], projection='3d')
    r_samples = np.linspace(0.5, 5.0, 5)
    for ri in r_samples:
        Cv = center(2*np.pi/3, np.pi/4, np.pi/2, ri, L0, h0)
        ax3d.scatter(*Cv, color='#e07000', s=40, zorder=5)
        if ri < 3:
            ax3d.text(Cv[0]+0.1, Cv[1]+0.1, Cv[2],
                      f'r={ri:.1f}', fontsize=7, color='#e07000')
    # Connect with line
    r_dense = np.linspace(0.5, 5.0, 50)
    C_r = np.array([center(2*np.pi/3, np.pi/4, np.pi/2, ri, L0, h0)
                     for ri in r_dense])
    ax3d.plot(C_r[:,0], C_r[:,1], C_r[:,2], '#e07000', lw=1.2, alpha=0.5)
    setup_3d(ax3d, r'$C_f$ vs $r$', view_radius=6.0)
    ax3d.view_init(elev=35.26, azim=45)

    # Bottom middle: C_f positions as L varies
    ax3d2 = fig.add_subplot(gs[1, 1], projection='3d')
    L_samples = np.linspace(0.5, 4.0, 4)
    for Li in L_samples:
        Cv = center(2*np.pi/3, np.pi/4, np.pi/2, r0, Li, h0)
        ax3d2.scatter(*Cv, color='#8855aa', s=40, zorder=5)
        ax3d2.text(Cv[0]+0.1, Cv[1]+0.1, Cv[2],
                   f'L={Li:.1f}', fontsize=7, color='#8855aa')
    L_dense = np.linspace(0.5, 4.0, 50)
    C_L = np.array([center(2*np.pi/3, np.pi/4, np.pi/2, r0, Li, h0)
                     for Li in L_dense])
    ax3d2.plot(C_L[:,0], C_L[:,1], C_L[:,2], '#8855aa', lw=1.2, alpha=0.5)
    setup_3d(ax3d2, r'$C_f$ vs $L$', view_radius=6.0)
    ax3d2.view_init(elev=35.26, azim=45)

    # Bottom right: C_f positions as h varies
    ax3d3 = fig.add_subplot(gs[1, 2], projection='3d')
    h_samples = np.linspace(-2.0, 2.0, 4)
    for hi in h_samples:
        Cv = center(2*np.pi/3, np.pi/4, np.pi/2, r0, L0, hi)
        ax3d3.scatter(*Cv, color='#4a90d9', s=40, zorder=5)
        ax3d3.text(Cv[0]+0.1, Cv[1]+0.1, Cv[2],
                   f'h={hi:.1f}', fontsize=7, color='#4a90d9')
    h_dense = np.linspace(-2.0, 2.0, 50)
    C_h = np.array([center(2*np.pi/3, np.pi/4, np.pi/2, r0, L0, hi)
                     for hi in h_dense])
    ax3d3.plot(C_h[:,0], C_h[:,1], C_h[:,2], '#4a90d9', lw=1.2, alpha=0.5)
    setup_3d(ax3d3, r'$C_f$ vs $h$', view_radius=6.0)
    ax3d3.view_init(elev=35.26, azim=45)

    fig.suptitle(
        r'Transform Matrix Translation $-\mathbf{R}_T^{\mathsf{T}}C_f(r,L,h)$',
        fontsize=13, fontweight='bold')
    fig.savefig(os.path.join(OUTDIR, 'fig_transform_func.png'), dpi=150,
                bbox_inches='tight')
    plt.close(fig)
    print('  fig_transform_func.png')

# =====================================================================
# Figure 4 — Transform matrix entries as functions of motion progress
# =====================================================================
def fig_transform_evolution():
    n_per_stage = 80

    # Progress parameter ranges
    t1 = np.linspace(0, np.pi/4, n_per_stage)
    t2 = np.linspace(0, np.pi/2, n_per_stage)
    t3 = np.linspace(0, 2*np.pi/3, n_per_stage)
    t_all = np.concatenate([t1, t2 + t1[-1], t3 + t1[-1] + t2[-1]])
    stage_bounds = [0, t1[-1], t1[-1] + t2[-1], t_all[-1]]

    alpha = np.sqrt(2)/2

    def compute_T_entries(theta, phi, omega, rv, Lv, hv):
        xh, yh, zh = target_axes(theta, phi, omega)
        Rmat = np.column_stack([xh, yh, zh]).T
        Cv = center(theta, phi, omega, rv, Lv, hv)
        tv = -Rmat @ Cv
        return Rmat, tv

    R_all, t_all_vec = [], []
    for t in t1:
        Rt, tv = compute_T_entries(0, t, 0, R, L, H)
        R_all.append(Rt.flatten()); t_all_vec.append(tv)
    for t in t2:
        Rt, tv = compute_T_entries(0, np.pi/4, t, R, L, H)
        R_all.append(Rt.flatten()); t_all_vec.append(tv)
    for t in t3:
        Rt, tv = compute_T_entries(t, np.pi/4, np.pi/2, R, L, H)
        R_all.append(Rt.flatten()); t_all_vec.append(tv)

    R_all = np.array(R_all)
    t_all_vec = np.array(t_all_vec)

    R_labels = [r'$R_{11}$', r'$R_{12}$', r'$R_{13}$',
                r'$R_{21}$', r'$R_{22}$', r'$R_{23}$',
                r'$R_{31}$', r'$R_{32}$', r'$R_{33}$']
    t_labels = [r'$t_x$', r'$t_y$', r'$t_z$']
    stage_colors = {0: '#e07000', 1: '#8855aa', 2: '#4a90d9'}

    fig, axes = plt.subplots(4, 3, figsize=(14, 12))
    fig.suptitle(
        r'Evolution of ${}^{\rm world}\mathbf{T}_{\rm target}(t)$ Entries',
        fontsize=14, fontweight='bold')

    for idx in range(9):
        ax = axes[idx // 3, idx % 3]
        for s in range(3):
            sl = slice(s * n_per_stage, (s + 1) * n_per_stage)
            ax.plot(t_all[sl], R_all[sl, idx], color=stage_colors[s], lw=1.2)
        ax.set_title(R_labels[idx], fontsize=9, fontweight='bold')
        ax.grid(alpha=0.3)

    for idx in range(3):
        ax = axes[3, idx]
        for s in range(3):
            sl = slice(s * n_per_stage, (s + 1) * n_per_stage)
            ax.plot(t_all[sl], t_all_vec[sl, idx], color=stage_colors[s], lw=1.5)
        ax.set_title(t_labels[idx], fontsize=9, fontweight='bold')
        ax.set_xlabel('Motion progress $t$')
        ax.grid(alpha=0.3)

    # Stage boundary lines
    for ax_row in axes:
        for ax in ax_row:
            for b in stage_bounds[1:-1]:
                ax.axvline(b, color='#333', lw=0.6, ls='--', alpha=0.4)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(OUTDIR, 'fig_transform_evolution.png'), dpi=150,
                bbox_inches='tight')
    plt.close(fig)
    print('  fig_transform_evolution.png')

# =====================================================================
if __name__ == '__main__':
    print('Generating figures...')
    fig_sequence()
    fig_trajectory()
    fig_transform_evolution()
    print('Done.')
