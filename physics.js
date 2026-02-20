/**
 * physics.js — 6-DOF Rigid Body Quadrotor Dynamics
 * 
 * State vector: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
 *   Position (x,y,z) in world frame
 *   Velocity (vx,vy,vz) in world frame
 *   Euler angles (φ=roll, θ=pitch, ψ=yaw) ZYX convention
 *   Angular rates (p,q,r) in body frame
 *
 * Inputs: [T, τ_φ, τ_θ, τ_ψ] — total thrust + torques
 *
 * References:
 *   Bouabdallah, "Design and control of quadrotors with application to autonomous flying," EPFL 2007
 */

const Physics = (() => {

    // --- Quadrotor Parameters ---
    const PARAMS = {
        m:    0.5,        // mass [kg] (Crazyflie-scale)
        g:    9.81,       // gravity [m/s²]
        Ixx:  0.0023,     // roll inertia [kg·m²]
        Iyy:  0.0023,     // pitch inertia
        Izz:  0.004,      // yaw inertia
        L:    0.17,       // arm length [m]
        kT:   2.98e-6,    // thrust coefficient [N/(rad/s)²]
        kD:   1.14e-7,    // drag coefficient [N·m/(rad/s)²]
        Cd:   0.04,       // translational drag coefficient
        wMax: 2200,       // max motor speed [rad/s]
        wMin: 0,          // min motor speed
    };

    // Motor mixing for X-configuration:
    //   Motor layout (top view):
    //     1(CW)   2(CCW)
    //        \ /
    //        / \
    //     3(CCW) 4(CW)
    //
    //   T   = kT*(w1² + w2² + w3² + w4²)
    //   τ_φ = kT*L*(−w1² − w2² + w3² + w4²) / √2
    //   τ_θ = kT*L*(−w1² + w2² + w3² − w4²) / √2
    //   τ_ψ = kD*(−w1² + w2² − w3² + w4²)

    function createState() {
        return {
            x: 0, y: 0, z: 0,
            vx: 0, vy: 0, vz: 0,
            phi: 0, theta: 0, psi: 0,
            p: 0, q: 0, r: 0,
            // Motor speeds (for visualization)
            motors: [0, 0, 0, 0],
        };
    }

    function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

    /**
     * Convert desired [T, τφ, τθ, τψ] → motor speeds squared
     * Inverse of the mixing matrix
     */
    function allocateMotors(T, tauPhi, tauTheta, tauPsi) {
        const { kT, kD, L } = PARAMS;
        const s2 = Math.SQRT2;
        // Invert mixing matrix
        const a = T / (4 * kT);
        const b = tauPhi * s2 / (4 * kT * L);
        const c = tauTheta * s2 / (4 * kT * L);
        const d = tauPsi / (4 * kD);

        let w1sq = a - b - c - d;
        let w2sq = a - b + c + d;
        let w3sq = a + b + c - d;
        let w4sq = a + b - c + d;

        const wMaxSq = PARAMS.wMax * PARAMS.wMax;
        w1sq = clamp(w1sq, 0, wMaxSq);
        w2sq = clamp(w2sq, 0, wMaxSq);
        w3sq = clamp(w3sq, 0, wMaxSq);
        w4sq = clamp(w4sq, 0, wMaxSq);

        return [Math.sqrt(w1sq), Math.sqrt(w2sq), Math.sqrt(w3sq), Math.sqrt(w4sq)];
    }

    /**
     * Full 6-DOF dynamics step (RK4 integration)
     * @param {object} s  - state
     * @param {array} u   - [T, τφ, τθ, τψ]
     * @param {object} env - { windX, windY, windZ }
     * @param {number} dt
     */
    function step(s, u, env, dt) {
        // RK4
        const k1 = derivatives(s, u, env);
        const s2 = addScaled(s, k1, dt / 2);
        const k2 = derivatives(s2, u, env);
        const s3 = addScaled(s, k2, dt / 2);
        const k3 = derivatives(s3, u, env);
        const s4 = addScaled(s, k3, dt);
        const k4 = derivatives(s4, u, env);

        // Combine
        s.x     += dt / 6 * (k1.dx + 2*k2.dx + 2*k3.dx + k4.dx);
        s.y     += dt / 6 * (k1.dy + 2*k2.dy + 2*k3.dy + k4.dy);
        s.z     += dt / 6 * (k1.dz + 2*k2.dz + 2*k3.dz + k4.dz);
        s.vx    += dt / 6 * (k1.dvx + 2*k2.dvx + 2*k3.dvx + k4.dvx);
        s.vy    += dt / 6 * (k1.dvy + 2*k2.dvy + 2*k3.dvy + k4.dvy);
        s.vz    += dt / 6 * (k1.dvz + 2*k2.dvz + 2*k3.dvz + k4.dvz);
        s.phi   += dt / 6 * (k1.dphi + 2*k2.dphi + 2*k3.dphi + k4.dphi);
        s.theta += dt / 6 * (k1.dtheta + 2*k2.dtheta + 2*k3.dtheta + k4.dtheta);
        s.psi   += dt / 6 * (k1.dpsi + 2*k2.dpsi + 2*k3.dpsi + k4.dpsi);
        s.p     += dt / 6 * (k1.dp + 2*k2.dp + 2*k3.dp + k4.dp);
        s.q     += dt / 6 * (k1.dq + 2*k2.dq + 2*k3.dq + k4.dq);
        s.r     += dt / 6 * (k1.dr + 2*k2.dr + 2*k3.dr + k4.dr);

        // Ground contact
        if (s.y < 0) { s.y = 0; s.vy = Math.max(0, s.vy); }

        // Store motor speeds for visual
        s.motors = allocateMotors(u[0], u[1], u[2], u[3]);

        return s;
    }

    function derivatives(s, u, env) {
        const { m, g, Ixx, Iyy, Izz, Cd } = PARAMS;
        const [T, tphi, ttheta, tpsi] = u;

        const cphi = Math.cos(s.phi), sphi = Math.sin(s.phi);
        const cth  = Math.cos(s.theta), sth = Math.sin(s.theta);
        const cpsi = Math.cos(s.psi), spsi = Math.sin(s.psi);

        // Rotation matrix body→world (ZYX)
        // Thrust is along body z-axis → world components
        const Tx = T * (cpsi * sth * cphi + spsi * sphi);
        const Ty = T * (cth * cphi);
        const Tz = T * (spsi * sth * cphi - cpsi * sphi);

        // Translational dynamics with drag and wind
        const dvx = (Tx / m) - Cd * s.vx + (env.windX || 0);
        const dvy = (Ty / m) - g - Cd * s.vy + (env.windY || 0);
        const dvz = (Tz / m) - Cd * s.vz + (env.windZ || 0);

        // Euler angle rates from body angular velocities
        // (small angle approximation avoided — full kinematics)
        const dphi   = s.p + (sphi * sth / cth) * s.q + (cphi * sth / cth) * s.r;
        const dtheta = cphi * s.q - sphi * s.r;
        const dpsi   = (sphi / cth) * s.q + (cphi / cth) * s.r;

        // Rotational dynamics (Euler equations)
        const dp = (tphi  - (Izz - Iyy) * s.q * s.r) / Ixx;
        const dq = (ttheta - (Ixx - Izz) * s.p * s.r) / Iyy;
        const dr = (tpsi  - (Iyy - Ixx) * s.p * s.q) / Izz;

        return { dx: s.vx, dy: s.vy, dz: s.vz, dvx, dvy, dvz, dphi, dtheta, dpsi, dp, dq, dr };
    }

    function addScaled(s, k, h) {
        return {
            x: s.x + k.dx * h, y: s.y + k.dy * h, z: s.z + k.dz * h,
            vx: s.vx + k.dvx * h, vy: s.vy + k.dvy * h, vz: s.vz + k.dvz * h,
            phi: s.phi + k.dphi * h, theta: s.theta + k.dtheta * h, psi: s.psi + k.dpsi * h,
            p: s.p + k.dp * h, q: s.q + k.dq * h, r: s.r + k.dr * h,
        };
    }

    return { PARAMS, createState, step, allocateMotors, clamp };
})();
