/**
 * controllers.js — Control Architectures for Quadrotor
 *
 * Architecture: Cascaded control
 *   Outer loop: Position controller → desired roll/pitch + thrust
 *   Inner loop: Attitude controller → torques
 *
 * Implements:
 *   1. Cascaded PID (standard PD + integral anti-windup)
 *   2. Sliding Mode Control (signum-based, with boundary layer option)
 *   3. Super-Twisting SMC (continuous, chattering-free)
 *   4. Model Predictive Control (linearized, receding horizon)
 *
 * All controllers output u = [T, τφ, τθ, τψ]
 */

const Controllers = (() => {

    const { PARAMS, clamp } = Physics;
    const { m, g, Ixx, Iyy, Izz } = PARAMS;

    // =====================================================================
    //  GAIN STRUCTURES (defaults — UI overrides these)
    // =====================================================================

    const defaultGains = {
        PID: {
            // Position PD gains
            Kp_xy: 4.0,   Ki_xy: 0.2,  Kd_xy: 3.0,
            Kp_z:  8.0,   Ki_z:  0.5,  Kd_z:  5.0,
            // Attitude PD gains
            Kp_att: 12.0,  Kd_att: 4.0,
            Kp_yaw: 3.0,   Kd_yaw: 1.5,
            // Integral windup limit
            iMax: 2.0,
        },
        SMC: {
            // Sliding surface slope
            lambda_xy: 2.5,  lambda_z: 3.0,
            // Switching gain (η)
            eta_xy: 3.0,     eta_z: 5.0,
            // Boundary layer thickness (φ) — 0 = pure signum
            phi_xy: 0.0,     phi_z: 0.0,
            // Attitude
            lambda_att: 8.0, eta_att: 4.0,
            phi_att: 0.1,
        },
        STS: {
            // Super-twisting gains: ẋ = −α₁|s|^½ sign(s) + v,  v̇ = −α₂ sign(s)
            lambda_xy: 2.5,  lambda_z: 3.0,
            alpha1_xy: 5.0,  alpha2_xy: 3.0,
            alpha1_z: 8.0,   alpha2_z: 5.0,
            // Attitude
            lambda_att: 8.0,
            alpha1_att: 10.0, alpha2_att: 6.0,
        },
        MPC: {
            // Horizon
            N: 10,
            // State cost weights [pos, vel]
            Q_pos: 8.0,  Q_vel: 2.0,
            // Control effort weight
            R: 0.1,
            // Attitude PD (inner loop stays PD)
            Kp_att: 12.0, Kd_att: 4.0,
        }
    };

    // Deep clone
    function getDefaultGains(algo) {
        return JSON.parse(JSON.stringify(defaultGains[algo]));
    }

    // =====================================================================
    //  CONTROLLER INTERNAL STATE (integrators, sliding surfaces, etc.)
    // =====================================================================
    let intState = null;

    function resetInternal() {
        intState = {
            // PID integrators
            iEx: 0, iEy: 0, iEz: 0,
            // STS integral terms
            v_x: 0, v_y: 0, v_z: 0,
            v_phi: 0, v_theta: 0, v_psi: 0,
            // Sliding surface values (for plotting)
            s_x: 0, s_y: 0, s_z: 0,
            s_phi: 0, s_theta: 0,
            // Control effort
            T: 0, tau_phi: 0, tau_theta: 0, tau_psi: 0,
        };
    }
    resetInternal();

    function getInternalState() { return intState; }

    // =====================================================================
    //  SWITCHING FUNCTIONS
    // =====================================================================
    function sign(x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

    function sat(s, phi) {
        // Boundary layer saturation: sat(s/φ)
        if (phi <= 0) return sign(s);
        const v = s / phi;
        return Math.abs(v) <= 1 ? v : sign(v);
    }

    // =====================================================================
    //  1. CASCADED PID
    // =====================================================================
    function computePID(state, target, gains, dt) {
        const G = gains;

        // --- Position errors (world frame) ---
        const ex = target.x - state.x;
        const ey = target.y - state.y;
        const ez = target.z - state.z;

        const evx = (target.vx || 0) - state.vx;
        const evy = (target.vy || 0) - state.vy;
        const evz = (target.vz || 0) - state.vz;

        // Integrate with anti-windup
        intState.iEx = clamp(intState.iEx + ex * dt, -G.iMax, G.iMax);
        intState.iEy = clamp(intState.iEy + ey * dt, -G.iMax, G.iMax);
        intState.iEz = clamp(intState.iEz + ez * dt, -G.iMax, G.iMax);

        // Desired accelerations (world frame)
        const ax_des = G.Kp_xy * ex + G.Ki_xy * intState.iEx + G.Kd_xy * evx;
        const ay_des = G.Kp_z  * ey + G.Ki_z  * intState.iEy + G.Kd_z  * evy;
        const az_des = G.Kp_xy * ez + G.Ki_xy * intState.iEz + G.Kd_xy * evz;

        // --- Thrust & desired attitude ---
        // Total thrust T = m * (g + ay_des) / (cos φ cos θ)
        const cphi = Math.cos(state.phi), cth = Math.cos(state.theta);
        const T = clamp(m * (g + ay_des) / Math.max(cphi * cth, 0.1), 0, m * g * 4);

        // Desired roll/pitch from horizontal acceleration demands
        const psi = state.psi;
        const cpsi = Math.cos(psi), spsi = Math.sin(psi);
        const phi_des   = clamp(Math.asin(clamp(m * (ax_des * spsi - az_des * cpsi) / Math.max(T, 0.1), -0.8, 0.8)), -0.6, 0.6);
        const theta_des = clamp(Math.atan2(ax_des * cpsi + az_des * spsi, g + ay_des), -0.6, 0.6);
        const psi_des   = target.yaw || 0;

        // --- Attitude PD ---
        const tau_phi   = G.Kp_att * (phi_des - state.phi)     - G.Kd_att * state.p;
        const tau_theta = G.Kp_att * (theta_des - state.theta)  - G.Kd_att * state.q;
        const tau_psi   = G.Kp_yaw * (psi_des - state.psi)     - G.Kd_yaw * state.r;

        // Store for telemetry
        intState.s_x = ex; intState.s_y = ey; intState.s_z = ez;
        intState.T = T; intState.tau_phi = tau_phi;
        intState.tau_theta = tau_theta; intState.tau_psi = tau_psi;

        return [T, tau_phi, tau_theta, tau_psi];
    }

    // =====================================================================
    //  2. SLIDING MODE CONTROL (Standard)
    // =====================================================================
    function computeSMC(state, target, gains, dt) {
        const G = gains;

        // Position errors
        const ex = target.x - state.x, evx = (target.vx||0) - state.vx;
        const ey = target.y - state.y, evy = (target.vy||0) - state.vy;
        const ez = target.z - state.z, evz = (target.vz||0) - state.vz;

        // Sliding surfaces:  s = ė + λ·e
        const sx = evx + G.lambda_xy * ex;
        const sy = evy + G.lambda_z  * ey;
        const sz = evz + G.lambda_xy * ez;

        intState.s_x = sx; intState.s_y = sy; intState.s_z = sz;

        // Control law:  u = ueq + η·sat(s, φ)
        // Equivalent control ueq ≈ λ·ė  (linearized around hover)
        const ax_des = G.lambda_xy * evx + G.eta_xy * sat(sx, G.phi_xy);
        const ay_des = G.lambda_z  * evy + G.eta_z  * sat(sy, G.phi_z);
        const az_des = G.lambda_xy * evz + G.eta_xy * sat(sz, G.phi_xy);

        // Thrust & attitude extraction (same approach)
        const cphi = Math.cos(state.phi), cth = Math.cos(state.theta);
        const T = clamp(m * (g + ay_des) / Math.max(cphi * cth, 0.1), 0, m * g * 4);

        const psi = state.psi;
        const cpsi = Math.cos(psi), spsi = Math.sin(psi);
        const phi_des   = clamp(Math.asin(clamp(m * (ax_des * spsi - az_des * cpsi) / Math.max(T, 0.1), -0.8, 0.8)), -0.6, 0.6);
        const theta_des = clamp(Math.atan2(ax_des * cpsi + az_des * spsi, g + ay_des), -0.6, 0.6);

        // Attitude SMC
        const s_phi   = -state.p + G.lambda_att * (phi_des - state.phi);
        const s_theta = -state.q + G.lambda_att * (theta_des - state.theta);
        const s_psi   = -state.r + G.lambda_att * (0 - state.psi);

        intState.s_phi = s_phi; intState.s_theta = s_theta;

        const tau_phi   = Ixx * (G.lambda_att * (-state.p) + G.eta_att * sat(s_phi, G.phi_att));
        const tau_theta = Iyy * (G.lambda_att * (-state.q) + G.eta_att * sat(s_theta, G.phi_att));
        const tau_psi   = Izz * (G.lambda_att * (-state.r) + G.eta_att * sat(s_psi, G.phi_att));

        intState.T = T; intState.tau_phi = tau_phi;
        intState.tau_theta = tau_theta; intState.tau_psi = tau_psi;

        return [T, tau_phi, tau_theta, tau_psi];
    }

    // =====================================================================
    //  3. SUPER-TWISTING SMC
    // =====================================================================
    function computeSTS(state, target, gains, dt) {
        const G = gains;

        const ex = target.x - state.x, evx = (target.vx||0) - state.vx;
        const ey = target.y - state.y, evy = (target.vy||0) - state.vy;
        const ez = target.z - state.z, evz = (target.vz||0) - state.vz;

        // Sliding surfaces
        const sx = evx + G.lambda_xy * ex;
        const sy = evy + G.lambda_z  * ey;
        const sz = evz + G.lambda_xy * ez;
        intState.s_x = sx; intState.s_y = sy; intState.s_z = sz;

        // Super-twisting algorithm:
        //   u₁ = −α₁ |s|^(1/2) sign(s) + v
        //   v̇  = −α₂ sign(s)
        intState.v_x += -G.alpha2_xy * sign(sx) * dt;
        intState.v_y += -G.alpha2_z  * sign(sy) * dt;
        intState.v_z += -G.alpha2_xy * sign(sz) * dt;

        const ax_des = G.alpha1_xy * Math.sqrt(Math.abs(sx)) * sign(sx) + intState.v_x;
        const ay_des = G.alpha1_z  * Math.sqrt(Math.abs(sy)) * sign(sy) + intState.v_y;
        const az_des = G.alpha1_xy * Math.sqrt(Math.abs(sz)) * sign(sz) + intState.v_z;

        const cphi = Math.cos(state.phi), cth = Math.cos(state.theta);
        const T = clamp(m * (g + ay_des) / Math.max(cphi * cth, 0.1), 0, m * g * 4);

        const psi = state.psi;
        const cpsi = Math.cos(psi), spsi = Math.sin(psi);
        const phi_des   = clamp(Math.asin(clamp(m * (ax_des * spsi - az_des * cpsi) / Math.max(T, 0.1), -0.8, 0.8)), -0.6, 0.6);
        const theta_des = clamp(Math.atan2(ax_des * cpsi + az_des * spsi, g + ay_des), -0.6, 0.6);

        // Attitude super-twisting
        const s_phi   = -state.p + G.lambda_att * (phi_des - state.phi);
        const s_theta = -state.q + G.lambda_att * (theta_des - state.theta);
        const s_psi   = -state.r + G.lambda_att * (0 - state.psi);
        intState.s_phi = s_phi; intState.s_theta = s_theta;

        intState.v_phi   += -G.alpha2_att * sign(s_phi) * dt;
        intState.v_theta += -G.alpha2_att * sign(s_theta) * dt;
        intState.v_psi   += -G.alpha2_att * sign(s_psi) * dt;

        const tau_phi   = Ixx * (G.alpha1_att * Math.sqrt(Math.abs(s_phi))   * sign(s_phi)   + intState.v_phi);
        const tau_theta = Iyy * (G.alpha1_att * Math.sqrt(Math.abs(s_theta)) * sign(s_theta) + intState.v_theta);
        const tau_psi   = Izz * (G.alpha1_att * Math.sqrt(Math.abs(s_psi))   * sign(s_psi)   + (intState.v_psi || 0));

        intState.T = T; intState.tau_phi = tau_phi;
        intState.tau_theta = tau_theta; intState.tau_psi = tau_psi;

        return [T, tau_phi, tau_theta, tau_psi];
    }

    // =====================================================================
    //  4. MODEL PREDICTIVE CONTROL (Linearized)
    // =====================================================================
    function computeMPC(state, target, gains, dt, trajectoryFn, time) {
        const G = gains;
        const N = G.N;
        const dtPred = dt * 2; // prediction timestep

        // Solve via batch QP approximation (for browser performance):
        // For each axis, solve 1D double-integrator MPC
        // State: [pos, vel], Input: accel
        // Cost: Σ Q‖x-xref‖² + R‖u‖²
        // Analytical solution for unconstrained LQR-like MPC

        function solve1D(pos, vel, refFn, Q_p, Q_v, R) {
            // Build prediction: propagate forward
            let totalAccel = 0;
            let weightSum = 0;

            for (let k = 1; k <= N; k++) {
                const tFuture = time + k * dtPred;
                const ref = refFn(tFuture);
                const predPos = pos + vel * (k * dtPred);
                const errPos = ref - predPos;
                // Weighting: closer predictions matter more but not too much
                const w = 1.0 - 0.3 * (k - 1) / N;
                totalAccel += w * (Q_p * errPos - Q_v * vel);
                weightSum += w;
            }
            return totalAccel / (weightSum * (1 + R));
        }

        const ax_des = solve1D(state.x, state.vx,
            (t) => { const r = trajectoryFn ? trajectoryFn(t) : target; return r.x; },
            G.Q_pos, G.Q_vel, G.R);
        const ay_des = solve1D(state.y, state.vy,
            (t) => { const r = trajectoryFn ? trajectoryFn(t) : target; return r.y; },
            G.Q_pos, G.Q_vel, G.R);
        const az_des = solve1D(state.z, state.vz,
            (t) => { const r = trajectoryFn ? trajectoryFn(t) : target; return r.z; },
            G.Q_pos, G.Q_vel, G.R);

        // Store predicted trajectory for visualization
        intState.mpcPrediction = [];
        for (let k = 0; k <= N; k++) {
            const tF = time + k * dtPred;
            const ref = trajectoryFn ? trajectoryFn(tF) : target;
            intState.mpcPrediction.push({
                x: state.x + state.vx * (k * dtPred) + 0.5 * ax_des * (k * dtPred) ** 2,
                y: state.y + state.vy * (k * dtPred) + 0.5 * ay_des * (k * dtPred) ** 2,
                z: state.z + state.vz * (k * dtPred) + 0.5 * az_des * (k * dtPred) ** 2,
            });
        }

        // Thrust & attitude
        const cphi = Math.cos(state.phi), cth = Math.cos(state.theta);
        const T = clamp(m * (g + ay_des) / Math.max(cphi * cth, 0.1), 0, m * g * 4);

        const psi = state.psi;
        const cpsi = Math.cos(psi), spsi = Math.sin(psi);
        const phi_des   = clamp(Math.asin(clamp(m * (ax_des * spsi - az_des * cpsi) / Math.max(T, 0.1), -0.8, 0.8)), -0.6, 0.6);
        const theta_des = clamp(Math.atan2(ax_des * cpsi + az_des * spsi, g + ay_des), -0.6, 0.6);

        // Attitude PD (inner loop)
        const tau_phi   = G.Kp_att * (phi_des - state.phi)     - G.Kd_att * state.p;
        const tau_theta = G.Kp_att * (theta_des - state.theta)  - G.Kd_att * state.q;
        const tau_psi   = G.Kp_att * (0 - state.psi)           - G.Kd_att * state.r;

        intState.s_x = target.x - state.x;
        intState.s_y = target.y - state.y;
        intState.s_z = target.z - state.z;
        intState.T = T; intState.tau_phi = tau_phi;
        intState.tau_theta = tau_theta; intState.tau_psi = tau_psi;

        return [T, tau_phi, tau_theta, tau_psi];
    }

    // =====================================================================
    //  DISPATCH
    // =====================================================================
    function compute(algo, state, target, gains, dt, trajectoryFn, time) {
        switch (algo) {
            case 'PID': return computePID(state, target, gains, dt);
            case 'SMC': return computeSMC(state, target, gains, dt);
            case 'STS': return computeSTS(state, target, gains, dt);
            case 'MPC': return computeMPC(state, target, gains, dt, trajectoryFn, time);
        }
    }

    return { compute, getDefaultGains, resetInternal, getInternalState, defaultGains };
})();
