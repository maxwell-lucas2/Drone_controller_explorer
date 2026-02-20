/**
 * trajectories.js — Reference Trajectory Generation
 *
 * Predefined:
 *   HOVER     — Fixed setpoint
 *   CIRCLE    — Horizontal circle
 *   HELIX     — Ascending helix
 *   FIGURE8   — Lemniscate of Gerono
 *   SQUARE    — Waypoint sequence with linear interpolation
 *
 * Custom:
 *   CUSTOM    — User-placed 3D waypoints
 *   KEYBOARD  — Manual keyboard piloting
 */

const Trajectories = (() => {

    // Current waypoint list for custom mode
    let customWaypoints = [];
    let customSpeed = 1.0;  // m/s traversal speed
    let customIdx = 0;
    let customT = 0;

    // Keyboard accumulator
    let kbTarget = { x: 0, y: 2, z: 0, yaw: 0 };
    let kbVel = { x: 0, y: 0, z: 0, yaw: 0 };

    const patterns = {
        HOVER: {
            name: 'Hover (Fixed Setpoint)',
            params: { x: 0, y: 3, z: 0 },
            fn(t, p) {
                return { x: p.x, y: p.y, z: p.z, vx: 0, vy: 0, vz: 0, yaw: 0 };
            },
            description: 'Stationary hover — ideal for comparing transient response and steady-state error between controllers.'
        },
        CIRCLE: {
            name: 'Horizontal Circle',
            params: { radius: 4, altitude: 3, speed: 0.5 },
            fn(t, p) {
                const w = p.speed;
                return {
                    x:  p.radius * Math.cos(w * t),
                    y:  p.altitude,
                    z:  p.radius * Math.sin(w * t),
                    vx: -p.radius * w * Math.sin(w * t),
                    vy: 0,
                    vz:  p.radius * w * Math.cos(w * t),
                    yaw: 0
                };
            },
            description: 'Constant curvature path — tests centripetal acceleration tracking and coupling between axes.'
        },
        HELIX: {
            name: 'Ascending Helix',
            params: { radius: 3, climbRate: 0.3, speed: 0.4 },
            fn(t, p) {
                const w = p.speed;
                return {
                    x:  p.radius * Math.cos(w * t),
                    y:  1 + p.climbRate * t,
                    z:  p.radius * Math.sin(w * t),
                    vx: -p.radius * w * Math.sin(w * t),
                    vy: p.climbRate,
                    vz:  p.radius * w * Math.cos(w * t),
                    yaw: 0
                };
            },
            description: 'Combined circular + vertical tracking — exposes altitude controller coupling with lateral dynamics.'
        },
        FIGURE8: {
            name: 'Lemniscate (Figure-8)',
            params: { scaleX: 5, scaleZ: 3, altitude: 3.5, speed: 0.4 },
            fn(t, p) {
                const w = p.speed;
                // Lemniscate of Gerono: x = cos(t), z = sin(2t)/2
                return {
                    x: p.scaleX * Math.cos(w * t),
                    y: p.altitude + 0.5 * Math.sin(w * t * 0.5),
                    z: p.scaleZ * Math.sin(2 * w * t) / 2,
                    vx: -p.scaleX * w * Math.sin(w * t),
                    vy: 0.5 * 0.5 * w * Math.cos(w * t * 0.5),
                    vz: p.scaleZ * w * Math.cos(2 * w * t),
                    yaw: 0
                };
            },
            description: 'Self-intersecting path with varying curvature — aggressive test for tracking fidelity and phase lag.'
        },
        SQUARE: {
            name: 'Waypoint Square',
            params: { size: 4, altitude: 3, holdTime: 3 },
            fn(t, p) {
                const corners = [
                    [p.size, p.altitude, p.size],
                    [-p.size, p.altitude, p.size],
                    [-p.size, p.altitude, -p.size],
                    [p.size, p.altitude, -p.size],
                ];
                const totalPeriod = corners.length * p.holdTime;
                const phase = (t % totalPeriod);
                const segIdx = Math.floor(phase / p.holdTime);
                const segFrac = (phase % p.holdTime) / p.holdTime;

                const curr = corners[segIdx % corners.length];
                const next = corners[(segIdx + 1) % corners.length];

                // Smooth cubic interpolation
                const s = segFrac * segFrac * (3 - 2 * segFrac); // smoothstep

                return {
                    x: curr[0] + (next[0] - curr[0]) * s,
                    y: curr[1] + (next[1] - curr[1]) * s,
                    z: curr[2] + (next[2] - curr[2]) * s,
                    vx: 0, vy: 0, vz: 0, yaw: 0
                };
            },
            description: 'Discontinuous velocity reference — tests controller response to step-like setpoint changes.'
        },
        STEP: {
            name: 'Step Response',
            params: { startAlt: 1, endAlt: 4, stepTime: 3 },
            fn(t, p) {
                const y = t < p.stepTime ? p.startAlt : p.endAlt;
                return { x: 0, y: y, z: 0, vx: 0, vy: 0, vz: 0, yaw: 0 };
            },
            description: 'Classical step input — directly measures rise time, overshoot, settling time, and steady-state error.'
        },
    };

    /**
     * Evaluate a predefined trajectory at time t
     */
    function evaluate(patternKey, t) {
        const pat = patterns[patternKey];
        if (!pat) return { x: 0, y: 3, z: 0, vx: 0, vy: 0, vz: 0, yaw: 0 };
        return pat.fn(t, pat.params);
    }

    /**
     * Get trajectory function for MPC lookahead
     */
    function getTrajectoryFn(patternKey) {
        const pat = patterns[patternKey];
        if (!pat) return (t) => ({ x: 0, y: 3, z: 0 });
        return (t) => pat.fn(t, pat.params);
    }

    /**
     * Generate preview points for the desired path visualization
     */
    function generatePreview(patternKey, numPoints, duration) {
        const points = [];
        const pat = patterns[patternKey];
        if (!pat) return points;
        for (let i = 0; i <= numPoints; i++) {
            const t = (i / numPoints) * duration;
            const p = pat.fn(t, pat.params);
            points.push([p.x, p.y, p.z]);
        }
        return points;
    }

    // --- Custom Waypoints ---
    function setCustomWaypoints(wps) {
        customWaypoints = wps;
        customIdx = 0;
        customT = 0;
    }

    function evaluateCustom(t) {
        if (customWaypoints.length === 0) return { x: 0, y: 3, z: 0, vx: 0, vy: 0, vz: 0, yaw: 0 };
        if (customWaypoints.length === 1) {
            const w = customWaypoints[0];
            return { x: w[0], y: w[1], z: w[2], vx: 0, vy: 0, vz: 0, yaw: 0 };
        }

        // Linear interpolation along waypoints at constant speed
        const curr = customWaypoints[customIdx % customWaypoints.length];
        const next = customWaypoints[(customIdx + 1) % customWaypoints.length];
        const dx = next[0] - curr[0], dy = next[1] - curr[1], dz = next[2] - curr[2];
        const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        const segTime = dist / customSpeed;

        customT += 0.016;
        const frac = Math.min(customT / segTime, 1);

        if (frac >= 1) {
            customIdx = (customIdx + 1) % customWaypoints.length;
            customT = 0;
        }

        const s = frac * frac * (3 - 2 * frac);
        return {
            x: curr[0] + dx * s,
            y: curr[1] + dy * s,
            z: curr[2] + dz * s,
            vx: 0, vy: 0, vz: 0, yaw: 0
        };
    }

    // --- Keyboard Control ---
    function updateKeyboard(keys, dt) {
        const speed = 3.0;
        const yawSpeed = 1.5;

        kbVel.x = 0; kbVel.y = 0; kbVel.z = 0; kbVel.yaw = 0;

        if (keys['KeyW'] || keys['ArrowUp'])    kbVel.z = -speed;
        if (keys['KeyS'] || keys['ArrowDown'])  kbVel.z =  speed;
        if (keys['KeyA'] || keys['ArrowLeft'])   kbVel.x = -speed;
        if (keys['KeyD'] || keys['ArrowRight']) kbVel.x =  speed;
        if (keys['Space'])                       kbVel.y =  speed;
        if (keys['ShiftLeft'] || keys['ShiftRight']) kbVel.y = -speed;
        if (keys['KeyQ'])                        kbVel.yaw = -yawSpeed;
        if (keys['KeyE'])                        kbVel.yaw =  yawSpeed;

        kbTarget.x   += kbVel.x * dt;
        kbTarget.y   = Math.max(0, kbTarget.y + kbVel.y * dt);
        kbTarget.z   += kbVel.z * dt;
        kbTarget.yaw += kbVel.yaw * dt;

        return {
            x: kbTarget.x, y: kbTarget.y, z: kbTarget.z,
            vx: kbVel.x, vy: kbVel.y, vz: kbVel.z,
            yaw: kbTarget.yaw
        };
    }

    function resetKeyboard() {
        kbTarget = { x: 0, y: 2, z: 0, yaw: 0 };
    }

    function getPatterns() { return patterns; }
    function setPatternParam(key, param, value) {
        if (patterns[key]) patterns[key].params[param] = value;
    }

    return {
        evaluate, getTrajectoryFn, generatePreview, getPatterns,
        setPatternParam, setCustomWaypoints, evaluateCustom,
        updateKeyboard, resetKeyboard
    };
})();
