/**
 * app.js — Main Application
 *
 * Orchestrates the simulation loop, UI binding, and coordination
 * between physics engine, controllers, 3D visualization, and charts.
 */

const App = (() => {

    // --- Simulation State ---
    let simState = null;
    let simTime = 0;
    let paused = false;
    let simSpeed = 1.0;
    let currentAlgo = 'PID';
    let currentPattern = 'HOVER';
    let currentGains = {};
    let chartUpdateCounter = 0;
    let desiredPathDirty = true;

    // Environment
    let windIntensity = 0;

    // --- Custom waypoints ---
    let customWaypoints = [];

    const DT = 1 / 120; // Physics timestep (120 Hz)
    const CHART_INTERVAL = 4; // Update charts every N frames

    // =====================================================================
    //  INITIALIZATION
    // =====================================================================
    function init() {
        // Physics
        simState = Physics.createState();
        Controllers.resetInternal();

        // 3D Scene
        Drone3D.init(document.getElementById('viewport'));

        // Keyboard
        Keyboard.init();

        // Charts
        Charts.init();

        // Load default gains
        currentGains = Controllers.getDefaultGains(currentAlgo);
        buildGainPanel();

        // UI bindings
        bindUI();

        // Set initial trajectory preview
        updateDesiredPath();

        // Start loop
        requestAnimationFrame(loop);
    }

    // =====================================================================
    //  MAIN LOOP
    // =====================================================================
    function loop(timestamp) {
        requestAnimationFrame(loop);
        if (paused) {
            Drone3D.render();
            return;
        }

        const stepsPerFrame = Math.round(simSpeed * 2);
        for (let i = 0; i < stepsPerFrame; i++) {
            simStep();
        }

        // Update 3D
        Drone3D.updateDrone(simState);
        Drone3D.render();

        // Update MPC prediction visualization
        if (currentAlgo === 'MPC') {
            const iState = Controllers.getInternalState();
            if (iState.mpcPrediction) {
                Drone3D.setMPCPrediction(iState.mpcPrediction);
            }
        } else {
            Drone3D.setMPCPrediction(null);
        }

        // Update charts at reduced rate
        chartUpdateCounter++;
        if (chartUpdateCounter >= CHART_INTERVAL) {
            chartUpdateCounter = 0;
            const target = getCurrentTarget();
            const iState = Controllers.getInternalState();
            Charts.push(simTime, simState, target, iState, currentAlgo);
            Charts.update(currentAlgo);
        }

        // Telemetry overlay
        updateTelemetry();

        // Data logging
        if (DataLogger.isRecording()) {
            const target = getCurrentTarget();
            const iState = Controllers.getInternalState();
            DataLogger.log(simTime, simState, target, iState, currentAlgo);
        }

        // Regenerate desired path if needed
        if (desiredPathDirty) {
            updateDesiredPath();
            desiredPathDirty = false;
        }
    }

    function simStep() {
        simTime += DT;

        // Get target from trajectory
        const target = getCurrentTarget();

        // Wind disturbance
        const windEnv = {
            windX: windIntensity * (Math.sin(simTime * 1.7) * 0.5 + Math.sin(simTime * 0.3) * 0.5),
            windY: windIntensity * Math.sin(simTime * 0.8) * 0.3,
            windZ: windIntensity * (Math.cos(simTime * 1.2) * 0.4 + Math.sin(simTime * 2.1) * 0.3),
        };

        // Compute control input
        const trajFn = Trajectories.getTrajectoryFn(currentPattern);
        const u = Controllers.compute(
            currentAlgo, simState, target, currentGains, DT, trajFn, simTime
        );

        // Step physics
        Physics.step(simState, u, windEnv, DT);
    }

    function getCurrentTarget() {
        if (currentPattern === 'KEYBOARD') {
            return Trajectories.updateKeyboard(Keyboard.getKeys(), DT);
        } else if (currentPattern === 'CUSTOM') {
            return Trajectories.evaluateCustom(simTime);
        } else {
            return Trajectories.evaluate(currentPattern, simTime);
        }
    }

    // =====================================================================
    //  UI MANAGEMENT
    // =====================================================================
    function bindUI() {
        // Algorithm selector
        document.getElementById('algo-select').addEventListener('change', (e) => {
            currentAlgo = e.target.value;
            currentGains = Controllers.getDefaultGains(currentAlgo);
            Controllers.resetInternal();
            buildGainPanel();
            updateControllerInfo();
            document.getElementById('gain-algo-tag').textContent = currentAlgo;
        });

        // Pattern selector
        document.getElementById('pattern-select').addEventListener('change', (e) => {
            currentPattern = e.target.value;
            Keyboard.setEnabled(currentPattern === 'KEYBOARD');
            document.getElementById('keyboard-hint').style.display = currentPattern === 'KEYBOARD' ? 'block' : 'none';
            document.getElementById('custom-wp-panel').style.display = currentPattern === 'CUSTOM' ? 'block' : 'none';
            desiredPathDirty = true;
            updateTrajectoryInfo();
        });

        // Wind slider
        document.getElementById('wind-slider').addEventListener('input', (e) => {
            windIntensity = parseFloat(e.target.value);
            document.getElementById('wind-value').textContent = windIntensity.toFixed(1) + ' m/s';
        });

        // Sim speed
        document.getElementById('speed-slider').addEventListener('input', (e) => {
            simSpeed = parseFloat(e.target.value);
            document.getElementById('speed-value').textContent = simSpeed.toFixed(1) + '×';
        });

        // Buttons
        document.getElementById('btn-reset').addEventListener('click', resetSim);
        document.getElementById('btn-pause').addEventListener('click', togglePause);
        document.getElementById('btn-record').addEventListener('click', toggleRecord);
        document.getElementById('btn-export').addEventListener('click', () => DataLogger.downloadCSV());
        document.getElementById('btn-export-chart').addEventListener('click', exportChartData);

        // Custom waypoint input
        document.getElementById('btn-add-wp').addEventListener('click', addCustomWaypoint);
        document.getElementById('btn-clear-wp').addEventListener('click', () => {
            customWaypoints = [];
            updateWaypointList();
            Trajectories.setCustomWaypoints([]);
        });

        // Initialize info
        updateControllerInfo();
        updateTrajectoryInfo();
    }

    function buildGainPanel() {
        const container = document.getElementById('gains-panel');
        container.innerHTML = '';

        const gainDefs = getGainDefinitions(currentAlgo);

        gainDefs.forEach(def => {
            const row = document.createElement('div');
            row.className = 'gain-row';

            const label = document.createElement('label');
            label.textContent = def.label;
            label.title = def.tooltip || '';

            const slider = document.createElement('input');
            slider.type = 'range';
            slider.min = def.min;
            slider.max = def.max;
            slider.step = def.step;
            slider.value = currentGains[def.key];

            const valueSpan = document.createElement('span');
            valueSpan.className = 'gain-value';
            valueSpan.textContent = parseFloat(currentGains[def.key]).toFixed(def.decimals || 1);

            slider.addEventListener('input', () => {
                currentGains[def.key] = parseFloat(slider.value);
                valueSpan.textContent = parseFloat(slider.value).toFixed(def.decimals || 1);
            });

            row.appendChild(label);
            row.appendChild(slider);
            row.appendChild(valueSpan);
            container.appendChild(row);
        });
    }

    function getGainDefinitions(algo) {
        switch (algo) {
            case 'PID':
                return [
                    { key: 'Kp_xy', label: 'Kp (XZ position)', min: 0, max: 20, step: 0.1, tooltip: 'Proportional gain — position loop (horizontal)' },
                    { key: 'Ki_xy', label: 'Ki (XZ integral)', min: 0, max: 2, step: 0.01, decimals: 2, tooltip: 'Integral gain — eliminates steady-state error' },
                    { key: 'Kd_xy', label: 'Kd (XZ derivative)', min: 0, max: 15, step: 0.1, tooltip: 'Derivative gain — damping for horizontal axes' },
                    { key: 'Kp_z', label: 'Kp (altitude)', min: 0, max: 30, step: 0.1, tooltip: 'Proportional gain — altitude loop' },
                    { key: 'Ki_z', label: 'Ki (altitude)', min: 0, max: 3, step: 0.01, decimals: 2, tooltip: 'Integral gain — altitude steady-state error' },
                    { key: 'Kd_z', label: 'Kd (altitude)', min: 0, max: 15, step: 0.1, tooltip: 'Derivative gain — altitude damping' },
                    { key: 'Kp_att', label: 'Kp (attitude)', min: 0, max: 30, step: 0.5, tooltip: 'Attitude proportional gain — inner loop' },
                    { key: 'Kd_att', label: 'Kd (attitude)', min: 0, max: 15, step: 0.1, tooltip: 'Attitude derivative gain' },
                ];
            case 'SMC':
                return [
                    { key: 'lambda_xy', label: 'λ (XZ surface slope)', min: 0.1, max: 10, step: 0.1, tooltip: 'Sliding surface slope — determines convergence rate on the surface' },
                    { key: 'lambda_z', label: 'λ (altitude slope)', min: 0.1, max: 10, step: 0.1, tooltip: 'Altitude sliding surface slope' },
                    { key: 'eta_xy', label: 'η (XZ switching gain)', min: 0.1, max: 15, step: 0.1, tooltip: 'Switching gain — must exceed matched uncertainty bound' },
                    { key: 'eta_z', label: 'η (altitude switching)', min: 0.1, max: 15, step: 0.1, tooltip: 'Altitude switching gain' },
                    { key: 'phi_xy', label: 'φ (XZ boundary layer)', min: 0, max: 2, step: 0.01, decimals: 2, tooltip: 'Boundary layer thickness — 0 = pure sign(), >0 = sat() for chattering reduction' },
                    { key: 'phi_z', label: 'φ (altitude boundary)', min: 0, max: 2, step: 0.01, decimals: 2, tooltip: 'Altitude boundary layer' },
                    { key: 'lambda_att', label: 'λ (attitude)', min: 1, max: 20, step: 0.5, tooltip: 'Attitude sliding surface slope' },
                    { key: 'eta_att', label: 'η (attitude switching)', min: 0.5, max: 15, step: 0.5, tooltip: 'Attitude switching gain' },
                ];
            case 'STS':
                return [
                    { key: 'lambda_xy', label: 'λ (XZ surface slope)', min: 0.1, max: 10, step: 0.1, tooltip: 'Sliding surface slope parameter' },
                    { key: 'lambda_z', label: 'λ (altitude slope)', min: 0.1, max: 10, step: 0.1 },
                    { key: 'alpha1_xy', label: 'α₁ (XZ)', min: 0.1, max: 20, step: 0.1, tooltip: 'Super-twisting gain — proportional to |s|^(1/2)' },
                    { key: 'alpha2_xy', label: 'α₂ (XZ)', min: 0.1, max: 15, step: 0.1, tooltip: 'Super-twisting integral gain — drives s to zero' },
                    { key: 'alpha1_z', label: 'α₁ (altitude)', min: 0.1, max: 20, step: 0.1, tooltip: 'Altitude super-twisting gain' },
                    { key: 'alpha2_z', label: 'α₂ (altitude)', min: 0.1, max: 15, step: 0.1, tooltip: 'Altitude integral gain' },
                    { key: 'lambda_att', label: 'λ (attitude)', min: 1, max: 20, step: 0.5 },
                    { key: 'alpha1_att', label: 'α₁ (attitude)', min: 1, max: 25, step: 0.5 },
                ];
            case 'MPC':
                return [
                    { key: 'N', label: 'N (horizon steps)', min: 3, max: 30, step: 1, decimals: 0, tooltip: 'Prediction horizon length — longer = better anticipation, more computation' },
                    { key: 'Q_pos', label: 'Q (position weight)', min: 0.1, max: 30, step: 0.1, tooltip: 'State cost weight for position error' },
                    { key: 'Q_vel', label: 'Q (velocity weight)', min: 0.1, max: 10, step: 0.1, tooltip: 'State cost weight for velocity error' },
                    { key: 'R', label: 'R (control effort)', min: 0.01, max: 5, step: 0.01, decimals: 2, tooltip: 'Control effort penalty — higher = smoother but slower response' },
                    { key: 'Kp_att', label: 'Kp (attitude inner)', min: 1, max: 30, step: 0.5, tooltip: 'Attitude PD inner loop proportional gain' },
                    { key: 'Kd_att', label: 'Kd (attitude inner)', min: 0.5, max: 15, step: 0.5, tooltip: 'Attitude PD inner loop derivative gain' },
                ];
        }
        return [];
    }

    function updateControllerInfo() {
        const descriptions = {
            PID: `<strong>Cascaded PID Controller</strong><br>
                Classical proportional-integral-derivative control with cascaded position/attitude loops. 
                The outer PD loop generates desired roll/pitch angles from position error; the inner PD loop 
                tracks these angles to produce torque commands. Integral action eliminates steady-state error 
                but may cause wind-up under saturation.<br>
                <em>Strengths:</em> Simple tuning, well-understood stability margins<br>
                <em>Weaknesses:</em> Linear — no robustness guarantees against model uncertainty`,
            SMC: `<strong>Sliding Mode Controller</strong><br>
                Defines a sliding surface <code>s = ė + λe</code> in the error state space. The discontinuous 
                control law <code>u = u_eq + η·sign(s)</code> drives the state to the surface in finite time, 
                then constrains it there (sliding phase). Invariant to matched uncertainties once on the surface.<br>
                <em>Strengths:</em> Robust to bounded disturbances and model mismatch<br>
                <em>Weaknesses:</em> Chattering from high-frequency switching (set φ > 0 for boundary layer)`,
            STS: `<strong>Super-Twisting SMC</strong><br>
                A second-order sliding mode algorithm that achieves finite-time convergence to s = 0 without 
                requiring the sign function in the control signal. The control law 
                <code>u = −α₁|s|^½ sign(s) + v, v̇ = −α₂sign(s)</code> provides continuous control while 
                maintaining robustness. The integral term v builds up to compensate for constant disturbances.<br>
                <em>Strengths:</em> Chattering-free, robust, continuous control signal<br>
                <em>Weaknesses:</em> Slower convergence than standard SMC, requires careful gain tuning (α₁² ≥ 4α₂)`,
            MPC: `<strong>Model Predictive Control</strong><br>
                At each timestep, solves an optimization over a receding N-step horizon, minimizing 
                <code>J = Σ Q‖x−x_ref‖² + R‖u‖²</code>. Uses a linearized double-integrator model for 
                real-time tractability. Only the first control input is applied; the problem is re-solved at the 
                next step. The yellow prediction line shows the anticipated trajectory.<br>
                <em>Strengths:</em> Handles constraints, anticipates future trajectory changes<br>
                <em>Weaknesses:</em> Computational cost scales with horizon, model-dependent`
        };

        document.getElementById('controller-info').innerHTML = descriptions[currentAlgo] || '';
    }

    function updateTrajectoryInfo() {
        const patterns = Trajectories.getPatterns();
        const pat = patterns[currentPattern];
        const desc = pat ? pat.description : (currentPattern === 'KEYBOARD' ?
            'Manual piloting via keyboard. The controller tracks your commanded position.' :
            'Place waypoints manually. The drone will traverse them in sequence.');
        document.getElementById('trajectory-info').textContent = desc;
    }

    function updateDesiredPath() {
        if (currentPattern === 'KEYBOARD' || currentPattern === 'CUSTOM') {
            Drone3D.setDesiredPath([]);
            return;
        }
        const points = Trajectories.generatePreview(currentPattern, 300, 40);
        Drone3D.setDesiredPath(points);
    }

    function updateTelemetry() {
        const s = simState;
        const target = getCurrentTarget();
        const iState = Controllers.getInternalState();

        document.getElementById('tel-alt').textContent = s.y.toFixed(2);
        document.getElementById('tel-x').textContent = s.x.toFixed(2);
        document.getElementById('tel-z').textContent = s.z.toFixed(2);
        document.getElementById('tel-vx').textContent = s.vx.toFixed(2);
        document.getElementById('tel-vy').textContent = s.vy.toFixed(2);
        document.getElementById('tel-vz').textContent = s.vz.toFixed(2);
        document.getElementById('tel-phi').textContent = (s.phi * 180/Math.PI).toFixed(1) + '°';
        document.getElementById('tel-theta').textContent = (s.theta * 180/Math.PI).toFixed(1) + '°';
        document.getElementById('tel-psi').textContent = (s.psi * 180/Math.PI).toFixed(1) + '°';

        const ex = target.x - s.x, ey = target.y - s.y, ez = target.z - s.z;
        const errNorm = Math.sqrt(ex*ex + ey*ey + ez*ez);
        document.getElementById('tel-err').textContent = errNorm.toFixed(3);
        document.getElementById('tel-thrust').textContent = iState.T.toFixed(2);
        document.getElementById('tel-time').textContent = simTime.toFixed(1);

        // Recording indicator
        const recEl = document.getElementById('rec-indicator');
        if (DataLogger.isRecording()) {
            recEl.style.display = 'inline';
            recEl.textContent = `● REC (${DataLogger.getRecordCount()})`;
        } else {
            recEl.style.display = 'none';
        }
    }

    // =====================================================================
    //  ACTIONS
    // =====================================================================
    function resetSim() {
        simState = Physics.createState();
        simTime = 0;
        Controllers.resetInternal();
        Drone3D.clearTrail();
        Charts.clear();
        Trajectories.resetKeyboard();
        desiredPathDirty = true;
    }

    function togglePause() {
        paused = !paused;
        document.getElementById('btn-pause').textContent = paused ? '▶ Resume' : '❚❚ Pause';
        document.getElementById('btn-pause').classList.toggle('active', paused);
    }

    function toggleRecord() {
        if (DataLogger.isRecording()) {
            DataLogger.stopRecording();
            document.getElementById('btn-record').textContent = '⏺ Record';
            document.getElementById('btn-record').classList.remove('recording');
        } else {
            DataLogger.startRecording(simTime);
            document.getElementById('btn-record').textContent = '⏹ Stop';
            document.getElementById('btn-record').classList.add('recording');
        }
    }

    function exportChartData() {
        const csv = Charts.exportCSV();
        const blob = new Blob([csv], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `chart_data_${currentAlgo}_${currentPattern}_${Date.now()}.csv`;
        a.click();
        URL.revokeObjectURL(url);
    }

    function addCustomWaypoint() {
        const xVal = parseFloat(document.getElementById('wp-x').value) || 0;
        const yVal = parseFloat(document.getElementById('wp-y').value) || 3;
        const zVal = parseFloat(document.getElementById('wp-z').value) || 0;
        customWaypoints.push([xVal, yVal, zVal]);
        Trajectories.setCustomWaypoints(customWaypoints);
        updateWaypointList();

        // Show desired path
        if (customWaypoints.length > 1) {
            const pts = [...customWaypoints, customWaypoints[0]]; // Close the loop
            Drone3D.setDesiredPath(pts);
        }
    }

    function updateWaypointList() {
        const list = document.getElementById('wp-list');
        list.innerHTML = '';
        customWaypoints.forEach((wp, i) => {
            const item = document.createElement('div');
            item.className = 'wp-item';
            item.innerHTML = `<span>WP${i+1}: (${wp[0]}, ${wp[1]}, ${wp[2]})</span>
                <button class="wp-remove" data-idx="${i}">✕</button>`;
            list.appendChild(item);
        });
        // Remove buttons
        list.querySelectorAll('.wp-remove').forEach(btn => {
            btn.addEventListener('click', () => {
                customWaypoints.splice(parseInt(btn.dataset.idx), 1);
                Trajectories.setCustomWaypoints(customWaypoints);
                updateWaypointList();
            });
        });
    }

    return { init };
})();

// Launch
document.addEventListener('DOMContentLoaded', () => App.init());
