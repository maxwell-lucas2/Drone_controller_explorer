/**
 * charts.js — Telemetry & Diagnostic Charts
 *
 * Always-on charts:
 *   - Position Tracking: Desired vs Actual for X, Y, Z
 *   - 3D Error Norm over time
 *
 * Controller-specific:
 *   PID:  Control effort, integral accumulation
 *   SMC:  Sliding surface values, chattering visualization
 *   STS:  Sliding surface (smooth), integral term convergence
 *   MPC:  Prediction horizon cost
 *
 * Phase portrait always available
 */

const Charts = (() => {

    const MAX_POINTS = 200;
    let charts = {};
    let dataBuffers = {};

    // Shared chart options
    const baseOpts = {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        elements: { point: { radius: 0 }, line: { borderWidth: 1.5 } },
        scales: {
            x: {
                display: true,
                grid: { color: 'rgba(255,255,255,0.05)' },
                ticks: { color: '#667', font: { size: 9 }, maxTicksLimit: 6 },
            },
            y: {
                grid: { color: 'rgba(255,255,255,0.05)' },
                ticks: { color: '#667', font: { size: 9 }, maxTicksLimit: 5 },
            }
        },
        plugins: {
            legend: {
                labels: { color: '#aab', font: { size: 9 }, boxWidth: 12, padding: 6 },
                position: 'top',
            }
        }
    };

    function init() {
        dataBuffers = {
            time: [],
            // Desired
            xDes: [], yDes: [], zDes: [],
            // Actual
            xAct: [], yAct: [], zAct: [],
            // Error norm
            errNorm: [],
            // Velocities
            vx: [], vy: [], vz: [],
            // Control
            thrust: [], tauPhi: [], tauTheta: [], tauPsi: [],
            // Sliding surfaces
            sx: [], sy: [], sz: [],
            // Phase: error vs velocity
            phaseErrY: [], phaseVelY: [],
            // Motor speeds
            m1: [], m2: [], m3: [], m4: [],
        };

        createPositionChart();
        createErrorChart();
        createControlChart();
        createPhaseChart();
        createMotorChart();
    }

    function createPositionChart() {
        const ctx = document.getElementById('chart-position');
        if (!ctx) return;
        charts.position = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'X desired', data: [], borderColor: '#ff4466', borderDash: [4, 2] },
                    { label: 'X actual',  data: [], borderColor: '#ff4466' },
                    { label: 'Y desired', data: [], borderColor: '#44ff66', borderDash: [4, 2] },
                    { label: 'Y actual',  data: [], borderColor: '#44ff66' },
                    { label: 'Z desired', data: [], borderColor: '#4466ff', borderDash: [4, 2] },
                    { label: 'Z actual',  data: [], borderColor: '#4466ff' },
                ]
            },
            options: {
                ...baseOpts,
                scales: {
                    ...baseOpts.scales,
                    x: { ...baseOpts.scales.x, title: { display: true, text: 'Time (s)', color: '#667', font: { size: 9 } } },
                    y: { ...baseOpts.scales.y, title: { display: true, text: 'Position (m)', color: '#667', font: { size: 9 } } }
                }
            }
        });
    }

    function createErrorChart() {
        const ctx = document.getElementById('chart-error');
        if (!ctx) return;
        charts.error = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: '‖e‖ (3D error)', data: [], borderColor: '#ff6622', fill: true, backgroundColor: 'rgba(255,102,34,0.1)' },
                    { label: 'Sliding S_z / Integral', data: [], borderColor: '#00d4ff' },
                ]
            },
            options: {
                ...baseOpts,
                scales: {
                    ...baseOpts.scales,
                    x: { ...baseOpts.scales.x, title: { display: true, text: 'Time (s)', color: '#667', font: { size: 9 } } },
                    y: { ...baseOpts.scales.y, title: { display: true, text: 'Magnitude', color: '#667', font: { size: 9 } } }
                }
            }
        });
    }

    function createControlChart() {
        const ctx = document.getElementById('chart-control');
        if (!ctx) return;
        charts.control = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Thrust (N)', data: [], borderColor: '#ffaa00' },
                    { label: 'τ_φ (N·m)', data: [], borderColor: '#ff4466' },
                    { label: 'τ_θ (N·m)', data: [], borderColor: '#44ff66' },
                    { label: 'τ_ψ (N·m)', data: [], borderColor: '#4466ff' },
                ]
            },
            options: {
                ...baseOpts,
                scales: {
                    ...baseOpts.scales,
                    x: { ...baseOpts.scales.x, title: { display: true, text: 'Time (s)', color: '#667', font: { size: 9 } } },
                    y: { ...baseOpts.scales.y, title: { display: true, text: 'Force/Torque', color: '#667', font: { size: 9 } } }
                }
            }
        });
    }

    function createPhaseChart() {
        const ctx = document.getElementById('chart-phase');
        if (!ctx) return;
        charts.phase = new Chart(ctx, {
            type: 'scatter',
            data: {
                datasets: [{
                    label: 'Phase (e_y vs ẏ)',
                    data: [],
                    borderColor: '#00d4ff',
                    backgroundColor: 'rgba(0,212,255,0.3)',
                    pointRadius: 1.5,
                    showLine: true,
                    borderWidth: 1,
                }]
            },
            options: {
                ...baseOpts,
                scales: {
                    x: { ...baseOpts.scales.x, title: { display: true, text: 'Error (m)', color: '#667', font: { size: 9 } } },
                    y: { ...baseOpts.scales.y, title: { display: true, text: 'Velocity (m/s)', color: '#667', font: { size: 9 } } }
                }
            }
        });
    }

    function createMotorChart() {
        const ctx = document.getElementById('chart-motors');
        if (!ctx) return;
        charts.motors = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'M1 (FR)', data: [], borderColor: '#ff4466' },
                    { label: 'M2 (FL)', data: [], borderColor: '#44ff66' },
                    { label: 'M3 (RL)', data: [], borderColor: '#4466ff' },
                    { label: 'M4 (RR)', data: [], borderColor: '#ffaa00' },
                ]
            },
            options: {
                ...baseOpts,
                scales: {
                    ...baseOpts.scales,
                    x: { ...baseOpts.scales.x, title: { display: true, text: 'Time (s)', color: '#667', font: { size: 9 } } },
                    y: { ...baseOpts.scales.y, title: { display: true, text: 'Speed (rad/s)', color: '#667', font: { size: 9 } } }
                }
            }
        });
    }

    /**
     * Push new data point
     */
    function push(time, state, target, controlState, algo) {
        const t = time.toFixed(1);
        const b = dataBuffers;

        b.time.push(t);

        b.xDes.push(target.x);  b.xAct.push(state.x);
        b.yDes.push(target.y);  b.yAct.push(state.y);
        b.zDes.push(target.z);  b.zAct.push(state.z);

        const ex = target.x - state.x, ey = target.y - state.y, ez = target.z - state.z;
        b.errNorm.push(Math.sqrt(ex*ex + ey*ey + ez*ez));

        b.thrust.push(controlState.T);
        b.tauPhi.push(controlState.tau_phi);
        b.tauTheta.push(controlState.tau_theta);
        b.tauPsi.push(controlState.tau_psi);

        b.sx.push(controlState.s_x || 0);
        b.sy.push(controlState.s_y || 0);
        b.sz.push(controlState.s_z || 0);

        b.phaseErrY.push(ey);
        b.phaseVelY.push(state.vy);

        b.m1.push(state.motors ? state.motors[0] : 0);
        b.m2.push(state.motors ? state.motors[1] : 0);
        b.m3.push(state.motors ? state.motors[2] : 0);
        b.m4.push(state.motors ? state.motors[3] : 0);

        // Trim
        for (const key in b) {
            if (b[key].length > MAX_POINTS) b[key].shift();
        }
    }

    function update(algo) {
        const b = dataBuffers;

        // Position chart
        if (charts.position) {
            const c = charts.position;
            c.data.labels = b.time;
            c.data.datasets[0].data = b.xDes;
            c.data.datasets[1].data = b.xAct;
            c.data.datasets[2].data = b.yDes;
            c.data.datasets[3].data = b.yAct;
            c.data.datasets[4].data = b.zDes;
            c.data.datasets[5].data = b.zAct;
            c.update('none');
        }

        // Error / sliding surface chart
        if (charts.error) {
            const c = charts.error;
            c.data.labels = b.time;
            c.data.datasets[0].data = b.errNorm;

            // Controller-specific second dataset
            if (algo === 'SMC' || algo === 'STS') {
                c.data.datasets[1].label = 'Sliding Surface S_z';
                c.data.datasets[1].data = b.sz;
            } else if (algo === 'PID') {
                c.data.datasets[1].label = 'Integral Error';
                const iState = Controllers.getInternalState();
                c.data.datasets[1].data = b.sy.map((_, i) => b.sy[i]); // error Y as proxy
            } else {
                c.data.datasets[1].label = 'Position Error Y';
                c.data.datasets[1].data = b.sy;
            }
            c.update('none');
        }

        // Control effort
        if (charts.control) {
            const c = charts.control;
            c.data.labels = b.time;
            c.data.datasets[0].data = b.thrust;
            c.data.datasets[1].data = b.tauPhi;
            c.data.datasets[2].data = b.tauTheta;
            c.data.datasets[3].data = b.tauPsi;
            c.update('none');
        }

        // Phase portrait
        if (charts.phase) {
            const c = charts.phase;
            const phaseData = b.phaseErrY.map((e, i) => ({ x: e, y: b.phaseVelY[i] }));
            c.data.datasets[0].data = phaseData;
            c.update('none');
        }

        // Motor speeds
        if (charts.motors) {
            const c = charts.motors;
            c.data.labels = b.time;
            c.data.datasets[0].data = b.m1;
            c.data.datasets[1].data = b.m2;
            c.data.datasets[2].data = b.m3;
            c.data.datasets[3].data = b.m4;
            c.update('none');
        }
    }

    function clear() {
        for (const key in dataBuffers) {
            dataBuffers[key] = [];
        }
        for (const key in charts) {
            if (charts[key]) {
                charts[key].data.labels = [];
                charts[key].data.datasets.forEach(d => d.data = []);
                charts[key].update('none');
            }
        }
    }

    /**
     * Export all buffered data as CSV
     */
    function exportCSV() {
        const b = dataBuffers;
        const headers = ['time','x_des','x_act','y_des','y_act','z_des','z_act',
                          'err_norm','thrust','tau_phi','tau_theta','tau_psi',
                          's_x','s_y','s_z','m1','m2','m3','m4'];
        let csv = headers.join(',') + '\n';

        for (let i = 0; i < b.time.length; i++) {
            csv += [
                b.time[i], b.xDes[i]?.toFixed(4), b.xAct[i]?.toFixed(4),
                b.yDes[i]?.toFixed(4), b.yAct[i]?.toFixed(4),
                b.zDes[i]?.toFixed(4), b.zAct[i]?.toFixed(4),
                b.errNorm[i]?.toFixed(4), b.thrust[i]?.toFixed(4),
                b.tauPhi[i]?.toFixed(6), b.tauTheta[i]?.toFixed(6), b.tauPsi[i]?.toFixed(6),
                b.sx[i]?.toFixed(4), b.sy[i]?.toFixed(4), b.sz[i]?.toFixed(4),
                b.m1[i]?.toFixed(1), b.m2[i]?.toFixed(1), b.m3[i]?.toFixed(1), b.m4[i]?.toFixed(1),
            ].join(',') + '\n';
        }
        return csv;
    }

    return { init, push, update, clear, exportCSV };
})();
