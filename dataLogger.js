/**
 * dataLogger.js — Simulation Data Recording & Export
 *
 * Records full state + control + target at configurable rate.
 * Exports to CSV for MATLAB/Python analysis.
 */

const DataLogger = (() => {

    let recording = false;
    let records = [];
    let startTime = 0;

    function startRecording(simTime) {
        recording = true;
        records = [];
        startTime = simTime;
    }

    function stopRecording() {
        recording = false;
    }

    function isRecording() { return recording; }

    function log(simTime, state, target, controlState, algo) {
        if (!recording) return;
        records.push({
            t: (simTime - startTime).toFixed(4),
            // State
            x: state.x.toFixed(6), y: state.y.toFixed(6), z: state.z.toFixed(6),
            vx: state.vx.toFixed(6), vy: state.vy.toFixed(6), vz: state.vz.toFixed(6),
            phi: state.phi.toFixed(6), theta: state.theta.toFixed(6), psi: state.psi.toFixed(6),
            p: state.p.toFixed(6), q: state.q.toFixed(6), r: state.r.toFixed(6),
            // Target
            x_ref: target.x.toFixed(6), y_ref: target.y.toFixed(6), z_ref: target.z.toFixed(6),
            // Control
            T: controlState.T.toFixed(6),
            tau_phi: controlState.tau_phi.toFixed(6),
            tau_theta: controlState.tau_theta.toFixed(6),
            tau_psi: controlState.tau_psi.toFixed(6),
            // Sliding surfaces
            s_x: (controlState.s_x || 0).toFixed(6),
            s_y: (controlState.s_y || 0).toFixed(6),
            s_z: (controlState.s_z || 0).toFixed(6),
            // Motors
            m1: state.motors ? state.motors[0].toFixed(2) : '0',
            m2: state.motors ? state.motors[1].toFixed(2) : '0',
            m3: state.motors ? state.motors[2].toFixed(2) : '0',
            m4: state.motors ? state.motors[3].toFixed(2) : '0',
            // Meta
            algo: algo
        });
    }

    function exportCSV() {
        if (records.length === 0) return null;
        const headers = Object.keys(records[0]);
        let csv = headers.join(',') + '\n';
        records.forEach(r => {
            csv += headers.map(h => r[h]).join(',') + '\n';
        });
        return csv;
    }

    function downloadCSV(filename) {
        const csv = exportCSV();
        if (!csv) { alert('No data recorded. Press Record first.'); return; }

        const blob = new Blob([csv], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename || `quadrotor_data_${Date.now()}.csv`;
        a.click();
        URL.revokeObjectURL(url);
    }

    function getRecordCount() { return records.length; }

    return { startRecording, stopRecording, isRecording, log, exportCSV, downloadCSV, getRecordCount };
})();
