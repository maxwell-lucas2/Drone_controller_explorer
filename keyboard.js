/**
 * keyboard.js — Input Management
 *
 * Tracks pressed keys for manual drone piloting.
 * WASD / Arrow Keys = XZ movement
 * Space / Shift     = Altitude up/down
 * Q / E             = Yaw rotation
 */

const Keyboard = (() => {
    const keys = {};
    let enabled = false;

    function init() {
        document.addEventListener('keydown', (e) => {
            // Don't capture when typing in inputs
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;
            keys[e.code] = true;
            // Prevent page scroll on space
            if (e.code === 'Space') e.preventDefault();
        });

        document.addEventListener('keyup', (e) => {
            keys[e.code] = false;
        });

        // Reset on blur
        window.addEventListener('blur', () => {
            for (const k in keys) keys[k] = false;
        });
    }

    function getKeys() { return keys; }
    function isEnabled() { return enabled; }
    function setEnabled(v) { enabled = v; }

    function isAnyPressed() {
        for (const k in keys) {
            if (keys[k]) return true;
        }
        return false;
    }

    return { init, getKeys, isEnabled, setEnabled, isAnyPressed };
})();
