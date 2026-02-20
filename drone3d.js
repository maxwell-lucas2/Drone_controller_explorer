/**
 * drone3d.js — Three.js Visualization
 *
 * - Detailed X-frame quadrotor model with spinning propellers
 * - Desired trajectory path (cyan)
 * - Actual flown path trail (orange)
 * - MPC prediction horizon visualization
 * - Ground grid with coordinate axes
 * - Orbit camera controls (manual implementation)
 */

const Drone3D = (() => {

    let scene, camera, renderer;
    let droneGroup, propellers = [];
    let trailPoints = [], trailLine, trailGeometry;
    let desiredPathLine, desiredGeometry;
    let mpcPredLine, mpcPredGeometry;
    let groundGrid;
    let axisLabels = [];

    // Camera orbit state
    let camTheta = Math.PI / 4;
    let camPhi = Math.PI / 5;
    let camRadius = 14;
    let camTarget = new THREE.Vector3(0, 2, 0);
    let isDragging = false;
    let lastMouse = { x: 0, y: 0 };

    const MAX_TRAIL = 800;
    const MAX_DESIRED = 400;

    function init(container) {
        // Scene
        scene = new THREE.Scene();
        scene.fog = new THREE.FogExp2(0x0a0a0f, 0.012);

        // Camera
        const aspect = container.clientWidth / container.clientHeight;
        camera = new THREE.PerspectiveCamera(55, aspect, 0.1, 500);
        updateCameraPosition();

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.1;
        container.appendChild(renderer.domElement);

        // Lighting
        const ambient = new THREE.AmbientLight(0x4488aa, 0.4);
        scene.add(ambient);

        const dirLight = new THREE.DirectionalLight(0xffeedd, 0.8);
        dirLight.position.set(10, 20, 10);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 1024;
        dirLight.shadow.mapSize.height = 1024;
        scene.add(dirLight);

        const pointLight = new THREE.PointLight(0x00d4ff, 0.3, 50);
        pointLight.position.set(0, 10, 0);
        scene.add(pointLight);

        // Ground
        createGround();

        // Drone model
        droneGroup = createDroneModel();
        scene.add(droneGroup);

        // Trail (actual path)
        trailGeometry = new THREE.BufferGeometry();
        const trailMat = new THREE.LineBasicMaterial({
            color: 0xff6622,
            transparent: true,
            opacity: 0.7,
            linewidth: 2
        });
        trailLine = new THREE.Line(trailGeometry, trailMat);
        scene.add(trailLine);

        // Desired trajectory path
        desiredGeometry = new THREE.BufferGeometry();
        const desiredMat = new THREE.LineDashedMaterial({
            color: 0x00d4ff,
            dashSize: 0.3,
            gapSize: 0.15,
            transparent: true,
            opacity: 0.5,
        });
        desiredPathLine = new THREE.Line(desiredGeometry, desiredMat);
        scene.add(desiredPathLine);

        // MPC prediction
        mpcPredGeometry = new THREE.BufferGeometry();
        const mpcMat = new THREE.LineBasicMaterial({
            color: 0xffff00,
            transparent: true,
            opacity: 0.6,
        });
        mpcPredLine = new THREE.Line(mpcPredGeometry, mpcMat);
        scene.add(mpcPredLine);

        // Camera controls
        setupCameraControls(renderer.domElement);

        // Resize
        window.addEventListener('resize', () => {
            const w = container.clientWidth;
            const h = container.clientHeight;
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
            renderer.setSize(w, h);
        });
    }

    function createGround() {
        // Large ground grid
        const gridSize = 80;
        const gridDiv = 80;
        const grid = new THREE.GridHelper(gridSize, gridDiv, 0x1a3040, 0x0d1a24);
        grid.material.transparent = true;
        grid.material.opacity = 0.6;
        scene.add(grid);

        // Coordinate axes
        const axLen = 3;
        const axMats = [0xff3333, 0x33ff33, 0x3333ff]; // R G B → X Y Z
        const dirs = [
            new THREE.Vector3(1,0,0),
            new THREE.Vector3(0,1,0),
            new THREE.Vector3(0,0,1)
        ];
        dirs.forEach((d, i) => {
            const arrow = new THREE.ArrowHelper(d, new THREE.Vector3(0,0.01,0), axLen, axMats[i], 0.15, 0.08);
            scene.add(arrow);
        });

        // Ground plane (subtle)
        const planeGeo = new THREE.PlaneGeometry(80, 80);
        const planeMat = new THREE.MeshStandardMaterial({
            color: 0x0a0f14,
            roughness: 0.9,
            metalness: 0.1,
        });
        const plane = new THREE.Mesh(planeGeo, planeMat);
        plane.rotation.x = -Math.PI / 2;
        plane.position.y = -0.01;
        plane.receiveShadow = true;
        scene.add(plane);
    }

    function createDroneModel() {
        const group = new THREE.Group();

        const carbonMat = new THREE.MeshStandardMaterial({
            color: 0x1a1a1a, roughness: 0.3, metalness: 0.8
        });
        const accentMat = new THREE.MeshStandardMaterial({
            color: 0x00d4ff, roughness: 0.4, metalness: 0.6, emissive: 0x003344, emissiveIntensity: 0.3
        });
        const motorMat = new THREE.MeshStandardMaterial({
            color: 0x333333, roughness: 0.5, metalness: 0.9
        });
        const propMat = new THREE.MeshStandardMaterial({
            color: 0x222222, roughness: 0.6, metalness: 0.3, transparent: true, opacity: 0.85,
            side: THREE.DoubleSide
        });
        const ledGreen = new THREE.MeshStandardMaterial({
            color: 0x00ff44, emissive: 0x00ff44, emissiveIntensity: 1.5
        });
        const ledRed = new THREE.MeshStandardMaterial({
            color: 0xff0022, emissive: 0xff0022, emissiveIntensity: 1.5
        });

        // Central body — flight controller housing
        const bodyGeo = new THREE.BoxGeometry(0.12, 0.025, 0.12);
        const body = new THREE.Mesh(bodyGeo, carbonMat);
        body.castShadow = true;
        group.add(body);

        // Top plate (PCB look)
        const topPlate = new THREE.Mesh(
            new THREE.BoxGeometry(0.10, 0.008, 0.10),
            accentMat
        );
        topPlate.position.y = 0.018;
        group.add(topPlate);

        // Battery underneath
        const battery = new THREE.Mesh(
            new THREE.BoxGeometry(0.06, 0.02, 0.035),
            new THREE.MeshStandardMaterial({ color: 0x222266, roughness: 0.5 })
        );
        battery.position.y = -0.025;
        group.add(battery);

        // X-frame arms
        const armLength = 0.17;
        const armGeo = new THREE.BoxGeometry(armLength * 2.2, 0.012, 0.018);
        const arm1 = new THREE.Mesh(armGeo, carbonMat);
        arm1.rotation.y = Math.PI / 4;
        arm1.castShadow = true;
        group.add(arm1);

        const arm2 = new THREE.Mesh(armGeo, carbonMat);
        arm2.rotation.y = -Math.PI / 4;
        arm2.castShadow = true;
        group.add(arm2);

        // Motors and propellers at 4 positions
        const s2 = Math.SQRT1_2 * armLength;
        const motorPositions = [
            [ s2, 0,  s2],    // Front-Right (CW)
            [-s2, 0,  s2],    // Front-Left (CCW)
            [-s2, 0, -s2],    // Rear-Left (CW)
            [ s2, 0, -s2],    // Rear-Right (CCW)
        ];
        const propDirections = [1, -1, 1, -1]; // CW/CCW

        propellers = [];

        motorPositions.forEach((pos, i) => {
            // Motor housing
            const motor = new THREE.Mesh(
                new THREE.CylinderGeometry(0.018, 0.022, 0.03, 12),
                motorMat
            );
            motor.position.set(pos[0], pos[1] + 0.015, pos[2]);
            motor.castShadow = true;
            group.add(motor);

            // Motor cap
            const cap = new THREE.Mesh(
                new THREE.CylinderGeometry(0.012, 0.018, 0.008, 12),
                accentMat
            );
            cap.position.set(pos[0], pos[1] + 0.034, pos[2]);
            group.add(cap);

            // Propeller disc (represented as a flattened torus for spinning)
            const propGroup = new THREE.Group();
            propGroup.position.set(pos[0], pos[1] + 0.04, pos[2]);

            // Two blades
            for (let b = 0; b < 2; b++) {
                const blade = new THREE.Mesh(
                    new THREE.BoxGeometry(0.14, 0.003, 0.018),
                    propMat
                );
                blade.rotation.y = b * Math.PI;
                // Slight twist
                blade.rotation.z = 0.05 * (b === 0 ? 1 : -1);
                propGroup.add(blade);
            }

            propGroup.userData = { dir: propDirections[i], speed: 0 };
            propellers.push(propGroup);
            group.add(propGroup);

            // LED indicators (front = green, rear = red)
            const isfront = pos[2] > 0;
            const led = new THREE.Mesh(
                new THREE.SphereGeometry(0.006, 8, 8),
                isfront ? ledGreen : ledRed
            );
            led.position.set(pos[0], pos[1] - 0.01, pos[2]);
            group.add(led);

            // LED glow
            const glowLight = new THREE.PointLight(
                isfront ? 0x00ff44 : 0xff0022,
                0.15, 0.5
            );
            glowLight.position.copy(led.position);
            group.add(glowLight);
        });

        // Landing gear (small legs)
        const legGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.04, 6);
        const legMat = new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.7 });
        [[-0.06, -0.035, 0.06], [0.06, -0.035, 0.06], [-0.06, -0.035, -0.06], [0.06, -0.035, -0.06]].forEach(p => {
            const leg = new THREE.Mesh(legGeo, legMat);
            leg.position.set(p[0], p[1], p[2]);
            group.add(leg);
        });

        group.scale.set(3, 3, 3); // Scale up for visibility
        return group;
    }

    function updateDrone(state) {
        if (!droneGroup) return;

        droneGroup.position.set(state.x, state.y, state.z);
        // Three.js uses intrinsic YXZ for Euler... we need ZYX
        droneGroup.rotation.set(state.theta, state.psi, state.phi, 'YXZ');

        // Spin propellers based on motor speeds
        propellers.forEach((p, i) => {
            const speed = state.motors ? state.motors[i] : 800;
            // Visual spin speed (scaled for frame rate visibility)
            p.rotation.y += p.userData.dir * speed * 0.003;
        });

        // Update trail
        trailPoints.push(new THREE.Vector3(state.x, state.y, state.z));
        if (trailPoints.length > MAX_TRAIL) trailPoints.shift();
        trailGeometry.setFromPoints(trailPoints);

        // Camera follow (smooth)
        camTarget.lerp(new THREE.Vector3(state.x, state.y, state.z), 0.03);
        updateCameraPosition();
    }

    function setDesiredPath(points3D) {
        const vecs = points3D.map(p => new THREE.Vector3(p[0], p[1], p[2]));
        desiredGeometry.setFromPoints(vecs);
        desiredPathLine.computeLineDistances();
    }

    function setMPCPrediction(predictions) {
        if (!predictions || predictions.length === 0) {
            mpcPredGeometry.setFromPoints([]);
            return;
        }
        const vecs = predictions.map(p => new THREE.Vector3(p.x, p.y, p.z));
        mpcPredGeometry.setFromPoints(vecs);
    }

    function clearTrail() {
        trailPoints = [];
        trailGeometry.setFromPoints([]);
    }

    function render() {
        if (renderer && scene && camera) {
            renderer.render(scene, camera);
        }
    }

    // --- Camera Controls ---
    function updateCameraPosition() {
        if (!camera) return;
        camera.position.set(
            camTarget.x + camRadius * Math.sin(camTheta) * Math.cos(camPhi),
            camTarget.y + camRadius * Math.sin(camPhi),
            camTarget.z + camRadius * Math.cos(camTheta) * Math.cos(camPhi)
        );
        camera.lookAt(camTarget);
    }

    function setupCameraControls(element) {
        element.addEventListener('mousedown', (e) => {
            if (e.button === 0 || e.button === 2) {
                isDragging = true;
                lastMouse = { x: e.clientX, y: e.clientY };
            }
        });
        element.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            const dx = e.clientX - lastMouse.x;
            const dy = e.clientY - lastMouse.y;
            camTheta -= dx * 0.005;
            camPhi = Math.max(0.05, Math.min(Math.PI / 2 - 0.05, camPhi + dy * 0.005));
            lastMouse = { x: e.clientX, y: e.clientY };
            updateCameraPosition();
        });
        element.addEventListener('mouseup', () => { isDragging = false; });
        element.addEventListener('mouseleave', () => { isDragging = false; });
        element.addEventListener('wheel', (e) => {
            camRadius = Math.max(3, Math.min(50, camRadius + e.deltaY * 0.01));
            updateCameraPosition();
        });
        element.addEventListener('contextmenu', (e) => e.preventDefault());
    }

    return { init, updateDrone, setDesiredPath, setMPCPrediction, clearTrail, render };
})();
