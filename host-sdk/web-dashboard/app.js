/**
 * Stepper Motion Core — High-Detail 3D Scanner Dashboard
 * Integrates Three.js 3D Viewport, JSON-RPC 2.0 Client & Scan Orchestrator
 */

// State variables
let ws = null;
let isConnected = false;
let isSimulation = false;
let currentStepSize = 5.0; // degrees or mm
let currentPos = { A: 0.0, B: 0.0, Z: 0.0 };
let systemState = "idle";
let estopActive = false;
let reqIdCounter = 0;
let pendingRequests = new Map();
let currentScanPoses = [];
let scanIndex = 0;
let isScanning = false;

// Three.js scene objects
let scene, camera, renderer, controls;
let turntableMesh, targetMesh, cameraHeadGroup, frustumLines, posePointsCloud;

// ---------------------------------------------------------------------------
// Three.js 3D Viewport Setup
// ---------------------------------------------------------------------------

function initThreeScene() {
  const container = document.getElementById("three-canvas");
  const width = container.clientWidth || 800;
  const height = container.clientHeight || 500;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1120);

  camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 100);
  camera.position.set(0.45, 0.35, 0.45);
  camera.lookAt(0, 0.05, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  // Orbit controls
  if (typeof THREE.OrbitControls !== "undefined") {
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 0.05, 0);
  }

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0x06b6d4, 1.2);
  dirLight.position.set(0.5, 1.0, 0.5);
  scene.add(dirLight);

  const dirLight2 = new THREE.DirectionalLight(0x3b82f6, 0.8);
  dirLight2.position.set(-0.5, -0.2, -0.5);
  scene.add(dirLight2);

  // Grid & Origin
  const gridHelper = new THREE.GridHelper(0.5, 20, 0x06b6d4, 0x1e293b);
  scene.add(gridHelper);

  const axesHelper = new THREE.AxesHelper(0.1);
  scene.add(axesHelper);

  // Turntable circular plate
  const plateGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.005, 32);
  const plateMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.4, metalness: 0.8 });
  turntableMesh = new THREE.Mesh(plateGeo, plateMat);
  turntableMesh.position.y = 0.0025;
  scene.add(turntableMesh);

  // Target object to scan (mechanical gear / calibration cube at origin)
  const targetGeo = new THREE.TorusKnotGeometry(0.025, 0.008, 64, 16);
  const targetMat = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.3, metalness: 0.2 });
  targetMesh = new THREE.Mesh(targetGeo, targetMat);
  targetMesh.position.y = 0.05;
  scene.add(targetMesh);

  // Camera Head & Optical Viewing Frustum Pyramid
  cameraHeadGroup = new THREE.Group();
  
  // Camera body box
  const camBodyGeo = new THREE.BoxGeometry(0.02, 0.015, 0.025);
  const camBodyMat = new THREE.MeshStandardMaterial({ color: 0xef4444 });
  const camBody = new THREE.Mesh(camBodyGeo, camBodyMat);
  cameraHeadGroup.add(camBody);

  // Frustum pyramid wireframe
  const frustumGeo = new THREE.BufferGeometry();
  const f_pts = [
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(-0.03, -0.02, 0.08),
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(0.03, -0.02, 0.08),
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(0.03, 0.02, 0.08),
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(-0.03, 0.02, 0.08),
    // Frustum rectangle
    new THREE.Vector3(-0.03, -0.02, 0.08), new THREE.Vector3(0.03, -0.02, 0.08),
    new THREE.Vector3(0.03, -0.02, 0.08), new THREE.Vector3(0.03, 0.02, 0.08),
    new THREE.Vector3(0.03, 0.02, 0.08), new THREE.Vector3(-0.03, 0.02, 0.08),
    new THREE.Vector3(-0.03, 0.02, 0.08), new THREE.Vector3(-0.03, -0.02, 0.08)
  ];
  frustumGeo.setFromPoints(f_pts);
  const frustumMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 });
  frustumLines = new THREE.LineSegments(frustumGeo, frustumMat);
  cameraHeadGroup.add(frustumLines);

  scene.add(cameraHeadGroup);

  // Initial Fibonacci pose points visualization
  updateFibonacciPreview(56);
  update3DCameraPosition(0, 0, 0);

  // Animation Loop
  function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
  }
  animate();

  window.addEventListener("resize", () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });
}

function updateFibonacciPreview(n_poses) {
  if (posePointsCloud) {
    scene.remove(posePointsCloud);
    posePointsCloud.geometry.dispose();
  }

  const radius = 0.20; // 200mm scan radius
  const goldenRatio = (1.0 + Math.sqrt(5.0)) / 2.0;
  const goldenAngle = 2.0 * Math.PI * (1.0 - 1.0 / goldenRatio);
  const yMin = Math.sin((5.0 * Math.PI) / 180.0);
  const yMax = Math.sin((85.0 * Math.PI) / 180.0);

  const vertices = [];
  const colors = [];
  currentScanPoses = [];

  for (let i = 0; i < n_poses; ++i) {
    const fraction = (i + 0.5) / n_poses;
    const y = yMin + fraction * (yMax - yMin);
    const elevation = Math.asin(y);
    const azimuth = (i * goldenAngle) % (2.0 * Math.PI);

    const cx = radius * Math.cos(elevation) * Math.cos(azimuth);
    const cz = radius * Math.cos(elevation) * Math.sin(azimuth);
    const cy = radius * Math.sin(elevation);

    vertices.push(cx, cy + 0.05, cz);

    // Color gradient from green to cyan
    colors.push(0.06, 0.71, 0.83);

    currentScanPoses.push({
      azimuth_deg: (azimuth * 180.0) / Math.PI,
      elevation_deg: (elevation * 180.0) / Math.PI,
      focus_z_mm: 0.0
    });
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geom.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  const mat = new THREE.PointsMaterial({ size: 0.008, vertexColors: true });
  posePointsCloud = new THREE.Points(geom, mat);
  scene.add(posePointsCloud);
}

function update3DCameraPosition(azimuth_deg, elevation_deg, focus_z_mm) {
  const radius = 0.20 + (focus_z_mm || 0.0) / 1000.0;
  const theta = (azimuth_deg * Math.PI) / 180.0;
  const phi = (elevation_deg * Math.PI) / 180.0;

  const x = radius * Math.cos(phi) * Math.cos(theta);
  const z = radius * Math.cos(phi) * Math.sin(theta);
  const y = radius * Math.sin(phi) + 0.05; // offset above turntable

  cameraHeadGroup.position.set(x, y, z);
  cameraHeadGroup.lookAt(0, 0.05, 0);

  // Rotate turntable around Y
  if (turntableMesh) {
    turntableMesh.rotation.y = -theta;
  }
}

// ---------------------------------------------------------------------------
// JSON-RPC 2.0 Communication & UI Handlers
// ---------------------------------------------------------------------------

function logMessage(msg, type = "") {
  const consoleEl = document.getElementById("log-console");
  const entry = document.createElement("div");
  entry.className = `log-entry ${type}`;
  const timestamp = new Date().toLocaleTimeString();
  entry.innerText = `[${timestamp}] ${typeof msg === "object" ? JSON.stringify(msg) : msg}`;
  consoleEl.appendChild(entry);
  consoleEl.scrollTop = consoleEl.scrollHeight;
}

function sendRpc(method, params = {}) {
  reqIdCounter++;
  const payload = {
    jsonrpc: "2.0",
    id: reqIdCounter,
    method: method,
    params: params
  };

  logMessage(`-> ${method}: ${JSON.stringify(params)}`);

  if (isSimulation) {
    handleSimulatedRpc(payload);
    return;
  }

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(payload));
  } else {
    logMessage("Cannot send: not connected", "error");
  }
}

function handleIncomingMessage(text) {
  let msg;
  try {
    msg = JSON.parse(text);
  } catch (e) {
    logMessage(`Malformed JSON: ${text}`, "error");
    return;
  }

  // Telemetry Event
  if (msg.event) {
    logMessage(`[EVENT] ${msg.event}: ${JSON.stringify(msg.data)}`, "event");
    if (msg.event === "motion.status") {
      updateTelemetryUI(msg.data);
    } else if (msg.event === "system.estop") {
      setEstopLocked(true, msg.data.reason);
    } else if (msg.event === "system.reset") {
      setEstopLocked(false);
    }
    return;
  }

  // Response
  if (msg.id) {
    if (msg.error) {
      logMessage(`<- Error ${msg.error.code}: ${msg.error.message}`, "error");
    } else {
      logMessage(`<- Result: ${JSON.stringify(msg.result)}`);
      if (msg.result.state) {
        setSystemState(msg.result.state);
      }
      if (msg.result.pos) {
        updateTelemetryUI(msg.result);
      }
    }
  }
}

function updateTelemetryUI(data) {
  if (data.pos) {
    currentPos.A = data.pos.A || 0.0;
    currentPos.B = data.pos.B || 0.0;
    currentPos.Z = data.pos.Z || 0.0;

    document.getElementById("pos-a").innerText = currentPos.A.toFixed(2) + "°";
    document.getElementById("pos-b").innerText = currentPos.B.toFixed(2) + "°";
    document.getElementById("pos-z").innerText = currentPos.Z.toFixed(2) + "mm";

    update3DCameraPosition(currentPos.A, currentPos.B, currentPos.Z);
  }

  if (data.state) {
    setSystemState(data.state);
  }
  if (data.estop_active !== undefined) {
    setEstopLocked(data.estop_active);
  }
}

function setSystemState(state) {
  systemState = state;
  const badge = document.getElementById("state-badge");
  badge.innerText = state.toUpperCase();
  if (state === "estop_locked") {
    badge.style.backgroundColor = "var(--accent-red)";
    badge.style.color = "white";
  } else if (state === "moving" || state === "homing") {
    badge.style.backgroundColor = "var(--accent-yellow)";
    badge.style.color = "black";
  } else {
    badge.style.backgroundColor = "var(--border-color)";
    badge.style.color = "var(--text-secondary)";
  }
}

function setEstopLocked(locked, reason = "") {
  estopActive = locked;
  const overlay = document.getElementById("estop-alert");
  if (locked) {
    setSystemState("estop_locked");
    overlay.style.display = "block";
    overlay.innerText = `EMERGENCY STOP ACTIVE — ${reason || "Motion locked. Click RESET to restore."}`;
  } else {
    setSystemState("idle");
    overlay.style.display = "none";
  }
}

// ---------------------------------------------------------------------------
// Simulation Mode Fallback
// ---------------------------------------------------------------------------

function handleSimulatedRpc(req) {
  const method = req.method;
  const params = req.params;

  if (method === "system.estop") {
    setEstopLocked(true, params.reason || "Simulated E-STOP");
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { estop: true, state: "estop_locked" } }));
    return;
  }

  if (estopActive && method !== "system.reset" && method !== "system.info" && method !== "axis.get_status") {
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, error: { code: -32001, message: "E-STOP active: locked until reset" } }));
    return;
  }

  if (method === "system.reset") {
    setEstopLocked(false);
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { reset: true, state: "idle" } }));
  } else if (method === "system.info") {
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { firmware: "stepper-motion-core", version: "0.2.0", mcu: "ESP32-S3 Simulator" } }));
  } else if (method === "axis.jog") {
    const ax = params.axis;
    const dist = params.distance;
    currentPos[ax] = (currentPos[ax] || 0) + dist;
    updateTelemetryUI({ pos: currentPos, state: "idle" });
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { jogging: true, axis: ax } }));
  } else if (method === "axis.home") {
    currentPos = { A: 0.0, B: 0.0, Z: 0.0 };
    updateTelemetryUI({ pos: currentPos, state: "idle" });
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { homing: true, axes: ["A", "B", "Z"] } }));
  } else if (method === "axis.move") {
    if (params.axes) {
      for (const [k, v] of Object.entries(params.axes)) {
        currentPos[k] = parseFloat(v);
      }
    }
    updateTelemetryUI({ pos: currentPos, state: "idle" });
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { queued: true } }));
  } else if (method === "trigger.capture") {
    handleIncomingMessage(JSON.stringify({ jsonrpc: "2.0", id: req.id, result: { triggered: true, status: "ack" } }));
  }
}

// ---------------------------------------------------------------------------
// UI Event Bindings
// ---------------------------------------------------------------------------

document.addEventListener("DOMContentLoaded", () => {
  initThreeScene();

  // Connection
  const connectBtn = document.getElementById("connect-btn");
  connectBtn.addEventListener("click", () => {
    const url = document.getElementById("target-url").value.trim();
    if (url.toLowerCase() === "mock" || url === "") {
      isSimulation = true;
      isConnected = true;
      document.getElementById("status-dot").className = "status-dot connected";
      connectBtn.innerText = "Disconnect";
      logMessage("[INFO] Connected in Mock/Simulation Mode");
      return;
    }

    if (isConnected) {
      if (ws) ws.close();
      isConnected = false;
      document.getElementById("status-dot").className = "status-dot";
      connectBtn.innerText = "Connect";
      logMessage("[INFO] Disconnected.");
      return;
    }

    try {
      ws = new WebSocket(url);
      ws.onopen = () => {
        isConnected = true;
        isSimulation = false;
        document.getElementById("status-dot").className = "status-dot connected";
        connectBtn.innerText = "Disconnect";
        logMessage(`[INFO] Connected to ${url}`);
        sendRpc("system.info");
      };
      ws.onmessage = (ev) => handleIncomingMessage(ev.data);
      ws.onclose = () => {
        isConnected = false;
        document.getElementById("status-dot").className = "status-dot";
        connectBtn.innerText = "Connect";
        logMessage("[INFO] WebSocket closed.");
      };
      ws.onerror = (err) => logMessage(`WebSocket error: ${err}`, "error");
    } catch (e) {
      logMessage(`Connection failed: ${e}`, "error");
    }
  });

  // Step size buttons
  document.querySelectorAll(".step-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".step-btn").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      currentStepSize = parseFloat(btn.getAttribute("data-step"));
    });
  });

  // Jog buttons
  document.querySelectorAll("[data-jog]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const axis = btn.getAttribute("data-axis");
      const dir = parseFloat(btn.getAttribute("data-jog"));
      const dist = dir * currentStepSize;
      sendRpc("axis.jog", { axis: axis, distance: dist });
    });
  });

  // Home buttons
  document.getElementById("btn-home-all").addEventListener("click", () => sendRpc("axis.home"));

  // E-STOP & Reset
  document.getElementById("btn-estop").addEventListener("click", () => sendRpc("system.estop", { reason: "Web Dashboard E-STOP button" }));
  document.getElementById("btn-reset").addEventListener("click", () => sendRpc("system.reset"));

  // Scan controls
  const startScanBtn = document.getElementById("btn-start-scan");
  const abortScanBtn = document.getElementById("btn-abort-scan");

  startScanBtn.addEventListener("click", async () => {
    if (isScanning) return;
    const poseCount = parseInt(document.getElementById("scan-poses-input").value) || 56;
    updateFibonacciPreview(poseCount);
    isScanning = true;
    startScanBtn.disabled = true;
    abortScanBtn.disabled = false;
    logMessage(`[SCAN] Starting automated 3D scan of ${poseCount} poses...`);

    for (let i = 0; i < currentScanPoses.length; ++i) {
      if (!isScanning) break;
      const p = currentScanPoses[i];
      document.getElementById("scan-progress-text").innerText = `Pose ${i + 1} / ${currentScanPoses.length}`;
      document.getElementById("scan-progress-bar").style.width = `${((i + 1) / currentScanPoses.length) * 100}%`;

      // Move to pose
      sendRpc("axis.move", { axes: { A: p.azimuth_deg, B: p.elevation_deg, Z: p.focus_z_mm } });
      await new Promise((r) => setTimeout(r, 150));

      // Trigger camera
      sendRpc("trigger.capture", { settle_ms: 100 });
      await new Promise((r) => setTimeout(r, 100));
    }

    isScanning = false;
    startScanBtn.disabled = false;
    abortScanBtn.disabled = true;
    logMessage("[SCAN] 3D Scan completed successfully!");
  });

  abortScanBtn.addEventListener("click", () => {
    isScanning = false;
    startScanBtn.disabled = false;
    abortScanBtn.disabled = true;
    logMessage("[SCAN] Scan aborted by user.");
  });

  // Clear log
  document.getElementById("btn-clear-log").addEventListener("click", () => {
    document.getElementById("log-console").innerHTML = "";
  });
});
