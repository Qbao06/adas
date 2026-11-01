/**
 * ADAS PRO // Cockpit Dashboard Application Logic
 * Noir Technical Automotive Interface with In-Memory Simulator & Hardware Control.
 */

(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const isStaticPreview = window.location.protocol === "file:";

  // Hardware readiness classification definitions
  const HARDWARE_COMPONENTS = [
    { id: "camera", name: "Camera", keys: ["camera", "video", "v4l2", "webcam"], tag: "CAMERA" },
    { id: "lidar", name: "LiDAR", keys: ["lidar", "driver", "serial", "uart", "ttyusb"], tag: "LIDAR" },
    { id: "ai", name: "AI Engine", keys: ["model", "engine", "torch", "pytorch", "cuda", "yolo", "weights"], tag: "AI ENGINE" },
    { id: "calibration", name: "Calibration", keys: ["calibration", "extrinsic", "intrinsic"], tag: "CALIBRATION" },
    { id: "config", name: "Configuration", keys: ["config", "ngưỡng", "warning", "mode", "distance", "safety"], tag: "CẤU HÌNH" },
  ];

  let currentTab = "cockpit";
  let activeCamera = "front";
  let isDemo = false;
  let isRechecking = false;
  let lastStatus = null;
  let animFrameId = null;
  let roadOffset = 0;
  // The displayed image is only replaced after a complete new JPEG arrives.
  // Assigning a failing request directly here cleared the canvas and looked
  // like camera flicker whenever a USB camera briefly reset.
  let liveFrameImage = new Image();
  let liveFrameLoading = false;
  const liveCameraImages = {};
  const liveCameraLoading = {};
  let moduleConfig = null;
  let radarConfig = null;
  let radarConfigDirty = false;
  let radarDevices = null;
  let radarStatus = null;
  let lastRadarSaveResult = null;
  let hardwareConfig = null;
  let hardwareDevices = null;
  let hardwareSelectedSlot = "front";
  const HW_CAMERA_SLOTS = [
    { id: "front", label: "TRƯỚC", full: "Camera trước", required: true },
    { id: "left_1", label: "TRÁI TRƯỚC", full: "Camera trái trước", required: false },
    { id: "left_2", label: "TRÁI SAU", full: "Camera trái sau", required: false },
    { id: "right_1", label: "PHẢI TRƯỚC", full: "Camera phải trước", required: false },
    { id: "right_2", label: "PHẢI SAU", full: "Camera phải sau", required: false },
    { id: "rear", label: "SAU", full: "Camera sau", required: false },
  ];
  const HW_PREVIEW = {
    camera: {
      enabled: true, width: 1280, height: 720, fps: 30,
      slots: {
        front: { enabled: true, device: "0", label: "Camera trước", required: true },
        left_1: { enabled: true, device: "1", label: "Camera trái trước", required: false },
        left_2: { enabled: false, device: "", label: "Camera trái sau", required: false },
        right_1: { enabled: true, device: "2", label: "Camera phải trước", required: false },
        right_2: { enabled: false, device: "", label: "Camera phải sau", required: false },
        rear: { enabled: true, device: "3", label: "Camera sau", required: false },
      },
    },
    lidar: { enabled: true, driver: "mock", source: "mock", fps: 10, driver_module: "" },
    radar_gateway: { enabled: true, device: "COM3", baudrate: 115200 },
  };
  const HW_PREVIEW_DEVICES = {
    cameras: ["/dev/video0", "/dev/video1"],
    camera_indices: ["0", "1", "2", "3"],
    serial: { by_id: [], ttyacm: ["/dev/ttyACM0"], ttyusb: [], com: ["COM3", "COM4"] },
    platform: "win32",
  };
  const previewState = {
    scenario: "normal", active_camera: "front", gear: "D", turn_signal: "none",
    cam_lost: false, lidar_lost: false, ai_lost: false, paused: false, frame_id: 0,
  };

  function getStaticPreviewData() {
    if (!previewState.paused) previewState.frame_id += 1;
    const t = Date.now() / 1000;
    let distance = 16 + 2 * Math.sin(t * 0.8);
    let level = distance > 15 ? "safe" : "caution";
    let message = `XE PHÍA TRƯỚC · ${distance.toFixed(1)}m`;
    let detections = [{ id: 101, label: "car", distance_m: distance, confidence: .94, bbox: [540, 310, 200, 160] }];
    if (previewState.scenario === "pedestrian") {
      distance = 6.8 + 1.2 * Math.cos(t * 1.2);
      level = distance > 5 ? "caution" : "danger";
      message = `CẢNH BÁO: NGƯỜI ĐI BỘ · ${distance.toFixed(1)}m`;
      detections = [{ id: 201, label: "person", distance_m: distance, confidence: .91, bbox: [580, 320, 75, 175] }];
    } else if (previewState.scenario === "danger") {
      distance = 3.2 + .8 * Math.sin(t * 2);
      level = "danger";
      message = `NGUY HIỂM: NGUY CƠ VA CHẠM · ${distance.toFixed(1)}m`;
      detections = [{ id: 301, label: "car", distance_m: distance, confidence: .97, bbox: [460, 240, 360, 280] }];
    }
    if (previewState.ai_lost || previewState.cam_lost) detections = [];
    const lidar_points = previewState.lidar_lost ? [] : Array.from({ length: 72 }, (_, i) => {
      const angle = i / 72 * Math.PI * 2;
      const radius = 14 + 3 * Math.sin(angle * 4 + t);
      return [radius * Math.cos(angle), radius * Math.sin(angle)];
    });
    return { ready: true, demo: true, errors: [], snapshot: {
      active_camera: previewState.active_camera,
      warning: { level, distance_m: distance, message },
      vehicle: { gear: previewState.gear, turn_signal: previewState.turn_signal },
      detections, lidar_points, fps: 30, latency_ms: 4.2, frame_id: previewState.frame_id,
      side_radar: { link_ok: true, left: false, right: false, left_m: null, right_m: null },
      health: {
        camera: previewState.cam_lost ? "error" : "ok",
        lidar: previewState.lidar_lost ? "error" : "ok",
        ai: previewState.ai_lost ? "error" : "ok", fusion: "ok", calibration: "ok",
        system: (previewState.cam_lost || previewState.lidar_lost || previewState.ai_lost) ? "warning" : "ok",
      },
    }};
  }

  function applyStaticPreviewAction(action) {
    if (["normal", "pedestrian", "danger"].includes(action)) previewState.scenario = action;
    if (action === "normal" || action === "reset") Object.assign(previewState, {
      scenario: "normal", active_camera: "front", gear: "D", turn_signal: "none",
      cam_lost: false, lidar_lost: false, ai_lost: false, paused: false,
    });
    if (action === "lost_cam") previewState.cam_lost = !previewState.cam_lost;
    if (action === "lost_lidar") previewState.lidar_lost = !previewState.lidar_lost;
    if (action === "lost_ai") previewState.ai_lost = !previewState.ai_lost;
    if (action === "turn_signal") previewState.turn_signal = ({ none: "left", left: "right", right: "none" })[previewState.turn_signal];
    if (action === "reverse") {
      previewState.gear = previewState.gear === "R" ? "D" : "R";
      previewState.active_camera = previewState.gear === "R" ? "rear" : "front";
    }
    if (action === "pause_resume") previewState.paused = !previewState.paused;
  }

  // --------------------------------------------------------------------------
  // Clock & Toast
  // --------------------------------------------------------------------------
  function updateClock() {
    const el = $("clock");
    if (el) {
      const now = new Date();
      el.textContent = now.toTimeString().split(" ")[0];
    }
  }
  setInterval(updateClock, 1000);
  updateClock();

  let toastTimer = null;
  function showToast(msg, duration = 3000) {
    const t = $("toast");
    if (!t) return;
    t.textContent = msg;
    t.classList.add("show");
    t.setAttribute("aria-hidden", "false");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
      t.classList.remove("show");
      t.setAttribute("aria-hidden", "true");
    }, duration);
  }

  // --------------------------------------------------------------------------
  // Navigation Tabs
  // --------------------------------------------------------------------------
  function switchTab(tabId) {
    currentTab = tabId;
    document.querySelectorAll(".nav-tab").forEach((b) => {
      b.classList.toggle("active", b.dataset.tab === tabId);
    });
    document.querySelectorAll(".cockpit-tab-view").forEach((v) => {
      v.classList.remove("active");
    });
    const target = $("view" + tabId.charAt(0).toUpperCase() + tabId.slice(1));
    if (target) target.classList.add("active");
  }

  // --------------------------------------------------------------------------
  // Canvas Rendering (Primary Camera & Multi-Cam)
  // --------------------------------------------------------------------------
  function drawPerspectiveRoad(ctx, w, h, offset, isRear = false) {
    // Sky gradient
    const sky = ctx.createLinearGradient(0, 0, 0, h * 0.45);
    sky.addColorStop(0, "#05070a");
    sky.addColorStop(1, "#101318");
    ctx.fillStyle = sky;
    ctx.fillRect(0, 0, w, h * 0.45);

    // Ground / Asphalt gradient
    const ground = ctx.createLinearGradient(0, h * 0.45, 0, h);
    ground.addColorStop(0, "#16161b");
    ground.addColorStop(1, "#070709");
    ctx.fillStyle = ground;
    ctx.fillRect(0, h * 0.45, w, h * 0.55);

    // Horizon line
    ctx.strokeStyle = "rgba(255, 255, 255, 0.1)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h * 0.45);
    ctx.lineTo(w, h * 0.45);
    ctx.stroke();

    // Vanishing point
    const vx = w / 2;
    const vy = h * 0.45;

    // Road borders
    ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(vx - 20, vy);
    ctx.lineTo(w * 0.12, h);
    ctx.moveTo(vx + 20, vy);
    ctx.lineTo(w * 0.88, h);
    ctx.stroke();

    // Animated dashed center lane
    ctx.strokeStyle = "rgba(255, 255, 255, 0.6)";
    ctx.lineWidth = 3;
    const numDashes = 7;
    const speedMultiplier = isRear ? -1 : 1;

    for (let i = 0; i < numDashes; i++) {
      const prog = ((i + (offset * speedMultiplier * 0.05)) % numDashes) / numDashes;
      if (prog < 0.02) continue;
      const y1 = vy + (h - vy) * Math.pow(prog, 2);
      const y2 = vy + (h - vy) * Math.pow(Math.min(prog + 0.07, 0.99), 2);
      const halfW = (w * 0.38) * prog;

      ctx.beginPath();
      ctx.moveTo(vx, y1);
      ctx.lineTo(vx, y2);
      ctx.stroke();
    }
  }

  function drawDetections(ctx, detections, w, h) {
    if (!detections || detections.length === 0) return;

    detections.forEach((det) => {
      // bbox: [x, y, bw, bh] in 1280x720 coords
      const scaleX = w / 1280;
      const scaleY = h / 720;
      const [bx, by, bw, bh] = det.bbox;
      const x = bx * scaleX;
      const y = by * scaleY;
      const width = bw * scaleX;
      const height = bh * scaleY;

      const dist = det.distance_m || 20;
      let color = "#22c55e"; // Safe
      if (dist <= 5.0) color = "#ef4444"; // Danger
      else if (dist <= 12.0) color = "#f59e0b"; // Caution

      // Bounding box with corner accents
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.strokeRect(x, y, width, height);

      // Target reticle
      ctx.fillStyle = color;
      const cornerLen = 10;
      ctx.fillRect(x - 2, y - 2, cornerLen, 3);
      ctx.fillRect(x - 2, y - 2, 3, cornerLen);
      ctx.fillRect(x + width - cornerLen + 2, y - 2, cornerLen, 3);
      ctx.fillRect(x + width - 1, y - 2, 3, cornerLen);

      // Label background
      const labelText = `${(det.label || "CAR").toUpperCase()} · ${dist.toFixed(1)}m`;
      ctx.font = "bold 11px monospace";
      const textWidth = ctx.measureText(labelText).width;

      ctx.fillStyle = "rgba(0, 0, 0, 0.85)";
      ctx.fillRect(x, y - 18, textWidth + 8, 18);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;
      ctx.strokeRect(x, y - 18, textWidth + 8, 18);

      // Label text
      ctx.fillStyle = color;
      ctx.fillText(labelText, x + 4, y - 5);
    });
  }

  function renderPrimaryViewport(snapshot) {
    const canvas = $("primaryCanvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    if (!isDemo) {
      if (liveFrameImage.complete && liveFrameImage.naturalWidth > 0) {
        ctx.drawImage(liveFrameImage, 0, 0, w, h);
        if (snapshot.detections && snapshot.health?.camera !== "error") {
          drawDetections(ctx, snapshot.detections, w, h);
        }
      } else {
        ctx.fillStyle = "#050507";
        ctx.fillRect(0, 0, w, h);
      }
      return;
    }

    const isRear = snapshot.active_camera === "rear" || snapshot.vehicle?.gear === "R";

    // Draw synthetic camera frame
    drawPerspectiveRoad(ctx, w, h, roadOffset, isRear);

    // Draw AI bounding boxes
    if (snapshot.detections && snapshot.health?.camera !== "error") {
      drawDetections(ctx, snapshot.detections, w, h);
    }
  }

  function renderMultiCameras(snapshot) {
    const cams = ["Front", "Left", "Right", "Rear"];
    cams.forEach((c) => {
      const canvas = $("multiCam" + c);
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const w = canvas.width;
      const h = canvas.height;
      if (isDemo) {
        drawPerspectiveRoad(ctx, w, h, roadOffset * 0.7, c === "Rear");
      } else {
        const cameraId = resolveLiveCameraId(c.toLowerCase(), snapshot.cameras_available || []);
        const frame = cameraId ? liveCameraImages[cameraId] : null;
        if (frame && frame.complete && frame.naturalWidth > 0) {
          ctx.drawImage(frame, 0, 0, w, h);
        } else {
          ctx.fillStyle = "#050507";
          ctx.fillRect(0, 0, w, h);
          ctx.fillStyle = "#ff4055";
          ctx.font = "600 15px monospace";
          ctx.textAlign = "center";
          ctx.fillText("CAMERA OFFLINE — KHÔNG CÓ TÍN HIỆU THẬT", w / 2, h / 2);
          ctx.textAlign = "left";
        }
      }
      if (c === "Front" && snapshot.detections && !isDemo) {
        drawDetections(ctx, snapshot.detections, w, h);
      }
    });
  }

  function resolveLiveCameraId(slot, available) {
    const choices = {
      front: ["front"], rear: ["rear"],
      left: ["left", "left_1", "left_2"],
      right: ["right", "right_1", "right_2"],
    }[slot] || [];
    return choices.find((id) => available.includes(id)) || null;
  }

  function refreshLiveCameras() {
    if (isStaticPreview || isDemo || !lastStatus) return;
    const available = lastStatus.snapshot?.cameras_available || [];
    available.forEach((cameraId) => {
      if (liveCameraLoading[cameraId]) return;
      const image = new Image();
      liveCameraLoading[cameraId] = true;
      image.onload = () => {
        liveCameraImages[cameraId] = image;
        liveCameraLoading[cameraId] = false;
      };
      image.onerror = () => { liveCameraLoading[cameraId] = false; };
      image.src = `/api/camera/${encodeURIComponent(cameraId)}.jpg?t=${Date.now()}`;
    });
  }

  // --------------------------------------------------------------------------
  // LiDAR Bird's Eye View (BEV) Map
  // --------------------------------------------------------------------------
  function renderBevMap(snapshot) {
    const canvas = $("bevCanvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h * 0.65; // vehicle position near bottom-center

    ctx.fillStyle = "#000000";
    ctx.fillRect(0, 0, w, h);

    // Range rings (5m, 10m, 20m)
    const scale = 5.5; // pixels per meter
    const ranges = [5, 10, 20];

    ctx.strokeStyle = "rgba(255, 255, 255, 0.1)";
    ctx.lineWidth = 1;
    ranges.forEach((r) => {
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    });

    // Cross axes
    ctx.beginPath();
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
    ctx.moveTo(0, cy); ctx.lineTo(w, cy);
    ctx.stroke();

    // Draw Ego vehicle at center
    ctx.fillStyle = "#ffffff";
    const vw = 14;
    const vh = 24;
    ctx.fillRect(cx - vw / 2, cy - vh / 2, vw, vh);

    // Direction arrow
    ctx.fillStyle = "#000000";
    ctx.beginPath();
    ctx.moveTo(cx, cy - vh / 2 + 3);
    ctx.lineTo(cx - 3, cy - vh / 2 + 9);
    ctx.lineTo(cx + 3, cy - vh / 2 + 9);
    ctx.fill();

    // Draw LiDAR points
    const points = snapshot.lidar_points || [];
    if (snapshot.health?.lidar !== "error") {
      ctx.fillStyle = "#22c55e";
      points.forEach(([px, py]) => {
        const sx = cx + px * scale;
        const sy = cy - py * scale;
        ctx.fillRect(sx, sy, 2, 2);
      });
    }
  }

  // --------------------------------------------------------------------------
  // Cockpit Telemetry & State Updates
  // --------------------------------------------------------------------------
  function updateTelemetry(data) {
    const snap = data.snapshot || {};
    const warn = snap.warning || { level: "safe", distance_m: 25.0, message: "AN TOÀN" };
    const veh = snap.vehicle || { gear: "D", turn_signal: "none" };
    const health = snap.health || {};
    const sideRadar = snap.side_radar || {};

    // Distance warning banner
    const banner = $("distanceBanner");
    const bannerTitle = $("bannerTitle");
    const bannerSub = $("bannerSub");
    const bannerDistVal = $("bannerDistVal");
    const threatBadge = $("threatBadge");
    const threatText = $("threatText");

    if (banner) {
      banner.className = `distance-banner banner-${warn.level}`;
    }
    if (bannerTitle) bannerTitle.textContent = warn.message;
    if (bannerDistVal) bannerDistVal.textContent = warn.distance_m == null ? "--" : `${warn.distance_m.toFixed(1)} m`;

    if (threatBadge && threatText) {
      threatBadge.className = `threat-badge threat-${warn.level}`;
      threatText.textContent = (warn.level || "safe").toUpperCase();
    }

    // Active Camera Badge
    if ($("activeCamBadge")) {
      const c = (snap.active_camera || activeCamera).toUpperCase();
      $("activeCamBadge").textContent = c === "REAR" ? "CAMERA SAU (LÙI)" : `CAMERA ${c}`;
    }

    // Camera thumbnails active state
    document.querySelectorAll(".cam-thumb").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.cam === snap.active_camera);
    });

    // Camera lost overlay
    if ($("camLostOverlay")) {
      $("camLostOverlay").style.display = health.camera === "error" ? "flex" : "none";
    }

    // Vehicle Gear
    ["P", "R", "N", "D"].forEach((g) => {
      const el = $("gear" + g);
      if (el) {
        el.classList.toggle("active", veh.gear === g);
        if (g === "R") el.classList.toggle("gear-r", veh.gear === "R");
      }
    });

    // Turn Signals
    if ($("signalLeft")) {
      $("signalLeft").classList.toggle("blinking", veh.turn_signal === "left");
    }
    if ($("signalRight")) {
      $("signalRight").classList.toggle("blinking", veh.turn_signal === "right");
    }

    const updateRadar = (id, detected, distance, linkOk) => {
      const el = $(id);
      if (!el) return;
      el.classList.toggle("detected", Boolean(detected));
      el.classList.toggle("offline", !linkOk);
      const value = el.querySelector("strong");
      if (value) value.textContent = !linkOk ? "OFFLINE" : (detected ? (distance == null ? "CÓ XE" : `${Number(distance).toFixed(1)}m`) : "TRỐNG");
    };
    updateRadar("radarLeftState", sideRadar.left, sideRadar.left_m, sideRadar.link_ok);
    updateRadar("radarRightState", sideRadar.right, sideRadar.right_m, sideRadar.link_ok);

    if ($("valFps")) $("valFps").textContent = snap.fps ?? "--";
    if ($("valLatency")) $("valLatency").textContent = snap.latency_ms == null ? "--" : `${snap.latency_ms}ms`;

    // Subsystems Health
    const setHealth = (id, val) => {
      const el = $(id);
      if (!el) return;
      const isOk = val === "ok";
      el.textContent = val === "disabled" ? "OFF" : (isOk ? "OK" : (val === "error" ? "ERR" : "WARN"));
      el.className = `subsys-val ${isOk ? "val-ok" : (val === "error" ? "val-err" : "val-warn")}`;
    };
    setHealth("healthCam", health.camera);
    setHealth("healthLidar", health.lidar);
    setHealth("healthAi", health.ai);
    setHealth("healthFusion", health.fusion);
    setHealth("healthCalib", health.calibration);
    setHealth("healthSys", health.system);

    // Objects List
    const objList = $("objectsList");
    const countEl = $("detectedCount");
    if (objList) {
      const dets = snap.detections || [];
      if (countEl) countEl.textContent = `${dets.length} mục`;
      objList.innerHTML = "";
      dets.forEach((d) => {
        const row = document.createElement("div");
        row.className = "object-row";
        row.innerHTML = `
          <span class="object-label">${d.label}</span>
          <span class="object-dist">${(d.distance_m || 0).toFixed(1)}m</span>
        `;
        objList.appendChild(row);
      });
    }

    // Render Canvas frames
    renderPrimaryViewport(snap);
    if (currentTab === "cameras") {
      renderMultiCameras(snap);
    }
    renderBevMap(snap);
  }

  // --------------------------------------------------------------------------
  // Diagnostics & 5 Hardware Readiness Cards
  // --------------------------------------------------------------------------
  function renderDiagnostics(data) {
    const grid = $("readinessGrid");
    const list = $("issuesList");
    const heading = $("issuesHeading");
    const diagBadge = $("diagBadge");
    if (!grid || !list) return;

    const errors = data.errors || [];
    const isReady = Boolean(data.ready);
    const isDemoMode = Boolean(data.demo);

    // Update Diag notification badge
    if (diagBadge) {
      diagBadge.style.display = !isReady && errors.length > 0 ? "inline-block" : "none";
    }

    // Render 5 Cards
    grid.innerHTML = "";
    HARDWARE_COMPONENTS.forEach((comp) => {
      const compErrors = errors.filter((e) => {
        const low = e.toLowerCase();
        return comp.keys.some((k) => low.includes(k));
      });

      const isOk = isReady || compErrors.length === 0;
      let badgeText = "READY";
      let badgeClass = "ok";
      let detailText = "Hoạt động bình thường";

      if (isDemoMode) {
        badgeText = "SIMULATED";
        badgeClass = "sim";
        detailText = "Mô phỏng trong RAM";
      } else if (!isOk) {
        badgeText = "NOT READY";
        badgeClass = "bad";
        detailText = compErrors[0] || "Yêu cầu kiểm tra";
      }

      const card = document.createElement("div");
      card.className = "ready-card";
      card.innerHTML = `
        <div class="ready-card-top">
          <span class="ready-card-name">${comp.name}</span>
          <span class="ready-badge ${badgeClass}">${badgeText}</span>
        </div>
        <p class="ready-card-detail">${detailText}</p>
      `;
      grid.appendChild(card);
    });

    // Render Issue List (Safe textContent injection)
    list.innerHTML = "";
    if (isDemoMode) {
      if (heading) heading.textContent = "Chế Độ Giao Diện Demo";
      const box = document.createElement("div");
      box.className = "ready-banner";
      box.innerHTML = `
        <div class="ready-banner-icon">✓</div>
        <div>
          <strong>DEMO UI COCKPIT HOẠT ĐỘNG HOÀN TOÀN TRONG RAM</strong>
          <p style="font-size:11px;margin-top:2px;">Bản xem thử giao diện vận hành xe tự hành. Không truy cập camera thật hay driver LiDAR.</p>
        </div>
      `;
      list.appendChild(box);
      return;
    }

    if (isReady && errors.length === 0) {
      if (heading) heading.textContent = "Hệ Thống Đã Sẵn Sàng";
      const box = document.createElement("div");
      box.className = "ready-banner";
      box.innerHTML = `
        <div class="ready-banner-icon">✓</div>
        <div>
          <strong>TẤT CẢ THIẾT BỊ PHẦN CỨNG ĐẠT CHUẨN LIVE</strong>
          <p style="font-size:11px;margin-top:2px;">Đóng cửa sổ này và khởi chạy shortcut ADAS Live để vận hành trực tiếp.</p>
        </div>
      `;
      list.appendChild(box);
      return;
    }

    if (heading) heading.textContent = `Điều Kiện Chưa Đạt (${errors.length} mục)`;
    errors.forEach((err, idx) => {
      const item = document.createElement("div");
      item.className = "issue-item";

      const numEl = document.createElement("span");
      numEl.className = "issue-num";
      numEl.textContent = `${idx + 1}.`;

      const textEl = document.createElement("div");
      textEl.style.flex = "1";
      textEl.textContent = err; // XSS-safe

      item.appendChild(numEl);
      item.appendChild(textEl);
      list.appendChild(item);
    });
  }

  // --------------------------------------------------------------------------
  // Poll & Render Orchestrator
  // --------------------------------------------------------------------------
  async function loadStatus() {
    try {
      let data;
      if (isStaticPreview) {
        data = getStaticPreviewData();
      } else {
        const res = await fetch("/api/status", { cache: "no-store" });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        data = await res.json();
      }
      lastStatus = data;
      isDemo = Boolean(data.demo);

      // Update Mode badge
      const pill = $("modePill");
      const pillText = $("modePillText");
      if (pill && pillText) {
        if (isDemo) {
          pill.className = "status-pill status-demo";
          pillText.textContent = "DEMO UI — NOT LIVE";
        } else if (data.ready) {
          pill.className = "status-pill status-live";
          pillText.textContent = "LIVE HARDWARE";
        } else {
          pill.className = "status-pill status-locked";
          pillText.textContent = "LIVE LOCKED";
        }
      }

      // Show/Hide Demo Elements
      if ($("demoBanner")) $("demoBanner").style.display = isDemo ? "block" : "none";
      if ($("demoControls")) $("demoControls").style.display = isDemo ? "flex" : "none";
      if ($("recheckBtn")) $("recheckBtn").style.display = isDemo ? "none" : "inline-flex";

      // If in Live mode and not ready on first load, switch to Diagnostics
      if (!isDemo && !data.ready && currentTab === "cockpit" && !window._hasSwitchedTab) {
        window._hasSwitchedTab = true;
        switchTab("diagnostics");
      }

      // Update Subsections
      updateTelemetry(data);
      renderDiagnostics(data);
    } catch (err) {
      const pill = $("modePill");
      const pillText = $("modePillText");
      if (pill && pillText) {
        pill.className = "status-pill status-offline";
        pillText.textContent = "DASHBOARD OFFLINE";
      }
    }
  }

  function refreshLiveFrame() {
    if (isStaticPreview || isDemo || liveFrameLoading) return;
    liveFrameLoading = true;
    const nextFrame = new Image();
    nextFrame.onload = () => {
      liveFrameImage = nextFrame;
      liveFrameLoading = false;
    };
    nextFrame.onerror = () => { liveFrameLoading = false; };
    nextFrame.src = `/api/camera/active.jpg?t=${Date.now()}`;
  }

  // --------------------------------------------------------------------------
  // Demo Actions Handler
  // --------------------------------------------------------------------------
  async function triggerDemoAction(action) {
    if (!isDemo) return;
    if (isStaticPreview) {
      applyStaticPreviewAction(action);
      showToast(`Mô phỏng: ${action.toUpperCase()}`);
      await loadStatus();
      return;
    }
    try {
      const res = await fetch("/api/demo/action", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: jsonSafe({ action }),
      });
      const result = await res.json();
      if (res.ok) {
        showToast(`Mô phỏng: ${action.toUpperCase()}`);
        document.querySelectorAll(".demo-btn").forEach((b) => {
          if (b.dataset.action === action && !["pause_resume", "reset"].includes(action)) {
            b.classList.add("active");
          } else if (!["pause_resume", "reset", "turn_signal", "reverse"].includes(b.dataset.action)) {
            b.classList.remove("active");
          }
        });
        await loadStatus();
      } else {
        showToast(result.error || "Thao tác thất bại");
      }
    } catch (e) {
      showToast("Lỗi kết nối khi gửi hành động demo");
    }
  }

  function jsonSafe(obj) {
    return JSON.stringify(obj);
  }

  // --------------------------------------------------------------------------
  // Side sensor module configuration
  // --------------------------------------------------------------------------
  function addModuleField(grid, label, key, value, type = "text", options = null) {
    const wrap = document.createElement("div");
    wrap.className = `module-field${["camera_device", "radar_device"].includes(key) ? " wide" : ""}`;
    const lbl = document.createElement("label");
    lbl.textContent = label;
    let input;
    if (options) {
      input = document.createElement("select");
      options.forEach(([optionValue, optionLabel]) => {
        const option = document.createElement("option");
        option.value = optionValue;
        option.textContent = optionLabel;
        input.appendChild(option);
      });
    } else {
      input = document.createElement("input");
      input.type = type;
      if (type === "number") input.step = "any";
    }
    input.dataset.field = key;
    input.value = value ?? "";
    wrap.append(lbl, input);
    grid.appendChild(wrap);
    return input;
  }

  function renderModuleConfig(config, readOnly = false) {
    moduleConfig = config;
    ["left", "right"].forEach((side) => {
      const host = $(side + "Modules");
      if (!host) return;
      host.innerHTML = "";
      Object.entries(config.modules || {}).filter(([, item]) => item.side === side).forEach(([moduleId, item]) => {
        const card = document.createElement("div");
        card.className = `module-config-card${item.enabled ? "" : " disabled"}`;
        card.dataset.moduleId = moduleId;
        const head = document.createElement("div");
        head.className = "module-card-head";
        const name = document.createElement("span");
        name.className = "module-card-name";
        name.textContent = moduleId.toUpperCase().replace("_", "-");
        const enableLabel = document.createElement("label");
        enableLabel.className = "module-enable";
        const enabled = document.createElement("input");
        enabled.type = "checkbox";
        enabled.dataset.field = "enabled";
        enabled.checked = Boolean(item.enabled);
        enabled.disabled = readOnly;
        enableLabel.append(enabled, document.createTextNode(" BẬT MODULE"));
        head.append(name, enableLabel);
        const grid = document.createElement("div");
        grid.className = "module-form-grid";
        addModuleField(grid, "VỊ TRÍ", "position", item.position, "text", [["front", "Phía trước"], ["rear", "Phía sau"]]);
        addModuleField(grid, "CAMERA DEVICE", "camera_device", item.camera.device);
        addModuleField(grid, "CHIỀU RỘNG", "camera_width", item.camera.width, "number");
        addModuleField(grid, "CHIỀU CAO", "camera_height", item.camera.height, "number");
        addModuleField(grid, "CAMERA FPS", "camera_fps", item.camera.fps, "number");
        addModuleField(grid, "UART ESP32", "radar_device", item.radar.device);
        addModuleField(grid, "BAUDRATE", "radar_baudrate", item.radar.baudrate, "number");
        addModuleField(grid, "TẦM RADAR (m)", "radar_max", item.radar.max_distance_m, "number");
        addModuleField(grid, "NGƯỠNG CẢNH BÁO (m)", "radar_warning", item.radar.warning_distance_m, "number");
        grid.querySelectorAll("input,select").forEach((input) => { input.disabled = readOnly; });
        const actions = document.createElement("div");
        actions.className = "module-actions";
        const test = document.createElement("button");
        test.className = "btn-secondary module-test-btn";
        test.textContent = "Kiểm tra thiết bị";
        test.disabled = readOnly;
        const result = document.createElement("span");
        result.className = "module-test-result";
        actions.append(test, result);
        enabled.addEventListener("change", () => card.classList.toggle("disabled", !enabled.checked));
        test.addEventListener("click", () => testModule(moduleId, result));
        card.append(head, grid, actions);
        host.appendChild(card);
      });
    });
    if ($("saveModulesBtn")) $("saveModulesBtn").disabled = readOnly;
  }

  async function loadModuleConfig() {
    try {
      if (isStaticPreview) {
        const modules = {};
        [["left_1", "left", "front", "/dev/video1", "/dev/ttyACM0", true],
         ["left_2", "left", "rear", "/dev/video3", "/dev/ttyACM2", false],
         ["right_1", "right", "front", "/dev/video2", "/dev/ttyACM1", true],
         ["right_2", "right", "rear", "/dev/video4", "/dev/ttyACM3", false]].forEach(([id, side, position, camera, uart, enabled]) => {
          modules[id] = { enabled, side, position,
            camera: { device: camera, width: 1280, height: 720, fps: 30 },
            radar: { enabled: true, device: uart, baudrate: 115200, max_distance_m: 30, warning_distance_m: 8 } };
        });
        renderModuleConfig({ enabled: true, max_modules_per_side: 2, camera_hold_seconds: 1.2, modules }, false);
        $("configNotice").textContent = "BẢN XEM TRƯỚC · Có thể chỉnh thử trong RAM; chạy ADAS Live để lưu vào thiết bị thật.";
        return;
      }
      const res = await fetch("/api/config/modules", { cache: "no-store" });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
      renderModuleConfig(data.config, false);
      if (data.demo) $("configNotice").textContent = "DEMO · Cấu hình thử chỉ được giữ trong RAM, không ghi vào phần cứng.";
    } catch (err) {
      showToast(`Không tải được cấu hình module: ${err.message}`);
    }
  }

  function collectModuleConfig() {
    const result = JSON.parse(JSON.stringify(moduleConfig));
    document.querySelectorAll(".module-config-card").forEach((card) => {
      const id = card.dataset.moduleId;
      const get = (field) => card.querySelector(`[data-field="${field}"]`);
      const item = result.modules[id];
      item.enabled = get("enabled").checked;
      item.position = get("position").value;
      item.camera.device = get("camera_device").value.trim();
      item.camera.width = Number(get("camera_width").value);
      item.camera.height = Number(get("camera_height").value);
      item.camera.fps = Number(get("camera_fps").value);
      item.radar.device = get("radar_device").value.trim();
      item.radar.baudrate = Number(get("radar_baudrate").value);
      item.radar.max_distance_m = Number(get("radar_max").value);
      item.radar.warning_distance_m = Number(get("radar_warning").value);
    });
    return result;
  }

  async function saveModuleConfig() {
    if (!moduleConfig) return;
    const btn = $("saveModulesBtn");
    btn.disabled = true;
    if (isDemo) {
      moduleConfig = collectModuleConfig();
      renderModuleConfig(moduleConfig, false);
      showToast("Đã áp dụng cấu hình thử trong RAM");
      $("configNotice").textContent = "DEMO · Đã áp dụng trong RAM; cấu hình này không ghi vào thiết bị thật.";
      return;
    }
    try {
      const res = await fetch("/api/config/modules", {
        method: "PUT", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: collectModuleConfig() }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || "Lưu thất bại");
      renderModuleConfig(data.config, false);
      showToast(data.message);
      $("configNotice").textContent = "ĐÃ LƯU · Cần khởi động lại ADAS Live để áp dụng cấu hình mới.";
    } catch (err) {
      showToast(`Lỗi cấu hình: ${err.message}`, 5000);
    } finally {
      btn.disabled = false;
    }
  }

  async function testModule(moduleId, target) {
    target.className = "module-test-result";
    target.textContent = "Đang kiểm tra...";
    if (isDemo) {
      target.classList.add("ok");
      target.textContent = "SIMULATED · Camera và UART mô phỏng sẵn sàng";
      return;
    }
    try {
      const res = await fetch("/api/config/modules/test", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ module_id: moduleId }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || "Không kiểm tra được");
      target.classList.add(data.ok ? "ok" : "bad");
      target.textContent = data.ok ? "Camera và UART sẵn sàng" : "Thiếu camera hoặc quyền truy cập UART";
    } catch (err) {
      target.classList.add("bad");
      target.textContent = err.message;
    }
  }

  // --------------------------------------------------------------------------
  // Hardware assignment (camera position, COM, LiDAR)
  // --------------------------------------------------------------------------
  function hwEscape(value) {
    return String(value ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function hwDeviceOptions(kind, current) {
    const devices = hardwareDevices || {};
    const opts = [{ value: "", label: "— Chưa gán —" }];
    const seen = new Set([""]);
    const push = (value, label) => {
      const key = String(value);
      if (seen.has(key)) return;
      seen.add(key);
      opts.push({ value: key, label });
    };
    if (kind === "camera") {
      (devices.cameras || []).forEach((path) => push(path, path));
      (devices.camera_indices || []).forEach((index) => push(String(index), `Index ${index}`));
    } else {
      const serial = devices.serial || {};
      ["by_id", "ttyacm", "ttyusb", "com"].forEach((key) => {
        (serial[key] || []).forEach((path) => push(path, `${key.toUpperCase()} · ${path}`));
      });
    }
    if (current && !seen.has(String(current))) {
      push(current, `${current} (đã lưu)`);
    }
    return opts;
  }

  function hwFillSelect(select, options, selected) {
    if (!select) return;
    select.innerHTML = "";
    options.forEach((item) => {
      const opt = document.createElement("option");
      opt.value = item.value;
      opt.textContent = item.label;
      if (String(item.value) === String(selected || "")) opt.selected = true;
      select.appendChild(opt);
    });
  }

  function hwSlotDevice(slotId) {
    return hardwareConfig?.camera?.slots?.[slotId]?.device || "";
  }

  function hwSlotEnabled(slotId, required) {
    if (required) return true;
    const slot = hardwareConfig?.camera?.slots?.[slotId];
    if (!slot) return false;
    return Boolean(slot.enabled && slot.device);
  }

  function renderHardwareVehicle() {
    const host = $("hwVehicle");
    if (!host) return;
    const trunc = (value) => {
      const text = String(value || "").trim();
      if (!text) return "CHƯA GÁN";
      return text.length > 22 ? `${text.slice(0, 20)}…` : text;
    };
    const slotBtn = (slot) => {
      const device = hwSlotDevice(slot.id);
      const assigned = Boolean(device) && (slot.required || hwSlotEnabled(slot.id, slot.required));
      const selected = hardwareSelectedSlot === slot.id ? " selected" : "";
      const state = assigned ? "assigned" : "empty";
      return `<button type="button" class="hw-slot-btn ${state}${selected} hw-slot-${slot.id}" data-hw-map-slot="${slot.id}">
        <span class="hw-slot-name">${slot.label}</span>
        <span class="hw-slot-dev">${hwEscape(trunc(device))}</span>
      </button>`;
    };
    const com = hardwareConfig?.radar_gateway?.device || "—";
    const lidar = hardwareConfig?.lidar?.source || "—";
    host.innerHTML = `
      ${slotBtn(HW_CAMERA_SLOTS[0])}
      ${slotBtn(HW_CAMERA_SLOTS[1])}
      ${slotBtn(HW_CAMERA_SLOTS[3])}
      <div class="hw-car hw-car-body">
        <div class="hw-car-glass"></div>
        <div class="hw-car-label">ADAS XE</div>
        <div class="hw-car-meta">COM ${hwEscape(trunc(com))}<br>LIDAR ${hwEscape(trunc(lidar))}</div>
      </div>
      ${slotBtn(HW_CAMERA_SLOTS[2])}
      ${slotBtn(HW_CAMERA_SLOTS[4])}
      ${slotBtn(HW_CAMERA_SLOTS[5])}
    `;
    host.querySelectorAll("[data-hw-map-slot]").forEach((btn) => {
      btn.addEventListener("click", () => {
        hardwareSelectedSlot = btn.dataset.hwMapSlot;
        const card = document.querySelector(`[data-hw-slot="${hardwareSelectedSlot}"]`);
        if (card) card.scrollIntoView({ block: "nearest", behavior: "smooth" });
        renderHardware();
      });
    });
  }

  function renderHardwareCameraCards() {
    const host = $("hwCameraSlots");
    if (!host || !hardwareConfig) return;
    const camEnabled = Boolean(hardwareConfig.camera?.enabled);
    host.innerHTML = "";
    HW_CAMERA_SLOTS.forEach((meta) => {
      const slot = hardwareConfig.camera?.slots?.[meta.id] || { device: "", enabled: meta.required };
      const enabled = meta.required ? true : Boolean(slot.enabled);
      const card = document.createElement("div");
      card.className = `hw-slot-card${hardwareSelectedSlot === meta.id ? " selected" : ""}${enabled && camEnabled ? "" : " disabled"}`;
      card.dataset.hwSlot = meta.id;
      const options = hwDeviceOptions("camera", slot.device)
        .map((item) => `<option value="${hwEscape(item.value)}"${String(item.value) === String(slot.device || "") ? " selected" : ""}>${hwEscape(item.label)}</option>`)
        .join("");
      card.innerHTML = `
        <div class="hw-slot-card-head">
          <span class="hw-slot-card-name">${meta.full}</span>
          ${meta.required ? "<span class=\"card-sub\">BẮT BUỘC</span>" : `<label class="module-enable"><input data-hw="enabled" type="checkbox"${enabled ? " checked" : ""}/> BẬT</label>`}
        </div>
        <label class="hw-field wide">
          <span>THIẾT BỊ</span>
          <div class="radar-device-input">
            <select data-hw="device-select">${options}</select>
            <input data-hw="device" type="text" value="${hwEscape(slot.device || "")}" placeholder="/dev/video0 hoặc 0" />
          </div>
        </label>
        <div class="module-actions">
          <button type="button" class="btn-secondary" data-hw="test">Kiểm tra</button>
          <span class="module-test-result" data-hw="test-result"></span>
        </div>
      `;
      const select = card.querySelector('[data-hw="device-select"]');
      const input = card.querySelector('[data-hw="device"]');
      const enable = card.querySelector('[data-hw="enabled"]');
      select.addEventListener("change", () => {
        if (select.value) input.value = select.value;
        syncHardwareFromDom();
        renderHardwareVehicle();
      });
      input.addEventListener("change", () => {
        syncHardwareFromDom();
        renderHardwareVehicle();
      });
      if (enable) {
        enable.addEventListener("change", () => {
          syncHardwareFromDom();
          renderHardware();
        });
      }
      card.querySelector('[data-hw="test"]').addEventListener("click", () => {
        testHardwareKind("camera", input.value.trim(), card.querySelector("[data-hw=\"test-result\"]"), true);
      });
      card.addEventListener("click", () => {
        hardwareSelectedSlot = meta.id;
        host.querySelectorAll(".hw-slot-card").forEach((el) => {
          el.classList.toggle("selected", el.dataset.hwSlot === meta.id);
        });
        document.querySelectorAll("[data-hw-map-slot]").forEach((el) => {
          el.classList.toggle("selected", el.dataset.hwMapSlot === meta.id);
        });
      });
      host.appendChild(card);
    });
  }

  function fillHardwareGlobals() {
    if (!hardwareConfig) return;
    const cam = hardwareConfig.camera || {};
    const radar = hardwareConfig.radar_gateway || {};
    const lidar = hardwareConfig.lidar || {};
    if ($("hwCamEnabled")) $("hwCamEnabled").checked = Boolean(cam.enabled);
    if ($("hwCamWidth")) $("hwCamWidth").value = cam.width || 1280;
    if ($("hwCamHeight")) $("hwCamHeight").value = cam.height || 720;
    if ($("hwCamFps")) $("hwCamFps").value = cam.fps || 30;
    if ($("hwRadarEnabled")) $("hwRadarEnabled").checked = Boolean(radar.enabled);
    if ($("hwRadarDevice")) $("hwRadarDevice").value = radar.device || "";
    const baud = $("hwRadarBaud");
    if (baud && baud.options.length === 0) {
      [9600, 19200, 38400, 57600, 115200, 230400, 256000].forEach((rate) => {
        const opt = document.createElement("option");
        opt.value = String(rate);
        opt.textContent = `${rate} baud`;
        baud.appendChild(opt);
      });
    }
    if (baud) baud.value = String(radar.baudrate || 115200);
    hwFillSelect($("hwRadarDeviceSelect"), hwDeviceOptions("serial", radar.device), radar.device);
    if ($("hwLidarEnabled")) $("hwLidarEnabled").checked = Boolean(lidar.enabled);
    if ($("hwLidarDriver")) $("hwLidarDriver").value = lidar.driver || "";
    if ($("hwLidarSource")) $("hwLidarSource").value = lidar.source || "";
    if ($("hwLidarFps")) $("hwLidarFps").value = lidar.fps || 10;
    if ($("hwLidarModule")) $("hwLidarModule").value = lidar.driver_module || "";
    hwFillSelect($("hwLidarSourceSelect"), hwDeviceOptions("serial", lidar.source), lidar.source);
  }

  function renderHardware() {
    fillHardwareGlobals();
    renderHardwareCameraCards();
    renderHardwareVehicle();
  }

  function collectHardwareConfig() {
    const slots = {};
    HW_CAMERA_SLOTS.forEach((meta) => {
      const card = document.querySelector(`[data-hw-slot="${meta.id}"]`);
      const enabled = meta.required ? true : Boolean(card?.querySelector('[data-hw="enabled"]')?.checked);
      const device = (card?.querySelector('[data-hw="device"]')?.value || "").trim();
      slots[meta.id] = { enabled, device };
    });
    return {
      camera: {
        enabled: Boolean($("hwCamEnabled")?.checked),
        width: Number($("hwCamWidth")?.value),
        height: Number($("hwCamHeight")?.value),
        fps: Number($("hwCamFps")?.value),
        slots,
      },
      radar_gateway: {
        enabled: Boolean($("hwRadarEnabled")?.checked),
        device: ($("hwRadarDevice")?.value || "").trim(),
        baudrate: Number($("hwRadarBaud")?.value),
      },
      lidar: {
        enabled: Boolean($("hwLidarEnabled")?.checked),
        driver: ($("hwLidarDriver")?.value || "").trim(),
        source: ($("hwLidarSource")?.value || "").trim(),
        fps: Number($("hwLidarFps")?.value),
        driver_module: ($("hwLidarModule")?.value || "").trim(),
      },
    };
  }

  function syncHardwareFromDom() {
    if (!hardwareConfig) return;
    const collected = collectHardwareConfig();
    HW_CAMERA_SLOTS.forEach((meta) => {
      const current = hardwareConfig.camera.slots[meta.id] || { label: meta.full, required: meta.required };
      hardwareConfig.camera.slots[meta.id] = { ...current, ...collected.camera.slots[meta.id] };
    });
    hardwareConfig.camera.enabled = collected.camera.enabled;
    hardwareConfig.camera.width = collected.camera.width;
    hardwareConfig.camera.height = collected.camera.height;
    hardwareConfig.camera.fps = collected.camera.fps;
    hardwareConfig.radar_gateway = { ...hardwareConfig.radar_gateway, ...collected.radar_gateway };
    hardwareConfig.lidar = { ...hardwareConfig.lidar, ...collected.lidar };
  }

  async function scanHardwareDevices() {
    if (isStaticPreview) {
      hardwareDevices = HW_PREVIEW_DEVICES;
      renderHardware();
      showToast("BẢN XEM TRƯỚC · danh sách thiết bị mô phỏng");
      return;
    }
    try {
      const res = await fetch("/api/hardware/devices", { cache: "no-store" });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
      hardwareDevices = data.devices || {};
      renderHardware();
      showToast("Đã quét camera và cổng COM/UART");
    } catch (err) {
      showToast(`Không quét được thiết bị: ${err.message}`);
    }
  }

  async function loadHardwareConfig() {
    if (!$("viewHardware")) return;
    if (isStaticPreview) {
      hardwareConfig = JSON.parse(JSON.stringify(HW_PREVIEW));
      hardwareDevices = HW_PREVIEW_DEVICES;
      renderHardware();
      if ($("hwNotice")) $("hwNotice").textContent = "BẢN XEM TRƯỚC · Chỉnh thử trong RAM; chạy ADAS Live để lưu vào YAML.";
      return;
    }
    try {
      const [cfgRes, devRes] = await Promise.all([
        fetch("/api/hardware/config", { cache: "no-store" }),
        fetch("/api/hardware/devices", { cache: "no-store" }),
      ]);
      const cfgData = await cfgRes.json();
      if (!cfgRes.ok) throw new Error(cfgData.error || `HTTP ${cfgRes.status}`);
      hardwareConfig = cfgData.config;
      try {
        const devData = await devRes.json();
        if (devRes.ok) hardwareDevices = devData.devices || {};
      } catch (_err) {
        hardwareDevices = { cameras: [], camera_indices: ["0", "1", "2", "3"], serial: { by_id: [], ttyacm: [], ttyusb: [], com: [] } };
      }
      renderHardware();
      if (cfgData.demo && $("hwNotice")) {
        $("hwNotice").textContent = "DEMO · Có thể chỉnh trên giao diện; lưu chỉ giữ trong RAM, không ghi YAML.";
      }
    } catch (err) {
      showToast(`Không tải được cấu hình phần cứng: ${err.message}`);
    }
  }

  async function saveHardwareConfig() {
    if (!hardwareConfig) return;
    const payload = collectHardwareConfig();
    if (isDemo || isStaticPreview) {
      hardwareConfig = {
        ...hardwareConfig,
        camera: { ...hardwareConfig.camera, ...payload.camera, slots: Object.fromEntries(
          HW_CAMERA_SLOTS.map((meta) => {
            const slot = payload.camera.slots[meta.id];
            return [meta.id, { ...(hardwareConfig.camera.slots[meta.id] || {}), ...slot }];
          })
        ) },
        radar_gateway: { ...hardwareConfig.radar_gateway, ...payload.radar_gateway },
        lidar: { ...hardwareConfig.lidar, ...payload.lidar },
      };
      renderHardware();
      if ($("hwNotice")) $("hwNotice").textContent = "DEMO · Đã áp dụng trong RAM; không ghi vào thiết bị thật.";
      showToast("Đã áp dụng cấu hình phần cứng trong RAM");
      return;
    }
    const btn = $("hwSaveBtn");
    if (btn) btn.disabled = true;
    try {
      const res = await fetch("/api/hardware/config", {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config: payload }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || "Lưu thất bại");
      hardwareConfig = data.config;
      renderHardware();
      if ($("hwNotice")) $("hwNotice").textContent = "ĐÃ LƯU · Cần khởi động lại ADAS Live để áp dụng camera, COM và LiDAR.";
      showToast(data.message || "Đã lưu cấu hình phần cứng");
    } catch (err) {
      showToast(`Lỗi cấu hình phần cứng: ${err.message}`, 5000);
    } finally {
      if (btn) btn.disabled = false;
    }
  }

  async function testHardwareKind(kind, device, target, probe) {
    if (!target) return;
    target.className = "module-test-result";
    target.textContent = "Đang kiểm tra...";
    if (!device) {
      target.classList.add("bad");
      target.textContent = "Chưa chọn thiết bị";
      return;
    }
    if (isDemo || isStaticPreview) {
      target.classList.add("ok");
      target.textContent = "SIMULATED · thiết bị mô phỏng sẵn sàng";
      return;
    }
    try {
      const res = await fetch("/api/hardware/test", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ kind, device, probe: Boolean(probe) }),
      });
      const data = await res.json();
      target.classList.add(data.ok ? "ok" : "bad");
      target.textContent = data.ok
        ? (data.note || (kind === "camera" ? "Đọc được frame" : "Thiết bị có mặt"))
        : (data.error || "Không đạt");
    } catch (err) {
      target.classList.add("bad");
      target.textContent = err.message;
    }
  }

  // --------------------------------------------------------------------------
  // Hardware Recheck Action
  // --------------------------------------------------------------------------
  async function handleRecheck() {
    if (isRechecking || isDemo) return;
    const btn = $("recheckBtn");
    if (!btn) return;

    isRechecking = true;
    btn.disabled = true;
    btn.classList.add("loading");
    showToast("Đang kiểm tra lại camera, CUDA, AI model và LiDAR...");

    try {
      const res = await fetch("/api/recheck", { method: "POST", cache: "no-store" });
      const data = await res.json();
      if (res.status === 409) {
        showToast("Tiến trình kiểm tra đang bận, vui lòng chờ...");
      } else if (!res.ok) {
        showToast(data.error || "Kiểm tra thất bại.");
      } else {
        showToast(data.ok ? "Preflight hoàn tất: Sẵn sàng Live!" : "Preflight hoàn tất: Vẫn còn lỗi.");
      }
      await loadStatus();
    } catch (e) {
      showToast("Lỗi kết nối khi kiểm tra phần cứng");
    } finally {
      isRechecking = false;
      btn.disabled = false;
      btn.classList.remove("loading");
    }
  }

  // --------------------------------------------------------------------------
  // Animation Loop (60 FPS)
  // --------------------------------------------------------------------------
  function animLoop() {
    roadOffset += 1;
    if (lastStatus && currentTab === "cockpit") {
      renderPrimaryViewport(lastStatus.snapshot || {});
    }
    animFrameId = requestAnimationFrame(animLoop);
  }

  // --------------------------------------------------------------------------
  // Event Bindings
  // --------------------------------------------------------------------------
  document.addEventListener("DOMContentLoaded", () => {
    // Navigation Tabs
    document.querySelectorAll(".nav-tab").forEach((btn) => {
      btn.addEventListener("click", () => switchTab(btn.dataset.tab));
    });

    // Camera Thumbnails
    document.querySelectorAll(".cam-thumb").forEach((btn) => {
      btn.addEventListener("click", async () => {
        const cam = btn.dataset.cam;
        activeCamera = cam;
        if (lastStatus?.snapshot?.mode === "camera-only") {
          await fetch("/api/camera/select", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({camera: cam})});
          await loadStatus();
        } else if (isDemo) {
          if (isStaticPreview) previewState.active_camera = cam;
          else await fetch("/api/demo/camera", {
            method: "POST", headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ camera: cam }),
          });
          loadStatus();
        }
      });
    });

    // Demo Action Buttons
    document.querySelectorAll(".demo-btn").forEach((btn) => {
      btn.addEventListener("click", () => triggerDemoAction(btn.dataset.action));
    });

    // Recheck Button
    const rBtn = $("recheckBtn");
    if (rBtn) rBtn.addEventListener("click", handleRecheck);
    const saveBtn = $("saveModulesBtn");
    if (saveBtn) saveBtn.addEventListener("click", saveModuleConfig);
    const hwSave = $("hwSaveBtn");
    if (hwSave) hwSave.addEventListener("click", saveHardwareConfig);
    const hwScan = $("hwScanBtn");
    if (hwScan) hwScan.addEventListener("click", scanHardwareDevices);
    const hwRadarSelect = $("hwRadarDeviceSelect");
    if (hwRadarSelect) {
      hwRadarSelect.addEventListener("change", () => {
        if (hwRadarSelect.value) $("hwRadarDevice").value = hwRadarSelect.value;
        syncHardwareFromDom();
        renderHardwareVehicle();
      });
    }
    const hwLidarSelect = $("hwLidarSourceSelect");
    if (hwLidarSelect) {
      hwLidarSelect.addEventListener("change", () => {
        if (hwLidarSelect.value) $("hwLidarSource").value = hwLidarSelect.value;
        syncHardwareFromDom();
        renderHardwareVehicle();
      });
    }
    ["hwCamEnabled", "hwCamWidth", "hwCamHeight", "hwCamFps", "hwRadarEnabled", "hwRadarDevice", "hwRadarBaud", "hwLidarEnabled", "hwLidarDriver", "hwLidarSource", "hwLidarFps", "hwLidarModule"].forEach((id) => {
      const el = $(id);
      if (el) el.addEventListener("change", () => {
        syncHardwareFromDom();
        renderHardwareVehicle();
      });
    });
    const hwRadarTest = $("hwRadarTestBtn");
    if (hwRadarTest) {
      hwRadarTest.addEventListener("click", () => {
        testHardwareKind("serial", ($("hwRadarDevice")?.value || "").trim(), $("hwRadarTestResult"), false);
      });
    }
    const hwLidarTest = $("hwLidarTestBtn");
    if (hwLidarTest) {
      hwLidarTest.addEventListener("click", () => {
        testHardwareKind("lidar", ($("hwLidarSource")?.value || "").trim(), $("hwLidarTestResult"), false);
      });
    }

    // Fullscreen
    const fsBtn = $("fullscreenBtn");
    if (fsBtn) {
      fsBtn.addEventListener("click", () => {
        if (!document.fullscreenElement) {
          document.documentElement.requestFullscreen().catch(() => {});
        } else {
          document.exitFullscreen().catch(() => {});
        }
      });
    }

    // Initial fetch & start loops
    loadStatus();
    loadModuleConfig();
    loadRadarConfig();
    loadHardwareConfig();
    setInterval(loadStatus, 250);
    setInterval(refreshLiveFrame, 100);
    setInterval(refreshLiveCameras, 100);
    setInterval(loadRadarStatus, 1000);
    animLoop();

    // ----- Radar Gateway bindings -----
    const radarSaveBtn = $("saveRadarBtn");
    if (radarSaveBtn) radarSaveBtn.addEventListener("click", saveRadarConfig);
    const radarTestBtn = $("radarTestBtn");
    if (radarTestBtn) radarTestBtn.addEventListener("click", testRadarGateway);
    const radarRescanBtn = $("radarRescanBtn");
    if (radarRescanBtn) {
      radarRescanBtn.addEventListener("click", () => loadRadarConfig());
    }
    const radarDeviceSelect = $("radarDeviceSelect");
    if (radarDeviceSelect) {
      radarDeviceSelect.addEventListener("change", () => {
        if (radarDeviceSelect.value) {
          $("radarDevice").value = radarDeviceSelect.value;
          radarConfigDirty = true;
          updateRadarRestartBanner();
        }
      });
    }
    ["radarEnabled", "radarDevice", "radarBaudrate", "radarProtocol", "radarStale", "radarHold"].forEach((id) => {
      const el = $(id);
      if (el) el.addEventListener("change", () => {
        radarConfigDirty = true;
        updateRadarRestartBanner();
      });
    });

    // =================================================================
    // Radar Gateway helpers
    // =================================================================
    function buildRadarStat(label, value, classes = "") {
      return `
        <div class="radar-stat">
          <span class="radar-stat-label">${label}</span>
          <span class="radar-stat-value ${classes}">${value}</span>
        </div>
      `;
    }

    function renderRadarTelemetry(status) {
      const gatewayBadge = $("radarGatewayBadge");
      const remoteBadge = $("radarRemoteBadge");
      if (gatewayBadge) {
        const online = status && status.gateway_online;
        gatewayBadge.textContent = online ? "ONLINE" : "OFFLINE";
        gatewayBadge.className = `radar-badge ${online ? "online" : "offline"}`;
      }
      if (remoteBadge) {
        const online = status && status.remote_online;
        remoteBadge.textContent = online ? "ONLINE" : "OFFLINE";
        remoteBadge.className = `radar-badge ${online ? "online" : "offline"}`;
      }
      const setText = (id, value) => { const el = $(id); if (el) el.textContent = value; };
      setText("radarGatewayBootId", status?.gateway_boot_id || "--");
      setText("radarRemoteBootId", status?.remote_boot_id || "--");
      setText("radarLastSeq", status?.last_seq == null || status.last_seq < 0 ? "--" : `#${status.last_seq}`);
      setText("radarRateHz", `${(status?.packet_rate_hz || 0).toFixed(1)} Hz`);
      setText("radarLoss", `${((status?.packet_loss || 0) * 100).toFixed(1)}%`);
      setText("radarErrors", `${status?.json_errors ?? 0} / ${status?.crc_errors ?? 0}`);
      setText("radarAge", status?.packet_age_ms == null ? "--" : `${status.packet_age_ms} ms`);
      setText("radarNote", status?.error || "Gateway đang gửi gói JSON v2 bình thường.");
    }

    function renderRadarCards(config, status) {
      const host = $("radarRadars");
      if (!host || !config) return;
      host.innerHTML = "";
      const radarIds = ["left_front", "left_rear", "right_front", "right_rear"];
      const live = status?.radars || {};
      radarIds.forEach((radarId) => {
        const cfg = (config.radars || {})[radarId] || { enabled: false, camera: "", warning_distance_m: 5.0 };
        const liveFrame = live[radarId] || {};
        const card = document.createElement("div");
        card.className = `radar-radar-card${cfg.enabled ? "" : " disabled"}`;
        card.dataset.radarId = radarId;
        const cameraOptions = (config.cameras_available || config.camerasAvailable || ["left_1", "left_2", "right_1", "right_2"])
          .map((cam) => `<option value="${cam}"${cfg.camera === cam ? " selected" : ""}>${cam}</option>`).join("");
        const det = !!liveFrame.detected;
        const stale = !!liveFrame.stale;
        const valid = !!liveFrame.valid;
        const dist = liveFrame.distance_m == null ? "--" : `${Number(liveFrame.distance_m).toFixed(2)} m`;
        const movingE = liveFrame.moving_energy == null ? "--" : liveFrame.moving_energy;
        const stationaryE = liveFrame.stationary_energy == null ? "--" : liveFrame.stationary_energy;
        const ageMs = liveFrame.age_ms == null ? "--" : `${liveFrame.age_ms} ms`;
        card.innerHTML = `
          <div class="radar-radar-head">
            <span class="radar-radar-title">${radarId.toUpperCase().replace("_", " ")}</span>
            <label class="radar-toggle">
              <input type="checkbox" data-radar-field="enabled"${cfg.enabled ? " checked" : ""} />
              <span>BẬT RADAR</span>
            </label>
          </div>
          <div class="radar-radar-form">
            <label class="radar-field">
              <span>CAMERA</span>
              <select data-radar-field="camera">${cameraOptions || `<option value="${cfg.camera || ""}">${cfg.camera || "--"}</option>`}</select>
            </label>
            <label class="radar-field">
              <span>WARNING DISTANCE (m)</span>
              <input type="number" min="0.1" max="5" step="0.1" data-radar-field="warning_distance_m" value="${Number(cfg.warning_distance_m || 5)}" />
            </label>
          </div>
          <div class="radar-radar-stats">
            ${buildRadarStat("DETECTED", det ? "YES" : "NO", det ? "" : "stale")}
            ${buildRadarStat("DISTANCE", dist, stale ? "stale" : "")}
            ${buildRadarStat("MOVING E", movingE)}
            ${buildRadarStat("STATIONARY E", stationaryE)}
            ${buildRadarStat("VALID", valid ? "OK" : "ERR", valid ? "" : "invalid")}
            ${buildRadarStat("STALE", stale ? "YES" : "NO", stale ? "stale" : "")}
            ${buildRadarStat("AGE", ageMs)}
            ${buildRadarStat("ERROR", liveFrame.error || "--")}
          </div>
        `;
        host.appendChild(card);
        card.querySelectorAll("[data-radar-field]").forEach((field) => {
          field.addEventListener("change", () => {
            card.classList.toggle("disabled", !card.querySelector('[data-radar-field="enabled"]').checked);
            radarConfigDirty = true;
            updateRadarRestartBanner();
          });
        });
      });
    }

    function updateRadarRestartBanner() {
      const banner = $("radarRestartBanner");
      if (!banner) return;
      banner.hidden = !radarConfigDirty && !lastRadarSaveResult?.restart_required;
    }

    function fillRadarForm(config) {
      $("radarEnabled").checked = Boolean(config.enabled);
      $("radarDevice").value = config.device || "";
      $("radarBaudrate").value = String(config.baudrate || 115200);
      $("radarProtocol").value = String(config.protocol_version || 2);
      $("radarStale").value = Number(config.stale_after_seconds || 0.6);
      $("radarHold").value = Number(config.camera_hold_seconds || 1.2);
      const baudSelect = $("radarBaudrate");
      if (baudSelect && baudSelect.options.length === 0) {
        [9600, 19200, 38400, 57600, 115200, 230400, 256000].forEach((b) => {
          const opt = document.createElement("option");
          opt.value = String(b);
          opt.textContent = `${b} baud`;
          baudSelect.appendChild(opt);
        });
      }
    }

    function populateRadarDeviceSelect(devices) {
      const select = $("radarDeviceSelect");
      if (!select) return;
      select.innerHTML = "";
      const placeholder = document.createElement("option");
      placeholder.value = "";
      placeholder.textContent = "Chọn thiết bị đã quét…";
      select.appendChild(placeholder);
      (["by_id", "ttyacm", "ttyusb", "com"]).forEach((label) => {
        const items = (devices || {})[label] || [];
        items.forEach((path) => {
          const opt = document.createElement("option");
          opt.value = path;
          opt.textContent = `${label.toUpperCase()} · ${path}`;
          select.appendChild(opt);
        });
      });
    }

    async function loadRadarConfig() {
      if (isStaticPreview) {
        renderRadarCards(RADAR_PREVIEW_CONFIG, RADAR_PREVIEW_STATUS);
        fillRadarForm(RADAR_PREVIEW_CONFIG);
        return;
      }
      try {
        const res = await fetch("/api/radar/config", { cache: "no-store" });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
        radarConfig = data.config;
        fillRadarForm(radarConfig);
        renderRadarCards(radarConfig, radarStatus);
        radarConfigDirty = false;
        updateRadarRestartBanner();
      } catch (err) {
        showToast(`Không tải được radar_gateway: ${err.message}`);
      }
      try {
        const devRes = await fetch("/api/devices", { cache: "no-store" });
        if (devRes.ok) {
          const devData = await devRes.json();
          radarDevices = devData.devices || {};
        }
      } catch {
        /* /api/devices is best-effort */
      }
      try {
        const hwRes = await fetch("/api/hardware/devices", { cache: "no-store" });
        if (hwRes.ok) {
          const hwData = await hwRes.json();
          const com = (hwData.devices && hwData.devices.serial && hwData.devices.serial.com) || [];
          radarDevices = { ...(radarDevices || {}), com };
        }
      } catch {
        /* COM listing is best-effort */
      }
      if (radarDevices) populateRadarDeviceSelect(radarDevices);
    }

    async function loadRadarStatus() {
      if (isStaticPreview) {
        renderRadarTelemetry(RADAR_PREVIEW_STATUS);
        return;
      }
      try {
        const res = await fetch("/api/radar/status", { cache: "no-store" });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
        const status = data.ok ? data : null;
        radarStatus = status;
        renderRadarTelemetry(status);
        renderRadarCards(radarConfig, status);
      } catch (err) {
        renderRadarTelemetry({
          gateway_online: false,
          remote_online: false,
          error: `Không đọc được radar status: ${err.message}`,
        });
      }
    }

    function collectRadarConfig() {
      const cfg = radarConfig || {};
      const radars = { ...(cfg.radars || {}) };
      const radarIds = ["left_front", "left_rear", "right_front", "right_rear"];
      radarIds.forEach((rid) => {
        const card = document.querySelector(`[data-radar-id="${rid}"]`);
        if (!card) return;
        const enabled = card.querySelector('[data-radar-field="enabled"]').checked;
        const camera = card.querySelector('[data-radar-field="camera"]').value;
        const warning = Number(card.querySelector('[data-radar-field="warning_distance_m"]').value);
        radars[rid] = { ...(radars[rid] || {}), enabled, camera, warning_distance_m: warning };
      });
      return {
        enabled: $("radarEnabled").checked,
        device: $("radarDevice").value.trim(),
        baudrate: Number($("radarBaudrate").value),
        protocol_version: Number($("radarProtocol").value),
        stale_after_seconds: Number($("radarStale").value),
        camera_hold_seconds: Number($("radarHold").value),
        radars,
      };
    }

    async function saveRadarConfig() {
      if (!radarConfig) {
        showToast("Chưa tải được radar_gateway hiện tại.");
        return;
      }
      if (isStaticPreview) {
        radarConfig = collectRadarConfig();
        radarConfigDirty = false;
        updateRadarRestartBanner();
        showToast("DEMO: đã áp dụng radar_gateway trong RAM; không lưu file.");
        return;
      }
      try {
        const payload = collectRadarConfig();
        const res = await fetch("/api/radar/config", {
          method: "PUT",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ config: payload }),
        });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || "Lưu thất bại");
        radarConfig = data.config;
        lastRadarSaveResult = { restart_required: data.restart_required };
        radarConfigDirty = false;
        fillRadarForm(radarConfig);
        renderRadarCards(radarConfig, radarStatus);
        updateRadarRestartBanner();
        showToast(data.message || "Đã lưu radar_gateway.");
      } catch (err) {
        showToast(`Lỗi lưu radar_gateway: ${err.message}`, 5000);
      }
    }

    async function testRadarGateway() {
      if (isStaticPreview) {
        showTestResult({ ok: true, demo: true, message: "DEMO: giả lập Gateway đã sẵn sàng." });
        return;
      }
      const device = $("radarDevice").value.trim();
      if (!device) {
        showTestResult({ ok: false, error: "Hãy nhập đường dẫn Gateway trước khi kiểm tra." });
        return;
      }
      showTestResult({ running: true });
      try {
        const res = await fetch("/api/radar/test", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ device, baudrate: Number($("radarBaudrate").value), duration: 2.0 }),
        });
        const data = await res.json();
        showTestResult(data);
      } catch (err) {
        showTestResult({ ok: false, error: `Không kiểm tra được Gateway: ${err.message}` });
      }
    }

    function showTestResult(report) {
      const host = $("radarTestResult");
      if (!host) return;
      host.hidden = false;
      if (report.running) {
        host.className = "radar-test-result";
        host.textContent = "Đang mở Gateway UART và chờ gói JSON v2…";
        return;
      }
      host.className = `radar-test-result ${report.ok ? "ok" : "bad"}`;
      if (report.ok) {
        host.textContent = JSON.stringify(report, null, 2);
      } else {
        const cat = report.category ? `[${report.category}] ` : "";
        host.textContent = `${cat}${report.error || "Không rõ nguyên nhân"}`;
      }
    }

    const RADAR_PREVIEW_CONFIG = {
      enabled: true,
      device: "/dev/serial/by-id/usb-ESP32_Gateway",
      baudrate: 115200,
      protocol_version: 2,
      stale_after_seconds: 0.6,
      camera_hold_seconds: 1.2,
      radars: {
        left_front: { enabled: true, camera: "left_1", warning_distance_m: 5.0 },
        left_rear: { enabled: true, camera: "left_2", warning_distance_m: 5.0 },
        right_front: { enabled: true, camera: "right_1", warning_distance_m: 5.0 },
        right_rear: { enabled: true, camera: "right_2", warning_distance_m: 5.0 },
      },
      cameras_available: ["left_1", "left_2", "right_1", "right_2"],
    };

    const RADAR_PREVIEW_STATUS = {
      gateway_online: true,
      remote_online: true,
      gateway_boot_id: "boot-aaaa",
      remote_boot_id: "boot-bbbb",
      last_seq: 4218,
      packet_rate_hz: 20.0,
      packet_loss: 0.0,
      json_errors: 0,
      crc_errors: 0,
      packet_age_ms: 24,
      error: "",
      radars: {
        left_front: { detected: true, distance_m: 3.4, moving_energy: 80, stationary_energy: 12, valid: true, stale: false, age_ms: 24, error: "" },
        left_rear: { detected: false, distance_m: null, moving_energy: 4, stationary_energy: 0, valid: true, stale: false, age_ms: 22, error: "" },
        right_front: { detected: false, distance_m: null, moving_energy: 0, stationary_energy: 0, valid: true, stale: false, age_ms: 23, error: "" },
        right_rear: { detected: false, distance_m: null, moving_energy: 0, stationary_energy: 0, valid: true, stale: false, age_ms: 25, error: "" },
      },
    };
  });
})();
