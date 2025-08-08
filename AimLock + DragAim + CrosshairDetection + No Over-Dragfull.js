let body = $response.body;

// Nếu là JSON thì parse thử
try { body = JSON.parse($response.body); } catch (e) {}
// == Utility Functions ==
const Utils = {
  clamp: (value, min, max) => Math.max(min, Math.min(max, value)),
  lerp: (a, b, t) => a + (b - a) * Utils.clamp(t, 0, 1),
  distance2D: (a, b) => Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2),
  distance3D: (a, b) => Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2),
  isInFOV: (target, player, fov = 90) => {
    const angle = Math.atan2(target.z - player.z, target.x - player.x) * 180 / Math.PI;
    const playerAngle = player.rotation || 0;
    const diff = Math.abs(angle - playerAngle);
    return Math.min(diff, 360 - diff) <= fov / 2;
  }
};

// == Vector3 Enhanced ==

class Vector3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  // ===== In-place operations =====
  addInPlace(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }

  subtractInPlace(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }

  multiplyScalarInPlace(s) {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
  }

  normalizeInPlace() {
    const lenSq = this.lengthSquared();
    if (lenSq > 0) {
      const inv = 1 / Math.sqrt(lenSq);
      this.x *= inv;
      this.y *= inv;
      this.z *= inv;
    }
    return this;
  }

  // ===== Return new vector operations =====
  add(v) {
    return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  subtract(v) {
    return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  multiplyScalar(s) {
    return new Vector3(this.x * s, this.y * s, this.z * s);
  }

  normalize() {
    const lenSq = this.lengthSquared();
    return lenSq > 0 ? this.multiplyScalar(1 / Math.sqrt(lenSq)) : new Vector3();
  }

  lerp(target, t) {
    return new Vector3(
      Utils.lerp(this.x, target.x, t),
      Utils.lerp(this.y, target.y, t),
      Utils.lerp(this.z, target.z, t)
    );
  }

  // ===== Dot & Cross Product =====
  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  cross(v) {
    return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    );
  }

  // ===== Magnitude =====
  lengthSquared() {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }

  length() {
    return Math.sqrt(this.lengthSquared());
  }

  // ===== Utility =====
  set(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  clone() {
    return new Vector3(this.x, this.y, this.z);
  }

  // ===== Static constants =====
  static zero() {
    return new Vector3(0, 0, 0);
  }

  static one() {
    return new Vector3(1, 1, 1);
  }

  static up() {
    return new Vector3(0, 1, 0);
  }

  static forward() {
    return new Vector3(0, 0, 1);
  }
}

// == Advanced Kalman Filter ==

class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001, processNoise = 0.001) {
    this.R = R;               // Độ ồn đo lường (measurement noise)
    this.Q = Q;               // Độ ồn quá trình (process noise covariance)
    this.processNoise = processNoise; // noise tỉ lệ thời gian
    this.A = 1;               // Hệ số chuyển trạng thái
    this.C = 1;               // Hệ số đo lường
    this.x = 0;               // Giá trị ước lượng hiện tại
    this.cov = 1.0;           // Độ biến thiên ước lượng
    this.velocity = 0;        // Vận tốc (đổi giá trị)
    this.isInitialized = false;
    this.lastValue = 0;
    this.lastTime = Date.now();
  }

  filter(z) {
    const now = Date.now();
    const dt = (now - this.lastTime) / 1000;

    if (!this.isInitialized) {
      this.x = z;
      this.cov = this.R;
      this.isInitialized = true;
      this.lastValue = z;
      this.lastTime = now;
      return this.x;
    }

    // Tính vận tốc dựa trên delta thời gian và giá trị
    this.velocity = (z - this.lastValue) / Math.max(dt, 0.001);

    // Dự đoán trạng thái kế tiếp dựa trên vận tốc và delta thời gian
    const predX = this.A * this.x + this.velocity * dt;

    // Dự đoán độ biến thiên (covariance)
    const predCov = this.cov + this.Q + this.processNoise * dt;

    // Tính hệ số Kalman gain
    const K = predCov / (predCov + this.R);

    // Cập nhật giá trị ước lượng với đo lường mới
    this.x = predX + K * (z - this.C * predX);

    // Cập nhật độ biến thiên
    this.cov = predCov - K * this.C * predCov;

    // Lưu giá trị và thời gian cho lần lọc kế tiếp
    this.lastValue = z;
    this.lastTime = now;

    return this.x;
  }

  // Dự đoán giá trị trong tương lai (giây)
  predict(futureTime = 0.1) {
    return this.x + this.velocity * futureTime;
  }

  reset() {
    this.isInitialized = false;
    this.x = 0;
    this.cov = 1.0;
    this.velocity = 0;
    this.lastTime = Date.now();
  }
}

// == Enhanced Crosshair Detector ==

const CrosshairDetector = {
  last: "none",
  lastUpdate: 0,
  updateInterval: 8, // ms, lấy nhỏ hơn để update nhanh hơn
  states: ["none", "white", "red", "head", "body"],

  update() {
    const now = Date.now();
    if (now - this.lastUpdate < this.updateInterval) return;
    this.lastUpdate = now;

    try {
      if (typeof GameAPI !== "undefined") {
        this.last = GameAPI.crosshairState || GameAPI.getCrosshairColor() || "none";
      } else if (typeof window !== "undefined" && window.gameAPI) {
        this.last = window.gameAPI.crosshairState || "none";
      } else {
        // Fallback giả lập khi không có API
        this.last = Math.random() > 0.8 ? "red" : "none";
      }
    } catch (e) {
      this.last = "none";
    }
  },

  isRed() {
    this.update();
    return this.last === "red";
  },

  isHeadshot() {
    this.update();
    return this.last === "head";
  },

  isTargeting() {
    this.update();
    return ["red", "head", "body"].includes(this.last);
  },

  getState() {
    this.update();
    return this.last;
  }
};

// == Enhanced Aim System ==

class AimLockDragStable {
  constructor(config = {}) {
    this.config = {
      smoothness: config.smoothness ?? 0.15,
      predictionTime: config.predictionTime ?? 0.1,
      maxDistance: config.maxDistance ?? 100,
      fov: config.fov ?? 120,
      aimSpeed: config.aimSpeed ?? 0.8,
      headPreference: config.headPreference ?? 1.5,
      alwaysActive: config.alwaysActive ?? false,
      debug: config.debug ?? false,
      ...config
    };

    this.kalman = {
      x: new KalmanFilter(0.02, 0.00001, 0.001),
      y: new KalmanFilter(0.02, 0.00001, 0.001),
      z: new KalmanFilter(0.02, 0.00001, 0.001)
    };

    this.currentAim = new Vector3();
    this.trackedHead = new Vector3();
    this.predictedPos = new Vector3();
    this.compensated = new Vector3();
    this.lastTargetPos = new Vector3();
    this.targetVelocity = new Vector3();
    this.lockStartTime = 0;
    this.isLocked = false;
  }

  updateBoneHeadTracking(pos) {
    this.trackedHead.set(
      this.kalman.x.filter(pos.x),
      this.kalman.y.filter(pos.y),
      this.kalman.z.filter(pos.z)
    );

    this.predictedPos.set(
      this.kalman.x.predict(this.config.predictionTime),
      this.kalman.y.predict(this.config.predictionTime),
      this.kalman.z.predict(this.config.predictionTime)
    );

    return this.trackedHead;
  }

  tick(boneHead, recoil = Vector3.zero(), player = null) {
    const shouldAim = CrosshairDetector.isTargeting() || this.config.alwaysActive;
    if (!shouldAim && !this.isLocked) return;

    if (player && Utils.distance3D(boneHead, player) > this.config.maxDistance) {
      this.reset();
      return;
    }

    const tracked = this.updateBoneHeadTracking(boneHead);
    // Nếu velocity lớn thì dùng predictedPos, không thì dùng tracked
    const targetPos = (this.kalman.x.velocity && Math.abs(this.kalman.x.velocity) > 0.1)
      ? this.predictedPos
      : tracked;

    this.compensated.set(
      targetPos.x - recoil.x,
      targetPos.y - recoil.y,
      targetPos.z - recoil.z
    );

    this.dragTowardTarget(this.compensated);
    this.setCrosshair(this.currentAim);

    this.isLocked = true;
    if (this.lockStartTime === 0) {
      this.lockStartTime = Date.now();
    }
  }

  dragTowardTarget(target) {
    const t = this.config.smoothness * this.config.aimSpeed;
    // lerp method, Vector3.lerp(target, t) trả về Vector3 mới
    // gán cho currentAim mới để tránh mất tham chiếu
    this.currentAim = this.currentAim.lerp(target, t);
    return this.currentAim;
  }

  setCrosshair(vec3) {
    try {
      if (typeof GameAPI !== "undefined" && GameAPI.setAim) {
        GameAPI.setAim(vec3.x, vec3.y, vec3.z);
      } else if (typeof window !== "undefined" && window.gameAPI) {
        window.gameAPI.setAim?.(vec3.x, vec3.y, vec3.z);
      } else if (typeof setAim !== "undefined") {
        setAim(vec3.x, vec3.y, vec3.z);
      }
    } catch (e) {
      // im lặng không làm gì nếu lỗi
    }

    if (this.config.debug) {
      console.log(`🎯 Aim: ${vec3.x.toFixed(3)}, ${vec3.y.toFixed(3)}, ${vec3.z.toFixed(3)}`);
    }
  }

  reset() {
    this.kalman.x.reset();
    this.kalman.y.reset();
    this.kalman.z.reset();
    this.currentAim.set(0, 0, 0);
    this.isLocked = false;
    this.lockStartTime = 0;
  }

  setSmoothing(value) {
    this.config.smoothness = Utils.clamp(value, 0.01, 1.0);
  }

  setPrediction(time) {
    this.config.predictionTime = Utils.clamp(time, 0, 0.5);
  }

  setFOV(degrees) {
    this.config.fov = Utils.clamp(degrees, 180, 360);
  }
}

// == Smart Target Selector ==
class TargetSelector {
  constructor(config = {}) {
    this.config = {
      maxDistance: config.maxDistance || 120,
      fov: config.fov || 140,
      allowThroughWall: config.allowThroughWall || false,
      headPriority: config.headPriority || 1.5,
      updateRate: config.updateRate || 30,
      ...config
    };

    this.lastUpdate = 0;
    this.cachedTargets = [];
  }

  update(enemies, player) {
    const now = Date.now();
    if (now - this.lastUpdate < this.config.updateRate) return;
    this.lastUpdate = now;

    this.cachedTargets = enemies
      .filter(e => this.validateEnemy(e, player))
      .map(e => {
        const head = e.bone?.head || e.position || e;
        const distance = Utils.distance3D(player, head);
        const angle = this.getAngleTo(player, head);
        const threat = this.calculateThreat(e, distance, angle);
        return { enemy: e, head, distance, angle, threat };
      })
      .sort((a, b) => a.threat - b.threat);
  }

  getBestTarget() {
    return this.cachedTargets.length > 0 ? this.cachedTargets[0] : null;
  }

  validateEnemy(enemy, player) {
    if (!enemy || enemy.health <= 0 || enemy.id === player.id) return false;
    const head = enemy.bone?.head || enemy.position || enemy;
    const dist = Utils.distance3D(player, head);
    if (dist > this.config.maxDistance) return false;

    const angle = this.getAngleTo(player, head);
    if (angle > this.config.fov) return false;

    if (!this.config.allowThroughWall && enemy.behindWall) return false;

    return true;
  }

  getAngleTo(p1, p2) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const dz = p2.z - p1.z;
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (len === 0) return 0;
    const dot = dx / len; // simplified: assume facing forward
    return Math.acos(dot) * (180 / Math.PI);
  }

  calculateThreat(enemy, distance, angle) {
    let threat = distance;
    if (angle < 10) threat *= 0.7;
    if (enemy.isAimingAtPlayer) threat *= 0.5;
    if (enemy.bone?.head) threat *= this.config.headPriority;
    return threat;
  }
}

// == Main Game Loop ==

class GameLoop {
  constructor(config = {}) {
    this.config = {
      interval: config.interval || 8, // khoảng 60 FPS mặc định
      fps: config.fps || 120,
      autoFire: config.autoFire ?? true,
      recoilCompensation: config.recoilCompensation ?? true,
      lockOnHead: config.lockOnHead ?? true,
      alwaysActive: config.alwaysActive ?? false,
      ...config
    };

    this.running = false;
    this.player = new Vector3(0, 0, 0);
    this.recoil = new Vector3(0, 0, 0);
    this.enemies = [];
    this.aimEngine = new AimLockDragStable(config);
    this.selector = new TargetSelector(config);
  }

  setEnemies(enemyList) {
    this.enemies = enemyList;
  }

  setPlayerPosition(pos) {
    this.player.set(pos.x, pos.y, pos.z);
  }

  setRecoil(vec) {
    if (!this.config.recoilCompensation) return;
    this.recoil.set(vec.x, vec.y, vec.z);
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.loop();
  }

  stop() {
    this.running = false;
  }

  loop() {
    if (!this.running) return;

    try {
      this.selector.update(this.enemies, this.player);
      const best = this.selector.getBestTarget();
      if (best) {
        const boneHead = this.config.lockOnHead
          ? (best.enemy.bone?.head || best.head)
          : (best.enemy.position || best.head);

        this.aimEngine.tick(boneHead, this.recoil, this.player);

        if (this.config.autoFire && CrosshairDetector.isRed()) {
          this.fire();
        }
      } else {
        this.aimEngine.reset();
      }
    } catch (err) {
      console.warn("⚠️ GameLoop Error:", err);
    }

    setTimeout(() => this.loop(), this.config.interval);
  }

  fire() {
    try {
      if (typeof GameAPI !== "undefined" && GameAPI.fire) {
        GameAPI.fire();
      } else if (typeof window !== "undefined" && window.gameAPI) {
        window.gameAPI.fire?.();
      } else if (typeof fire !== "undefined") {
        fire();
      }
    } catch {
      // silent fail
    }
  }
}

// == Performance & Debug Monitor ==
class PerformanceMonitor {
  constructor(updateInterval = 1000) {
    this.updateInterval = updateInterval;
    this.lastTime = Date.now();
    this.frameCount = 0;
    this.fps = 0;
    this.avgTickTime = 0;
    this.tickTimes = [];
  }

  startFrame() {
    this.startTick = Date.now();
  }

  endFrame() {
    const now = Date.now();
    const tickTime = now - this.startTick;
    this.tickTimes.push(tickTime);
    if (this.tickTimes.length > 100) this.tickTimes.shift();

    this.frameCount++;
    if (now - this.lastTime >= this.updateInterval) {
      this.fps = (this.frameCount * 1000) / (now - this.lastTime);
      this.avgTickTime = this.tickTimes.reduce((a, b) => a + b, 0) / this.tickTimes.length;
      this.frameCount = 0;
      this.lastTime = now;
      this.log();
    }
  }

  log() {
    console.log(`📊 FPS: ${this.fps.toFixed(1)} | Avg Tick: ${this.avgTickTime.toFixed(2)}ms`);
  }

  getFPS() {
    return this.fps;
  }

  getAvgTick() {
    return this.avgTickTime;
  }
}

// == Vector3 tối ưu ==

// == Kalman Filter ==

// == Crosshair Detector ==

// == Drag đến Head ==
const tempVector = new Vector3();
function dragTowardBoneHead(currentAim, boneHead, maxStep = 0.05) {
  tempVector.set(boneHead.x - currentAim.x, boneHead.y - currentAim.y, boneHead.z - currentAim.z);
  const distSq = tempVector.lengthSquared();
  const maxStepSq = maxStep * maxStep;

  if (distSq <= maxStepSq) {
    return currentAim.set(boneHead.x, boneHead.y, boneHead.z);
  }

  const invDist = maxStep / Math.sqrt(distSq);
  currentAim.x += tempVector.x * invDist;
  currentAim.y += tempVector.y * invDist;
  currentAim.z += tempVector.z * invDist;

  return currentAim;
}
function dragTowardBoneHead(currentAim, boneHead) {
  return currentAim.set(boneHead.x, boneHead.y, boneHead.z);
}
class Quaternion {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x; this.y = y; this.z = z; this.w = w;
  }

  rotateVector(v) {
    const x = v.x, y = v.y, z = v.z;
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;

    const ix =  qw * x + qy * z - qz * y;
    const iy =  qw * y + qz * x - qx * z;
    const iz =  qw * z + qx * y - qy * x;
    const iw = -qx * x - qy * y - qz * z;

    return new Vector3(
      ix * qw + iw * -qx + iy * -qz - iz * -qy,
      iy * qw + iw * -qy + iz * -qx - ix * -qz,
      iz * qw + iw * -qz + ix * -qy - iy * -qx
    );
  }
}
// == AimLock Engine ==

// === Drag ngay lập tức đến Head ===
function dragTowardBoneHead(currentAim, boneHead) {
  return currentAim.set(boneHead.x, boneHead.y, boneHead.z);
}

// === Game Loop ===

// === Lock Target Logic ===
function findNearestEnemy(enemies, player) {
  function getBoneHeadTopPosition(enemy, baseOffset = { x: 0, y: 0.06, z: 0 }, headHeight = 0.1) {
    const pos = {
      x: enemy.x + baseOffset.x,
      y: enemy.y + baseOffset.y,
      z: enemy.z + baseOffset.z
    };

    const rx = enemy.rx ?? 0, ry = enemy.ry ?? 0, rz = enemy.rz ?? 0, rw = enemy.rw ?? 1;
    const sx = enemy.sx ?? 1, sy = enemy.sy ?? 1, sz = enemy.sz ?? 1;

    const qx = rx, qy = ry, qz = rz, qw = rw;

    // Rotate vector (0,1,0) by quaternion
    const up = { x: 0, y: 1, z: 0 };
    const ix =  qw * up.x + qy * up.z - qz * up.y;
    const iy =  qw * up.y + qz * up.x - qx * up.z;
    const iz =  qw * up.z + qx * up.y - qy * up.x;
    const iw = -qx * up.x - qy * up.y - qz * up.z;

    const rotatedUp = {
      x: (ix * qw + iw * -qx + iy * -qz - iz * -qy) * headHeight * sy,
      y: (iy * qw + iw * -qy + iz * -qx - ix * -qz) * headHeight * sy,
      z: (iz * qw + iw * -qz + ix * -qy - iy * -qx) * headHeight * sy
    };

    return {
      x: pos.x + rotatedUp.x,
      y: pos.y + rotatedUp.y,
      z: pos.z + rotatedUp.z
    };
  }

  let minDist = Infinity;
  let closest = null;
  for (const enemy of enemies) {
    if (!enemy || enemy.health <= 0) continue;

    // Tính vị trí đỉnh đầu bone head
    const head = getBoneHeadTopPosition(enemy);

    const dx = head.x - player.x;
    const dy = head.y - player.y;
    const dz = head.z - player.z;
    const distSq = dx * dx + dy * dy + dz * dz;

    if (distSq < minDist) {
      minDist = distSq;
      closest = enemy;
    }
  }
  return closest;
}

// === Khởi động hệ thống ===
const headOffset = new Vector3(-0.04089227, 0.00907892, 0.02748467);
const recoil = new Vector3(0, 0, 0);
const aimSystem = new AimLockDragStable();
const loop = new GameLoop();

let lockedTarget = null;

loop.start(() => {
  if (typeof enemies === "undefined" || typeof localPlayer === "undefined") return;

  // Nếu mục tiêu cũ chết hoặc không còn
  if (!lockedTarget || lockedTarget.health <= 0) {
    lockedTarget = findNearestEnemy(enemies, localPlayer);
    aimSystem.reset(); // reset kalman
  }

  if (!lockedTarget) return;

  // Tính tọa độ head
  // Tính tọa độ đầu (đã giới hạn đỉnh đầu thật)
const boneHeadPos = getBoneHeadTopPosition(lockedTarget);
const boneHead = new Vector3(boneHeadPos.x, boneHeadPos.y, boneHeadPos.z);

  aimSystem.tick(boneHead, recoil);
});
if (typeof body === "object") {
  $done({ body: JSON.stringify(body) });
} else {
  $done({ body });
}
