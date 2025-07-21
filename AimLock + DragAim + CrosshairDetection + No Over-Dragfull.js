// == Vector3 tối ưu ==
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }

  addInPlace(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  subtractInPlace(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
  multiplyScalarInPlace(s) { this.x *= s; this.y *= s; this.z *= s; return this; }

  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }

  lengthSquared() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  length() { return Math.sqrt(this.lengthSquared()); }

  normalize() {
    const lenSq = this.lengthSquared();
    return lenSq > 0 ? this.multiplyScalar(1 / Math.sqrt(lenSq)) : new Vector3();
  }

  normalizeInPlace() {
    const lenSq = this.lengthSquared();
    if (lenSq > 0) {
      const inv = 1 / Math.sqrt(lenSq);
      this.x *= inv; this.y *= inv; this.z *= inv;
    }
    return this;
  }

  set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

// == Kalman Filter ==
class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q;
    this.A = 1; this.C = 1;
    this.x = 0; this.cov = 1.0;
    this.isInitialized = false;
  }

  filter(z) {
    if (!this.isInitialized) {
      this.x = z;
      this.cov = this.R;
      this.isInitialized = true;
      return this.x;
    }

    const predX = this.A * this.x;
    const predCov = this.cov + this.Q;
    const K = predCov / (predCov + this.R);
    this.x = predX + K * (z - this.C * predX);
    this.cov = predCov - K * this.C * predCov;
    return this.x;
  }

  reset() {
    this.isInitialized = false;
    this.x = 0;
    this.cov = 1.0;
  }
}

// == Crosshair Detector ==
const CrosshairDetector = {
  last: "none",
  lastUpdate: 0,
  updateInterval: 16,

  update() {
    const now = Date.now();
    if (now - this.lastUpdate < this.updateInterval) return;
    this.lastUpdate = now;

    try {
      this.last = (typeof GameAPI !== "undefined" && GameAPI.crosshairState)
        ? GameAPI.crosshairState
        : (Math.random() > 0.8 ? "red" : "none"); // giả lập nếu GameAPI không tồn tại
    } catch {
      this.last = "none";
    }
  },

  isRed() {
    this.update();
    return this.last === "red";
  },

  isHeadshot() {
    this.update();
    return this.last === "head"; // thêm headshot detection
  }
};

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
class AimLockDragStable {
  constructor() {
    this.kalman = {
      x: new KalmanFilter(0.03, 0.00001),
      y: new KalmanFilter(0.03, 0.00001),
      z: new KalmanFilter(0.03, 0.00001)
    };
    this.currentAim = new Vector3();
    this.trackedHead = new Vector3();
    this.compensated = new Vector3();
  }

  updateBoneHeadTracking(pos) {
    this.trackedHead.set(
      this.kalman.x.filter(pos.x),
      this.kalman.y.filter(pos.y),
      this.kalman.z.filter(pos.z)
    );
    return this.trackedHead;
  }

  tick(boneHead, recoil = Vector3.zero()) {
    // Luôn lock bất kể crosshair màu gì (nếu muốn lọc thêm thì bật dòng dưới)
    // if (!CrosshairDetector.isRed() && !CrosshairDetector.isHeadshot()) return;

    const tracked = this.updateBoneHeadTracking(boneHead);
    this.compensated.set(
      tracked.x - recoil.x,
      tracked.y - recoil.y,
      tracked.z - recoil.z
    );
    dragTowardBoneHead(this.currentAim, this.compensated);
    this.setCrosshair(this.currentAim);
  }

  setCrosshair(vec3) {
    if (typeof GameAPI !== "undefined" && GameAPI.setAim) {
      GameAPI.setAim(vec3.x, vec3.y, vec3.z);
    }
    console.log(`🎯 Aimed at: ${vec3.x.toFixed(2)}, ${vec3.y.toFixed(2)}, ${vec3.z.toFixed(2)}`);
  }

  reset() {
    this.kalman.x.reset();
    this.kalman.y.reset();
    this.kalman.z.reset();
    this.currentAim.set(0, 0, 0);
  }
}

// === Drag ngay lập tức đến Head ===
function dragTowardBoneHead(currentAim, boneHead) {
  return currentAim.set(boneHead.x, boneHead.y, boneHead.z);
}

// === Game Loop ===
class GameLoop {
  constructor() {
    this.interval = null;
    this.fps = 120;
  }
  start(callback) {
    if (this.interval) return;
    const ms = 1000 / this.fps;
    this.interval = setInterval(callback, ms);
  }
  stop() {
    if (this.interval) {
      clearInterval(this.interval);
      this.interval = null;
    }
  }
}

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
const boneHead = getBoneHeadTopPosition(lockedTarget);

  aimSystem.tick(boneHead, recoil);
});
 
