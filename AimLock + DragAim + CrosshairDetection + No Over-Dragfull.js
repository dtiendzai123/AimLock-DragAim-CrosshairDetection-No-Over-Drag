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

// == AimLock Engine ==
class AimLockDragStable {
  constructor() {
    this.kalman = {
      x: new KalmanFilter(0.03, 0.00001),
      y: new KalmanFilter(0.03, 0.00001),
      z: new KalmanFilter(0.03, 0.00001)
    };

    this.prevPos = new Vector3();
    this.velocity = new Vector3();
    this.currentAim = new Vector3();
    this.trackedHead = new Vector3();
    this.compensated = new Vector3();

    this.lastUpdate = Date.now();
    this.isFirstUpdate = true;

    this.adaptiveStep = 0.05;
    this.minStep = 0.01;
    this.maxStep = 0.1;
    this.velocityInfluence = 0.3;
  }

  updateBoneHeadTracking(pos) {
    const now = Date.now();
    const dt = (now - this.lastUpdate) / 1000;

    if (!this.isFirstUpdate && dt > 0 && dt < 0.1) {
      this.velocity.set(
        (pos.x - this.prevPos.x) / dt,
        (pos.y - this.prevPos.y) / dt,
        (pos.z - this.prevPos.z) / dt
      );

      const velocityMag = this.velocity.length();
      this.adaptiveStep = Math.max(this.minStep, Math.min(this.maxStep, this.minStep + velocityMag * this.velocityInfluence));
    }

    this.prevPos.set(pos.x, pos.y, pos.z);
    this.lastUpdate = now;
    this.isFirstUpdate = false;

    this.trackedHead.set(
      this.kalman.x.filter(pos.x),
      this.kalman.y.filter(pos.y),
      this.kalman.z.filter(pos.z)
    );

    return this.trackedHead;
  }

  tick(boneHead, recoil = Vector3.zero()) {
    if (!CrosshairDetector.isRed() && !CrosshairDetector.isHeadshot()) return;

    const trackedHead = this.updateBoneHeadTracking(boneHead);

    this.compensated.set(
      trackedHead.x - recoil.x,
      trackedHead.y - recoil.y,
      trackedHead.z - recoil.z
    );

    dragTowardBoneHead(this.currentAim, this.compensated, this.adaptiveStep);
    this.setCrosshair(this.currentAim);
  }

  setCrosshair(vec3) {
    console.log(`🎯 AimLock: ${vec3.x.toFixed(4)}, ${vec3.y.toFixed(4)}, ${vec3.z.toFixed(4)}`);
    if (typeof GameAPI !== "undefined" && GameAPI.setAim) {
      GameAPI.setAim(vec3.x, vec3.y, vec3.z);
    }
  }

  reset() {
    this.kalman.x.reset();
    this.kalman.y.reset();
    this.kalman.z.reset();
    this.isFirstUpdate = true;
    this.currentAim.set(0, 0, 0);
    this.velocity.set(0, 0, 0);
  }
}

// == Game Loop (setInterval fallback for Shadowrocket) ==
class GameLoop {
  constructor() {
    this.interval = null;
    this.fps = 120;
  }

  start(callback) {
    if (this.interval) return;
    const frameTime = 1000 / this.fps;
    this.interval = setInterval(callback, frameTime);
  }

  stop() {
    if (this.interval) {
      clearInterval(this.interval);
      this.interval = null;
    }
  }
}

// == Khởi chạy ==
const boneHead = new Vector3(-0.0456970781, -0.004478302, -0.0200432576);
const recoil = new Vector3(0.0, 0.0, 0.0);

const aimSystem = new AimLockDragStable();
const loop = new GameLoop();

loop.start(() => {
  aimSystem.tick(boneHead, recoil);
});

// Dừng khi cần
// loop.stop();
