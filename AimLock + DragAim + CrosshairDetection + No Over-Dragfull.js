// == Vector3 + Kalman ==
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x**2 + this.y**2 + this.z**2); }
  normalize() {
    const len = this.length();
    return len > 0 ? this.multiplyScalar(1 / len) : new Vector3();
  }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q; this.A = 1; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }
  filter(z) {
    if (isNaN(this.x)) { this.x = z; this.cov = this.R; }
    else {
      const predX = this.A * this.x;
      const predCov = this.cov + this.Q;
      const K = predCov * this.C / (this.C * predCov * this.C + this.R);
      this.x = predX + K * (z - this.C * predX);
      this.cov = predCov - K * this.C * predCov;
    }
    return this.x;
  }
}

// == Crosshair Detection ==
const CrosshairDetector = {
  last: "none",
  update() {
    try {
      this.last = (typeof GameAPI !== "undefined" && GameAPI.crosshairState)
        ? GameAPI.crosshairState
        : (Math.random() > 0.8 ? "red" : "none");
    } catch {
      this.last = "none";
    }
  },
  isRed() {
    this.update();
    return this.last === "red";
  }
};

// == Drag Aim với giới hạn không vượt bone head ==
function dragTowardBoneHead(currentAim, boneHead, maxStep = 0.05) {
  const delta = boneHead.subtract(currentAim);
  const dist = delta.length();
  if (dist <= maxStep) return boneHead.clone();
  return currentAim.add(delta.normalize().multiplyScalar(maxStep));
}

// == AimLock Engine ==
class AimLockDragStable {
  constructor() {
    this.kalman = {
      x: new KalmanFilter(0.03, 0.00001),
      y: new KalmanFilter(0.03, 0.00001),
      z: new KalmanFilter(0.03, 0.00001)
    };
    this.prevPos = null;
    this.velocity = Vector3.zero();
    this.currentAim = Vector3.zero();
    this.lastUpdate = Date.now();
  }

  updateBoneHeadTracking(pos) {
    const now = Date.now();
    const dt = (now - this.lastUpdate) / 1000;
    if (this.prevPos && dt > 0) {
      this.velocity = pos.subtract(this.prevPos).multiplyScalar(1 / dt);
    }
    this.prevPos = pos.clone();
    this.lastUpdate = now;

    return new Vector3(
      this.kalman.x.filter(pos.x),
      this.kalman.y.filter(pos.y),
      this.kalman.z.filter(pos.z)
    );
  }

  tick(boneHead, recoil = Vector3.zero()) {
    if (!CrosshairDetector.isRed()) return;

    const trackedHead = this.updateBoneHeadTracking(boneHead);
    const compensated = trackedHead.subtract(recoil);
    const newAim = dragTowardBoneHead(this.currentAim, compensated);
    this.currentAim = newAim.clone();

    this.setCrosshair(newAim);
  }

  setCrosshair(vec3) {
    console.log("🎯 DragAim Lock:", vec3.x.toFixed(5), vec3.y.toFixed(5), vec3.z.toFixed(5));
    // GameAPI.setAim(vec3.x, vec3.y, vec3.z);
  }
}

// == Dữ liệu mô phỏng bone head ==
const boneHead = new Vector3(-0.0456970781, -0.004478302, -0.0200432576);
const recoil = new Vector3(0.003, -0.001, 0.002); // có thể cập nhật theo từng weapon

const aimSystem = new AimLockDragStable();

// == Loop 60fps ==
function runLoop() {
  aimSystem.tick(boneHead, recoil);
  setTimeout(runLoop, 16); // ~60FPS
}
runLoop();
