// Trapezoidal Velocity Profile Generator
// Generates smooth acceleration/constant velocity/deceleration motion profiles

import { ProfileType } from './types';

/**
 * Trapezoidal Velocity Profile
 *
 * Generates a time-parameterized motion profile with:
 * 1. Acceleration phase (0 → maxVel)
 * 2. Constant velocity phase (maxVel)
 * 3. Deceleration phase (maxVel → 0)
 *
 * If distance is too short for full trapezoidal, falls back to triangular profile.
 */
export class VelocityProfile {
  private accelTime: number;
  private constTime: number;
  private decelTime: number;
  private totalTime: number;
  private peakVelocity: number;
  private profileType: ProfileType;
  private direction: number;

  constructor(
    private distance: number,
    private maxVelocity: number,
    private maxAcceleration: number
  ) {
    this.direction = Math.sign(distance);
    const absDist = Math.abs(distance);

    // Time to accelerate to max velocity
    const tAccel = maxVelocity / maxAcceleration;
    // Distance covered during acceleration + deceleration
    const accelDist = maxAcceleration * tAccel * tAccel; // 2 × (0.5 × a × t²)

    if (accelDist >= absDist) {
      // Triangular profile: can't reach max velocity
      this.profileType = 'triangular';
      this.peakVelocity = Math.sqrt(absDist * maxAcceleration);
      this.accelTime = this.peakVelocity / maxAcceleration;
      this.constTime = 0;
      this.decelTime = this.accelTime;
    } else {
      // Trapezoidal profile: reaches max velocity
      this.profileType = 'trapezoidal';
      this.peakVelocity = maxVelocity;
      this.accelTime = tAccel;
      this.constTime = (absDist - accelDist) / maxVelocity;
      this.decelTime = tAccel;
    }

    this.totalTime = this.accelTime + this.constTime + this.decelTime;
  }

  /**
   * Get interpolated position at time t
   * @param t - Time in seconds (0 to totalTime)
   * @returns Position (same units as distance)
   */
  getPosition(t: number): number {
    t = Math.max(0, Math.min(t, this.totalTime));

    if (t <= 0) return 0;
    if (t >= this.totalTime) return this.distance;

    let pos: number;

    if (t <= this.accelTime) {
      // Acceleration phase: x = 0.5 * a * t²
      const a = this.peakVelocity / this.accelTime;
      pos = 0.5 * a * t * t;
    } else if (t <= this.accelTime + this.constTime) {
      // Constant velocity phase
      const tConst = t - this.accelTime;
      const xAccel = 0.5 * this.peakVelocity * this.accelTime;
      pos = xAccel + this.peakVelocity * tConst;
    } else {
      // Deceleration phase
      const tDecel = t - this.accelTime - this.constTime;
      const xAccel = 0.5 * this.peakVelocity * this.accelTime;
      const xConst = this.peakVelocity * this.constTime;
      const a = this.peakVelocity / this.decelTime;
      pos = xAccel + xConst + this.peakVelocity * tDecel - 0.5 * a * tDecel * tDecel;
    }

    return this.direction * pos;
  }

  /**
   * Get velocity at time t
   * @param t - Time in seconds
   * @returns Velocity (units/s)
   */
  getVelocity(t: number): number {
    t = Math.max(0, Math.min(t, this.totalTime));

    if (t <= 0 || t >= this.totalTime) return 0;

    let vel: number;

    if (t <= this.accelTime) {
      const a = this.peakVelocity / this.accelTime;
      vel = a * t;
    } else if (t <= this.accelTime + this.constTime) {
      vel = this.peakVelocity;
    } else {
      const tDecel = t - this.accelTime - this.constTime;
      const a = this.peakVelocity / this.decelTime;
      vel = this.peakVelocity - a * tDecel;
    }

    return this.direction * vel;
  }

  /**
   * Get normalized progress at time t (0 to 1)
   */
  getProgress(t: number): number {
    if (Math.abs(this.distance) < 1e-10) return 1;
    return Math.abs(this.getPosition(t) / this.distance);
  }

  /**
   * Get total duration of the profile
   */
  getDuration(): number {
    return this.totalTime;
  }

  /**
   * Get the peak velocity achieved
   */
  getPeakVelocity(): number {
    return this.peakVelocity;
  }

  /**
   * Get the profile type (trapezoidal or triangular)
   */
  getProfileType(): ProfileType {
    return this.profileType;
  }

  /**
   * Sample the profile at regular intervals
   * @param hz - Sampling frequency in Hz
   * @returns Array of {time, position, velocity} samples
   */
  sample(hz: number): { time: number; position: number; velocity: number }[] {
    const dt = 1.0 / hz;
    const samples: { time: number; position: number; velocity: number }[] = [];

    for (let t = 0; t <= this.totalTime; t += dt) {
      samples.push({
        time: t,
        position: this.getPosition(t),
        velocity: this.getVelocity(t)
      });
    }

    // Ensure final point is included
    const last = samples[samples.length - 1];
    if (last && Math.abs(last.time - this.totalTime) > dt * 0.5) {
      samples.push({
        time: this.totalTime,
        position: this.getPosition(this.totalTime),
        velocity: 0
      });
    }

    return samples;
  }
}
