/**
 * In-memory broker state: the last telemetry the executor reported, and an
 * audit trail of every command that came in.
 *
 * Nothing here is persisted. The broker is a relay, not a source of truth —
 * the robot's actual state lives in the web app, and the arming decision lives
 * with the human at the machine.
 */
export function createBrokerState({ auditLimit = 200 } = {}) {
  let telemetry = null;
  let telemetryAt = null;
  let executor = null;
  const audit = [];

  return {
    setExecutor(info) {
      executor = info;
      if (!info) {
        // Stale telemetry from a disconnected executor is worse than none:
        // it would let the broker answer "motors on" about a robot it can no
        // longer see.
        telemetry = null;
        telemetryAt = null;
      }
    },

    getExecutor() {
      return executor;
    },

    setTelemetry(state) {
      telemetry = state;
      telemetryAt = Date.now();
    },

    getTelemetry() {
      return telemetry ? { state: telemetry, receivedAt: telemetryAt } : null;
    },

    /** True only when the executor has affirmatively said it is armed. */
    isArmed() {
      if (!telemetry?.armed) return false;
      const until = telemetry.armedUntil;
      if (typeof until !== 'number') return false;
      return until > Date.now();
    },

    recordAudit(entry) {
      audit.push({ at: new Date().toISOString(), ...entry });
      while (audit.length > auditLimit) audit.shift();
    },

    getAudit(limit = 50) {
      return audit.slice(-limit).reverse();
    }
  };
}
