import { randomBytes } from 'node:crypto';

/**
 * Broker configuration, read once at startup.
 *
 * The token is mandatory in spirit: a local HTTP server is reachable from any
 * web page the user happens to visit, so an unauthenticated motion API on
 * localhost is a genuine hazard, not a theoretical one. If none is supplied we
 * generate one and print it rather than running open.
 */
export function loadConfig(env = process.env) {
  const port = Number.parseInt(env.BRIDGE_PORT ?? '8765', 10);

  // Hermes reaches the broker over loopback; the browser executor reaches it
  // across the LAN. Binding 0.0.0.0 is therefore the useful default, and the
  // token is what protects it.
  const host = env.BRIDGE_HOST ?? '0.0.0.0';

  const token = env.BRIDGE_TOKEN?.trim() || randomBytes(24).toString('base64url');
  const generatedToken = !env.BRIDGE_TOKEN?.trim();

  return {
    port: Number.isFinite(port) ? port : 8765,
    host,
    token,
    generatedToken,

    /** How long a read-only command may wait for the executor before we give up. */
    commandTimeoutMs: Number.parseInt(env.BRIDGE_COMMAND_TIMEOUT_MS ?? '20000', 10),

    /**
     * Motion needs far longer: planning alone can take up to 20 s, and then the
     * arm has to actually travel. Kept above the executor's own settle timeout
     * so the executor is the one that reports the outcome, not the broker.
     */
    motionTimeoutMs: Number.parseInt(env.BRIDGE_MOTION_TIMEOUT_MS ?? '120000', 10),

    /** Entries kept in the in-memory audit log. */
    auditLimit: Number.parseInt(env.BRIDGE_AUDIT_LIMIT ?? '200', 10)
  };
}
