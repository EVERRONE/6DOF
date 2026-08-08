import { createServer } from 'node:http';
import { pathToFileURL } from 'node:url';
import { loadConfig } from './config.js';
import { createBrokerState } from './state.js';
import { createExecutorLink } from './executorLink.js';
import { createHttpApi } from './httpApi.js';

const log = (message) => {
  console.log(`[bridge ${new Date().toISOString()}] ${message}`);
};

export function startBridge(env = process.env) {
  const config = loadConfig(env);
  const state = createBrokerState({ auditLimit: config.auditLimit });
  const server = createServer();

  const executorLink = createExecutorLink({
    server,
    token: config.token,
    state,
    commandTimeoutMs: config.commandTimeoutMs,
    log
  });

  server.on('request', createHttpApi({ config, state, executorLink, log }));

  return new Promise((resolve) => {
    server.listen(config.port, config.host, () => {
      const address = server.address();
      log(`listening on ${config.host}:${address.port}`);
      log(`  agent API   http://127.0.0.1:${address.port}/api/status`);
      log(`  executor    ws://<this-host>:${address.port}/executor?token=…`);

      if (config.generatedToken) {
        log('');
        log('  No BRIDGE_TOKEN was set, so one was generated for this run:');
        log(`    ${config.token}`);
        log('  Set BRIDGE_TOKEN in the environment to keep it stable across restarts.');
        log('');
      }

      resolve({ server, config, state, executorLink });
    });
  });
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  startBridge().catch((err) => {
    console.error('Failed to start bridge:', err);
    process.exit(1);
  });
}
