#!/usr/bin/env node
/**
 * One-click launcher for the robot arm web app.
 *
 * Started by double-clicking start-robot-arm.bat / .command / .sh in the repo
 * root, or with `npm run launch` from robot-arm-control/.
 *
 * Order of operations:
 *   1. fast-forward the checkout to its upstream branch (never touches local work)
 *   2. reinstall dependencies, but only when package-lock.json actually changed
 *   3. start the CRA dev server on a free port
 *   4. open Chrome or Edge -- Web Serial does not exist in other browsers
 *
 * Flags:
 *   --no-update     skip the git fetch/fast-forward step (offline start)
 *   --no-open       start the server but do not open a browser
 *   --update-only   run steps 1-2 and exit without starting the server
 *   --port <n>      serve on a specific port instead of 3000
 */

import { spawn, spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import http from 'node:http';
import net from 'node:net';
import { basename, dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const APP_DIR = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const REPO_ROOT = resolve(APP_DIR, '..');
const LOCK_FILE = join(APP_DIR, 'package-lock.json');
const STAMP_FILE = join(APP_DIR, 'node_modules', '.launch-stamp.json');

const IS_WINDOWS = process.platform === 'win32';
const NPM = IS_WINDOWS ? 'npm.cmd' : 'npm';

const DEFAULT_PORT = 3000;
const PORT_SCAN_RANGE = 10;
const SERVER_READY_TIMEOUT_MS = 5 * 60 * 1000;

// ---------------------------------------------------------------------------
// logging
// ---------------------------------------------------------------------------

const step = (message) => console.log(`\n== ${message}`);
const info = (message) => console.log(`   ${message}`);
const ok = (message) => console.log(`   OK  ${message}`);
const warn = (message) => console.log(`   !   ${message}`);

function fail(message) {
  console.error(`\n   FAILED  ${message}\n`);
  process.exit(1);
}

// ---------------------------------------------------------------------------
// arguments
// ---------------------------------------------------------------------------

const args = process.argv.slice(2);

const flags = {
  skipUpdate: args.includes('--no-update') || process.env.ROBOT_ARM_SKIP_UPDATE === '1',
  skipOpen: args.includes('--no-open'),
  updateOnly: args.includes('--update-only'),
};

function resolvePort() {
  const index = args.indexOf('--port');
  const raw = index !== -1 ? args[index + 1] : process.env.PORT;
  const parsed = Number.parseInt(raw ?? '', 10);
  if (Number.isInteger(parsed) && parsed > 0 && parsed < 65536) return parsed;
  if (raw) warn(`Ignoring invalid port "${raw}", using ${DEFAULT_PORT}.`);
  return DEFAULT_PORT;
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

const sleep = (ms) => new Promise((done) => setTimeout(done, ms));

function hasCommand(command) {
  const probe = spawnSync(IS_WINDOWS ? 'where' : 'which', [command], { stdio: 'ignore' });
  return probe.status === 0;
}

function git(gitArgs, { timeout = 60_000 } = {}) {
  return spawnSync('git', gitArgs, { cwd: REPO_ROOT, encoding: 'utf8', timeout });
}

function gitOut(gitArgs, options) {
  const result = git(gitArgs, options);
  return result.status === 0 ? result.stdout.trim() : null;
}

// npm on Windows is a .cmd shim, which Node refuses to spawn without a shell.
function runNpm(npmArgs) {
  const result = spawnSync(NPM, npmArgs, {
    cwd: APP_DIR,
    stdio: 'inherit',
    shell: IS_WINDOWS,
  });
  return result.status ?? 1;
}

// ---------------------------------------------------------------------------
// step 1 -- pull updates
// ---------------------------------------------------------------------------

function updateCheckout() {
  step('Checking for updates');

  if (flags.skipUpdate) {
    info('Skipped (--no-update).');
    return;
  }
  if (!hasCommand('git')) {
    warn('git is not on PATH -- starting with the files as they are.');
    return;
  }
  if (gitOut(['rev-parse', '--is-inside-work-tree']) !== 'true') {
    warn('Not a git checkout -- nothing to update.');
    return;
  }

  const branch = gitOut(['rev-parse', '--abbrev-ref', 'HEAD']);
  if (!branch || branch === 'HEAD') {
    warn('Detached HEAD -- skipping the update to avoid surprises.');
    return;
  }

  const dirty = gitOut(['status', '--porcelain']);
  if (dirty) {
    warn(`Uncommitted changes on ${branch}, so nothing was pulled:`);
    const lines = dirty.split('\n');
    for (const line of lines.slice(0, 10)) info(`  ${line}`);
    if (lines.length > 10) info(`  ... and ${lines.length - 10} more`);
    warn('Commit or stash them, then start again to pick up new commits.');
    return;
  }

  const upstream = gitOut(['rev-parse', '--abbrev-ref', '--symbolic-full-name', '@{u}']);
  if (!upstream) {
    warn(`Branch ${branch} has no upstream branch -- nothing to pull.`);
    return;
  }

  info(`Fetching ${upstream} ...`);
  if (git(['fetch', '--quiet', '--prune'], { timeout: 120_000 }).status !== 0) {
    warn('Fetch failed (no network?) -- starting with the local version.');
    return;
  }

  const behind = gitOut(['rev-list', '--count', `HEAD..${upstream}`]);
  if (behind === '0') {
    ok(`Already up to date with ${upstream}.`);
    return;
  }

  info(`${behind} new commit(s) on ${upstream} -- fast-forwarding ...`);
  if (git(['merge', '--ff-only', upstream]).status !== 0) {
    warn(`Cannot fast-forward ${branch} onto ${upstream}: the branches diverged.`);
    warn('Sort it out manually, for example with: git pull --rebase');
    return;
  }

  ok(`Updated ${branch} to ${gitOut(['rev-parse', '--short', 'HEAD'])}.`);
}

// ---------------------------------------------------------------------------
// step 2 -- dependencies
// ---------------------------------------------------------------------------

function lockHash() {
  if (!existsSync(LOCK_FILE)) return null;
  return createHash('sha256').update(readFileSync(LOCK_FILE)).digest('hex');
}

function readStamp() {
  try {
    return JSON.parse(readFileSync(STAMP_FILE, 'utf8'));
  } catch {
    return null;
  }
}

function writeStamp(hash) {
  try {
    mkdirSync(dirname(STAMP_FILE), { recursive: true });
    writeFileSync(
      STAMP_FILE,
      `${JSON.stringify({ lockHash: hash, installedAt: new Date().toISOString() }, null, 2)}\n`
    );
  } catch {
    warn('Could not write the install stamp -- dependencies will be rechecked next start.');
  }
}

function lockfileMatchesHead() {
  if (!hasCommand('git') || gitOut(['rev-parse', '--is-inside-work-tree']) !== 'true') return false;
  const status = gitOut(['status', '--porcelain', '--', LOCK_FILE]);
  return status === '';
}

/**
 * npm sometimes rewrites package-lock.json while installing (it repairs missing
 * optional/peer entries, and that differs per npm version). A permanently
 * modified lockfile would make every later start report local changes and
 * refuse to pull, so put the committed version back.
 */
function restoreLockfileToHead() {
  if (git(['restore', '--', LOCK_FILE]).status === 0) return true;
  return git(['checkout', '--', LOCK_FILE]).status === 0;
}

function installDependencies() {
  step('Checking dependencies');

  const installed = existsSync(join(APP_DIR, 'node_modules', 'react-scripts'));
  const hash = lockHash();
  const stamp = readStamp();

  if (installed && !hash) {
    ok('No package-lock.json found -- keeping the current node_modules.');
    return;
  }
  if (installed && stamp?.lockHash === hash) {
    ok('Dependencies are up to date.');
    return;
  }

  const reason = !installed ? 'node_modules is missing' : 'package-lock.json changed';
  info(`Installing dependencies (${reason}). The first run can take a few minutes ...`);

  const lockWasClean = lockfileMatchesHead();

  // npm ci installs exactly what the lockfile pins, which is what a fresh pull
  // expects. npm install is only the fallback for a lockfile npm ci rejects.
  let status = hash ? runNpm(['ci']) : runNpm(['install']);
  if (status !== 0 && hash) {
    warn('npm ci failed -- retrying with npm install.');
    status = runNpm(['install']);
  }
  if (status !== 0) {
    fail('Installing dependencies failed. Scroll up for the npm error.');
  }

  // Only undo a lockfile edit this run caused, starting from a clean file --
  // never a change the user made themselves.
  if (lockWasClean && lockHash() !== hash && restoreLockfileToHead()) {
    info('npm rewrote package-lock.json; restored the committed version.');
  }

  writeStamp(lockHash());
  ok('Dependencies installed.');
}

// ---------------------------------------------------------------------------
// step 3 -- dev server
// ---------------------------------------------------------------------------

function isPortFree(port) {
  return new Promise((done) => {
    const server = net.createServer();
    server.once('error', () => done(false));
    server.once('listening', () => server.close(() => done(true)));
    server.listen(port, '0.0.0.0');
  });
}

async function findFreePort(start) {
  for (let port = start; port < start + PORT_SCAN_RANGE; port += 1) {
    if (await isPortFree(port)) return port;
  }
  return null;
}

function httpGet(port, path = '/', timeoutMs = 2000) {
  return new Promise((done) => {
    const request = http.get({ host: '127.0.0.1', port, path, timeout: timeoutMs }, (response) => {
      let body = '';
      response.setEncoding('utf8');
      response.on('data', (chunk) => {
        if (body.length < 65_536) body += chunk;
      });
      response.on('end', () => done({ status: response.statusCode ?? 0, body }));
    });
    request.on('timeout', () => {
      request.destroy();
      done(null);
    });
    request.on('error', () => done(null));
  });
}

// The CRA dev server always serves the index.html shell with the React root div.
const looksLikeOurApp = (response) =>
  response?.status === 200 && response.body.includes('id="root"');

async function waitForServer(port, child) {
  const deadline = Date.now() + SERVER_READY_TIMEOUT_MS;
  while (Date.now() < deadline) {
    if (child.exitCode !== null) return false;
    if (looksLikeOurApp(await httpGet(port))) return true;
    await sleep(500);
  }
  return false;
}

// ---------------------------------------------------------------------------
// step 4 -- browser
// ---------------------------------------------------------------------------

function chromiumCandidates() {
  if (IS_WINDOWS) {
    // Chrome is often a per-user install under LOCALAPPDATA, so check every
    // root for Chrome before falling back to Edge.
    const roots = [
      process.env.PROGRAMFILES ?? 'C:\\Program Files',
      process.env['PROGRAMFILES(X86)'] ?? 'C:\\Program Files (x86)',
      process.env.LOCALAPPDATA,
    ].filter(Boolean);
    return [
      ...roots.map((root) => join(root, 'Google', 'Chrome', 'Application', 'chrome.exe')),
      ...roots.map((root) => join(root, 'Microsoft', 'Edge', 'Application', 'msedge.exe')),
    ];
  }
  if (process.platform === 'darwin') {
    return [
      '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
      '/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge',
      '/Applications/Chromium.app/Contents/MacOS/Chromium',
    ];
  }
  return ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser', 'microsoft-edge'];
}

function findChromiumBrowser() {
  for (const candidate of chromiumCandidates()) {
    if (candidate.includes('/') || candidate.includes('\\')) {
      if (existsSync(candidate)) return candidate;
    } else if (hasCommand(candidate)) {
      return candidate;
    }
  }
  return null;
}

function openBrowser(url) {
  const browser = findChromiumBrowser();
  if (!browser) {
    warn('No Chrome, Edge or Chromium found. Web Serial only works in those browsers,');
    warn(`so open ${url} in one of them yourself.`);
    return;
  }

  const child = spawn(browser, [url], { detached: true, stdio: 'ignore' });
  child.on('error', () => warn(`Could not launch ${browser}. Open ${url} yourself.`));
  child.unref();
  ok(`Opened ${url} in ${basename(browser)}.`);
}

// ---------------------------------------------------------------------------
// shutdown
// ---------------------------------------------------------------------------

let devServer = null;
let stopping = false;

function stopDevServer() {
  if (stopping || !devServer || devServer.exitCode !== null) return;
  stopping = true;
  if (IS_WINDOWS) {
    // The npm.cmd shell wrapper is the direct child; kill the whole tree.
    spawnSync('taskkill', ['/pid', String(devServer.pid), '/T', '/F'], { stdio: 'ignore' });
  } else {
    devServer.kill('SIGTERM');
  }
}

for (const signal of ['SIGINT', 'SIGTERM']) {
  process.on(signal, () => {
    console.log('\n   Stopping the dev server ...');
    stopDevServer();
    process.exit(0);
  });
}

// Safety net so the server never outlives the launcher window.
process.on('exit', stopDevServer);

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

async function main() {
  console.log('\n6DOF robot arm -- control app launcher');
  console.log(`   repo: ${REPO_ROOT}`);

  updateCheckout();
  installDependencies();

  if (flags.updateOnly) {
    step('Done');
    info('Update finished. Start the app with start-robot-arm (or npm run launch).');
    return;
  }

  step('Starting the control app');

  const requestedPort = resolvePort();
  if (looksLikeOurApp(await httpGet(requestedPort))) {
    ok(`The app is already running on http://localhost:${requestedPort}.`);
    if (!flags.skipOpen) openBrowser(`http://localhost:${requestedPort}`);
    info('Nothing else to do -- this window can be closed.');
    return;
  }

  const port = await findFreePort(requestedPort);
  if (port === null) {
    fail(`Ports ${requestedPort}-${requestedPort + PORT_SCAN_RANGE - 1} are all in use.`);
  }
  if (port !== requestedPort) {
    warn(`Port ${requestedPort} is taken, using ${port} instead.`);
  }

  const url = `http://localhost:${port}`;
  info(`Compiling and serving on ${url} -- this takes about half a minute.`);

  devServer = spawn(NPM, ['start'], {
    cwd: APP_DIR,
    stdio: 'inherit',
    shell: IS_WINDOWS,
    env: { ...process.env, PORT: String(port), BROWSER: 'none' },
  });

  devServer.on('error', (error) => fail(`Could not start the dev server: ${error.message}`));
  devServer.on('exit', (code) => process.exit(code ?? 0));

  if (await waitForServer(port, devServer)) {
    if (flags.skipOpen) {
      ok(`Ready at ${url} (browser not opened, --no-open).`);
    } else {
      openBrowser(url);
    }
    console.log('\n   ---------------------------------------------------------------');
    console.log(`   The control app runs at ${url}`);
    console.log('   Connect the Teensy over USB, then press "Connect to Robot".');
    console.log('   Keep this window open; press Ctrl+C here to stop the app.');
    console.log('   ---------------------------------------------------------------\n');
  } else if (devServer.exitCode === null) {
    warn(`The dev server did not answer on ${url} yet. Watch the output above.`);
  }
}

main().catch((error) => {
  stopDevServer();
  fail(error?.stack ?? String(error));
});
