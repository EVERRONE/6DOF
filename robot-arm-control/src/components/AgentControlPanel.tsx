import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { AgentBridgeClient, AgentActivityEntry } from '../agent/AgentBridgeClient';
import { BridgeLinkState } from '../agent/bridgeProtocol';

/**
 * Agent bridge control panel.
 *
 * Two rules shape this component:
 *
 *  - It is off by default. With the bridge disabled — or the broker simply not
 *    running — the app behaves exactly as it did before this panel existed.
 *  - Arming is a time-boxed window, not a per-command click. The operator will
 *    be texting the agent from another room, so requiring a click per motion
 *    would make the feature useless; requiring a human at the machine to open
 *    the window keeps the safety property that matters.
 */

const STORAGE_KEY = 'agentBridge.settings.v1';
const DEFAULT_URL = 'ws://localhost:8765/executor';
const ARM_DURATION_CHOICES = [5, 15, 30, 60];

interface StoredSettings {
  url: string;
  token: string;
  armMinutes: number;
}

const loadSettings = (): StoredSettings => {
  const fallback: StoredSettings = { url: DEFAULT_URL, token: '', armMinutes: 30 };
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return fallback;
    const parsed = JSON.parse(raw) as Partial<StoredSettings>;
    return {
      url: typeof parsed.url === 'string' && parsed.url ? parsed.url : fallback.url,
      token: typeof parsed.token === 'string' ? parsed.token : '',
      armMinutes: typeof parsed.armMinutes === 'number' ? parsed.armMinutes : fallback.armMinutes
    };
  } catch {
    return fallback;
  }
};

const LINK_LABEL: Record<BridgeLinkState, string> = {
  disabled: 'Off',
  connecting: 'Connecting…',
  connected: 'Connected',
  reconnecting: 'Reconnecting…',
  error: 'Error'
};

const LINK_STYLE: Record<BridgeLinkState, string> = {
  disabled: 'bg-gray-100 text-gray-600',
  connecting: 'bg-amber-50 text-amber-700',
  connected: 'bg-green-50 text-green-700',
  reconnecting: 'bg-amber-50 text-amber-700',
  error: 'bg-red-50 text-red-700'
};

export const AgentControlPanel: React.FC = () => {
  const [settings, setSettings] = useState<StoredSettings>(loadSettings);
  const [enabled, setEnabled] = useState(false);
  const [linkState, setLinkState] = useState<BridgeLinkState>('disabled');
  const [linkDetail, setLinkDetail] = useState<string>('');
  const [activity, setActivity] = useState<AgentActivityEntry[]>([]);
  const [armedUntil, setArmedUntil] = useState<number | null>(null);
  const [now, setNow] = useState(Date.now());

  const clientRef = useRef<AgentBridgeClient | null>(null);

  useEffect(() => {
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
    } catch {
      // Private browsing or a full quota — the panel still works, it just
      // forgets the settings between reloads.
    }
  }, [settings]);

  // Drives the arming countdown.
  useEffect(() => {
    if (armedUntil === null) return;
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, [armedUntil]);

  const armSecondsLeft = useMemo(() => {
    if (armedUntil === null) return 0;
    return Math.max(0, Math.ceil((armedUntil - now) / 1000));
  }, [armedUntil, now]);

  useEffect(() => {
    if (armedUntil !== null && armSecondsLeft === 0) {
      clientRef.current?.disarm();
      setArmedUntil(null);
    }
  }, [armSecondsLeft, armedUntil]);

  const pushActivity = useCallback((entry: AgentActivityEntry) => {
    setActivity((prev) => [entry, ...prev].slice(0, 12));
  }, []);

  useEffect(() => {
    if (!enabled) {
      clientRef.current?.stop();
      clientRef.current = null;
      setLinkState('disabled');
      setArmedUntil(null);
      return;
    }

    const client = new AgentBridgeClient({
      url: settings.url,
      token: settings.token,
      onLinkStateChange: (state, detail) => {
        setLinkState(state);
        setLinkDetail(detail ?? '');
      },
      onActivity: pushActivity
    });

    clientRef.current = client;
    client.start();

    return () => {
      client.stop();
      clientRef.current = null;
    };
  }, [enabled, settings.url, settings.token, pushActivity]);

  const handleArm = () => {
    const client = clientRef.current;
    if (!client) return;
    client.arm(settings.armMinutes);
    setArmedUntil(client.getArmedUntil());
    setNow(Date.now());
  };

  const handleDisarm = () => {
    clientRef.current?.disarm();
    setArmedUntil(null);
  };

  const armed = armedUntil !== null && armSecondsLeft > 0;
  const canArm = enabled && linkState === 'connected';

  return (
    <div className="p-4 bg-white border-b">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-xl font-bold">Agent Control</h2>
        <span className={`text-xs px-2 py-0.5 rounded font-medium ${LINK_STYLE[linkState]}`}>
          {LINK_LABEL[linkState]}
        </span>
      </div>

      <label className="flex items-center gap-2 mb-3 text-sm">
        <input
          type="checkbox"
          checked={enabled}
          onChange={(e) => setEnabled(e.target.checked)}
        />
        <span>Connect to agent bridge</span>
      </label>

      {!enabled && (
        <p className="text-xs text-gray-500 mb-3">
          Off. The app runs exactly as it does without the bridge.
        </p>
      )}

      <div className="grid grid-cols-1 gap-2 mb-3">
        <div>
          <label className="block text-xs text-gray-600 mb-1" htmlFor="agent-bridge-url">
            Bridge URL
          </label>
          <input
            id="agent-bridge-url"
            type="text"
            className="w-full px-2 py-1 border rounded text-sm font-mono"
            value={settings.url}
            disabled={enabled}
            onChange={(e) => setSettings((s) => ({ ...s, url: e.target.value }))}
          />
        </div>
        <div>
          <label className="block text-xs text-gray-600 mb-1" htmlFor="agent-bridge-token">
            Token
          </label>
          <input
            id="agent-bridge-token"
            type="password"
            className="w-full px-2 py-1 border rounded text-sm font-mono"
            value={settings.token}
            disabled={enabled}
            placeholder="BRIDGE_TOKEN from the broker"
            onChange={(e) => setSettings((s) => ({ ...s, token: e.target.value }))}
          />
        </div>
      </div>

      {linkDetail && linkState !== 'connected' && (
        <p className="text-xs text-amber-700 mb-3">{linkDetail}</p>
      )}

      <div className="p-3 rounded border mb-3 bg-gray-50">
        <div className="flex items-center justify-between mb-2">
          <span className="text-sm font-semibold">AI control</span>
          <span className={`text-xs px-2 py-0.5 rounded font-medium ${armed ? 'bg-green-50 text-green-700' : 'bg-gray-100 text-gray-600'}`}>
            {armed ? `Armed · ${Math.floor(armSecondsLeft / 60)}m ${armSecondsLeft % 60}s left` : 'Not armed'}
          </span>
        </div>

        <p className="text-xs text-gray-600 mb-2">
          No motion command is accepted unless armed here, at the machine. Arming
          expires on its own, and an emergency stop cancels it.
        </p>

        <div className="flex items-center gap-2">
          <select
            className="px-2 py-1 border rounded text-sm"
            value={settings.armMinutes}
            disabled={armed}
            onChange={(e) => setSettings((s) => ({ ...s, armMinutes: Number(e.target.value) }))}
          >
            {ARM_DURATION_CHOICES.map((m) => (
              <option key={m} value={m}>{m} min</option>
            ))}
          </select>

          {armed ? (
            <button
              type="button"
              onClick={handleDisarm}
              className="px-3 py-1 rounded text-sm bg-red-600 text-white hover:bg-red-700"
            >
              Disarm
            </button>
          ) : (
            <button
              type="button"
              onClick={handleArm}
              disabled={!canArm}
              className="px-3 py-1 rounded text-sm bg-blue-600 text-white hover:bg-blue-700 disabled:bg-gray-300 disabled:text-gray-500"
            >
              Arm AI control
            </button>
          )}
        </div>

        {!canArm && enabled && (
          <p className="text-xs text-gray-500 mt-2">Connect to the bridge before arming.</p>
        )}
      </div>

      <div>
        <h3 className="text-sm font-semibold mb-1 text-gray-700">Recent agent activity</h3>
        {activity.length === 0 ? (
          <p className="text-xs text-gray-500">Nothing yet.</p>
        ) : (
          <ul className="text-xs font-mono space-y-0.5 max-h-32 overflow-y-auto">
            {activity.map((entry) => (
              <li key={`${entry.at}-${entry.type}`} className={entry.ok ? 'text-gray-700' : 'text-red-700'}>
                {new Date(entry.at).toLocaleTimeString()} · {entry.type} · {entry.summary}
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
};
