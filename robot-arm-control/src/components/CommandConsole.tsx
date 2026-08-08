import React, { useEffect, useMemo, useRef, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

/**
 * Event log and raw command entry.
 *
 * Two things the app could not do before, both needed for bench work:
 *
 *  - See what the arm said. Firmware replies went to console.error, which hid
 *    "Endstop triggered during move on J3" from the person standing next to it.
 *  - Send an exact command. The bring-up checklist calls for lines like
 *    `J 0 0 10 0 0 0 5`, and the only alternative was a separate serial monitor,
 *    which cannot hold the port at the same time as the app.
 */

const KIND_STYLE: Record<string, string> = {
  sent: 'text-blue-700',
  ok: 'text-green-700',
  error: 'text-red-700 font-semibold',
  info: 'text-gray-600'
};

const KIND_PREFIX: Record<string, string> = {
  sent: '>',
  ok: 'ok',
  error: '!!',
  info: '--'
};

/** Ready-made commands from the bring-up checklist. */
const SHORTCUTS: { label: string; command: string; title: string }[] = [
  { label: 'Q', command: 'Q', title: 'Ask for position, endstops and status' },
  { label: 'E 1', command: 'E 1', title: 'Energise the drivers' },
  { label: 'E 0', command: 'E 0', title: 'De-energise the drivers (the arm may sag)' },
  { label: 'A', command: 'A', title: 'Abort: decelerate and drop the queue' }
];

const COLLAPSED_KEY = 'robot-arm.console.collapsed';

function loadCollapsed(): boolean {
  try {
    return window.localStorage?.getItem(COLLAPSED_KEY) === '1';
  } catch {
    return false;
  }
}

export const CommandConsole: React.FC = () => {
  const { events, clearEvents, sendRawCommand, connectionStatus } = useRobotStore();

  const [input, setInput] = useState('');
  const [history, setHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [follow, setFollow] = useState(true);
  const [collapsed, setCollapsed] = useState(loadCollapsed);

  const listRef = useRef<HTMLDivElement>(null);
  const connected = connectionStatus === ConnectionStatus.CONNECTED;

  // Keep the newest line in view, unless the user has scrolled up to read.
  useEffect(() => {
    if (collapsed || !follow || !listRef.current) return;
    listRef.current.scrollTop = listRef.current.scrollHeight;
  }, [events, follow, collapsed]);

  const toggle = () => {
    const next = !collapsed;
    setCollapsed(next);
    try {
      window.localStorage?.setItem(COLLAPSED_KEY, next ? '1' : '0');
    } catch {
      // No storage. The choice still holds for this session, which is the part
      // that matters.
    }
  };

  const errorCount = useMemo(
    () => events.filter(e => e.kind === 'error').length,
    [events]
  );

  /**
   * The newest line that has been on screen.
   *
   * Hiding the log must not hide a fault. While the console is open this tracks
   * the latest event, so nothing counts as unseen; while it is shut it stops,
   * and anything that arrives after becomes something to say out loud. Without
   * it, collapsing the console would quietly turn "Endstop triggered during move
   * on J3" into a line nobody ever reads.
   */
  const lastSeen = useRef(0);
  useEffect(() => {
    if (!collapsed && events.length > 0) {
      lastSeen.current = events[events.length - 1].id;
    }
  }, [collapsed, events]);

  const unseenErrors = collapsed
    ? events.filter(e => e.kind === 'error' && e.id > lastSeen.current).length
    : 0;

  if (collapsed) {
    return (
      <div className="px-4 py-2 bg-white border-t flex items-center justify-between">
        <h2 className="text-sm font-semibold text-gray-700">
          <button
            onClick={toggle}
            className="flex items-center gap-2 hover:text-gray-900"
            title="Show the console"
          >
            <span className="text-gray-400">▸</span>
            Console
          </button>
        </h2>

        <div className="flex items-center gap-2">
          {unseenErrors > 0 && (
            <button
              onClick={toggle}
              className="px-2 py-0.5 rounded-full bg-red-600 text-white text-xs font-semibold hover:bg-red-700"
              title="Open the console and read them"
            >
              {unseenErrors} new error{unseenErrors === 1 ? '' : 's'}
            </button>
          )}
          {unseenErrors === 0 && errorCount > 0 && (
            <span className="px-2 py-0.5 rounded-full bg-red-100 text-red-700 text-xs">
              {errorCount} error{errorCount === 1 ? '' : 's'}
            </span>
          )}
          <span className="text-xs text-gray-400">{events.length} lines</span>
        </div>
      </div>
    );
  }

  const submit = async (command: string) => {
    const trimmed = command.trim();
    if (trimmed.length === 0) return;

    setHistory(previous => [...previous.filter(h => h !== trimmed), trimmed].slice(-30));
    setHistoryIndex(-1);
    setInput('');
    await sendRawCommand(trimmed);
  };

  const handleKeyDown = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      void submit(input);
      return;
    }

    // Up and down walk the command history, as in a terminal.
    if (event.key === 'ArrowUp') {
      event.preventDefault();
      if (history.length === 0) return;
      const next = historyIndex < 0 ? history.length - 1 : Math.max(0, historyIndex - 1);
      setHistoryIndex(next);
      setInput(history[next]);
      return;
    }

    if (event.key === 'ArrowDown') {
      event.preventDefault();
      if (historyIndex < 0) return;
      const next = historyIndex + 1;
      if (next >= history.length) {
        setHistoryIndex(-1);
        setInput('');
        return;
      }
      setHistoryIndex(next);
      setInput(history[next]);
    }
  };

  return (
    <div className="p-4 bg-white border-t">
      <div className="flex items-center justify-between mb-2">
        {/* The title stays a heading and the toggle lives inside it. Turning the
            whole thing into a button removed the heading role, which is what a
            screen reader and the app's own smoke test navigate the panel by. */}
        <h2 className="text-lg font-bold">
          <button
            onClick={toggle}
            className="flex items-center gap-2 hover:text-gray-700"
            title="Hide the console"
          >
            <span className="text-gray-400 text-sm">▾</span>
            Console
            {errorCount > 0 && (
              <span className="px-2 py-0.5 rounded-full bg-red-100 text-red-700 text-xs font-semibold align-middle">
                {errorCount} error{errorCount === 1 ? '' : 's'}
              </span>
            )}
          </button>
        </h2>

        <div className="flex items-center gap-3 text-xs">
          <label className="flex items-center gap-1 text-gray-600">
            <input
              type="checkbox"
              checked={follow}
              onChange={e => setFollow(e.target.checked)}
            />
            Follow
          </label>
          <button
            onClick={clearEvents}
            className="px-2 py-1 border rounded text-gray-600 hover:bg-gray-50"
          >
            Clear
          </button>
          <button
            onClick={toggle}
            className="px-2 py-1 border rounded text-gray-600 hover:bg-gray-50"
          >
            Hide
          </button>
        </div>
      </div>

      <div
        ref={listRef}
        className="h-48 overflow-y-auto bg-gray-900 rounded p-2 font-mono text-xs leading-relaxed"
      >
        {events.length === 0 ? (
          <div className="text-gray-500">
            Nothing yet. Everything the arm reports appears here.
          </div>
        ) : (
          events.map(event => (
            <div key={event.id} className="flex gap-2">
              <span className="text-gray-500 shrink-0">
                {new Date(event.at).toLocaleTimeString([], { hour12: false })}
              </span>
              <span
                className={`shrink-0 w-5 ${
                  event.kind === 'error' ? 'text-red-400' : 'text-gray-500'
                }`}
              >
                {KIND_PREFIX[event.kind]}
              </span>
              <span
                className={`${KIND_STYLE[event.kind]} break-all`}
                style={{
                  color:
                    event.kind === 'error'
                      ? '#fca5a5'
                      : event.kind === 'ok'
                      ? '#86efac'
                      : event.kind === 'sent'
                      ? '#93c5fd'
                      : '#d1d5db'
                }}
              >
                {event.text}
              </span>
            </div>
          ))
        )}
      </div>

      <div className="mt-2 flex gap-2">
        <input
          type="text"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          disabled={!connected}
          placeholder={connected ? 'J 0 5 55 129 220 0 20' : 'Connect first'}
          spellCheck={false}
          className="flex-1 px-2 py-1 border rounded text-sm font-mono disabled:bg-gray-100"
        />
        <button
          onClick={() => void submit(input)}
          disabled={!connected || input.trim().length === 0}
          className="px-3 py-1 bg-gray-700 text-white text-sm rounded hover:bg-gray-800 disabled:bg-gray-300"
        >
          Send
        </button>
      </div>

      <div className="mt-2 flex flex-wrap items-center gap-1">
        <span className="text-xs text-gray-500 mr-1">Quick:</span>
        {SHORTCUTS.map(shortcut => (
          <button
            key={shortcut.command}
            onClick={() => void submit(shortcut.command)}
            disabled={!connected}
            title={shortcut.title}
            className="px-2 py-0.5 border rounded text-xs font-mono text-gray-700 hover:bg-gray-50 disabled:opacity-40"
          >
            {shortcut.label}
          </button>
        ))}
        <span className="text-xs text-gray-400 ml-2">
          Up/Down for history. See docs/SERIAL_PROTOCOL.md for the full command set.
        </span>
      </div>
    </div>
  );
};
