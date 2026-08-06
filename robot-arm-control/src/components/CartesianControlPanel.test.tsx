// The target fields have to survive the arm reporting its position.
//
// They used to be driven by an effect on currentPosition, which the store
// rebuilds on every POS report - a new object twenty times a second, so the
// effect fired even with the arm standing still and identical numbers. Every
// keystroke was overwritten before the next one landed, and the fields could
// not be typed into at all.

import React from 'react';
import { render, screen, act } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { CartesianControlPanel } from './CartesianControlPanel';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

/** Pretend a POS report arrived, the way the store handles one. */
function reportPosition(x: number, y: number, z: number) {
  act(() => {
    useRobotStore.setState({ currentPosition: { x, y, z } });
  });
}

beforeEach(() => {
  useRobotStore.setState({
    connectionStatus: ConnectionStatus.CONNECTED,
    currentPosition: null,
    currentRotation: null,
    toolLocked: false,
    lockedRotation: null,
    motorsEnabled: true,
    firmwareStatus: null
  });
});

function fields() {
  const boxes = screen.getAllByRole('spinbutton') as HTMLInputElement[];
  return { x: boxes[0], y: boxes[1], z: boxes[2] };
}

it('seeds the target fields from the arm once', () => {
  render(<CartesianControlPanel />);
  reportPosition(-0.2022, -0.0005, 0.311);

  const { x, z } = fields();
  expect(x.value).toBe('-202.2');
  expect(z.value).toBe('311.0');
});

it('keeps what the user typed while the arm keeps reporting', () => {
  render(<CartesianControlPanel />);
  reportPosition(-0.2022, -0.0005, 0.311);

  const { x } = fields();
  userEvent.clear(x);
  userEvent.type(x, '-150');

  // Twenty reports, as one second of standing still used to produce.
  for (let i = 0; i < 20; i++) reportPosition(-0.2022, -0.0005, 0.311);

  expect(x.value).toBe('-150');
});

it('survives reports that arrive mid-edit', () => {
  render(<CartesianControlPanel />);
  reportPosition(-0.2022, -0.0005, 0.311);

  const { z } = fields();
  userEvent.clear(z);

  // A report between two keystrokes is exactly what made this unusable.
  userEvent.type(z, '3');
  reportPosition(-0.2022, -0.0005, 0.311);
  userEvent.type(z, '5');
  reportPosition(-0.2022, -0.0005, 0.311);
  userEvent.type(z, '0');

  expect(z.value).toBe('350');
});

it('re-seeds after a reconnect rather than keeping stale numbers', () => {
  render(<CartesianControlPanel />);
  reportPosition(-0.2022, -0.0005, 0.311);

  act(() => {
    // What disconnect() actually does: the tool position stops being known.
    useRobotStore.setState({
      connectionStatus: ConnectionStatus.DISCONNECTED,
      currentPosition: null
    });
  });
  act(() => {
    useRobotStore.setState({ connectionStatus: ConnectionStatus.CONNECTED });
  });
  reportPosition(-0.1, 0.05, 0.25);

  const { x, z } = fields();
  expect(x.value).toBe('-100.0');
  expect(z.value).toBe('250.0');
});
