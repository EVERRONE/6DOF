// Hiding the console must not hide a fault.
//
// The log is where the firmware's own words end up - "Endstop triggered during
// move on J3" among them - so a collapse that simply stops rendering would turn
// the one channel the arm has for bad news into a silent one.

import React from 'react';
import { render, screen, act } from '@testing-library/react';
import { CommandConsole } from './CommandConsole';
import { useRobotStore } from '../store/robotStore';

beforeEach(() => {
  window.localStorage.clear();
  useRobotStore.setState({ events: [] });
});

const log = (kind: 'error' | 'info', text: string) =>
  act(() => {
    useRobotStore.getState().logEvent(kind, text);
  });

describe('collapsing the console', () => {
  it('starts open and hides on the toggle', () => {
    render(<CommandConsole />);
    expect(screen.getByText(/Nothing yet/)).toBeInTheDocument();

    act(() => {
      screen.getByTitle('Hide the console').click();
    });
    expect(screen.queryByText(/Nothing yet/)).not.toBeInTheDocument();

    act(() => {
      screen.getByTitle('Show the console').click();
    });
    expect(screen.getByText(/Nothing yet/)).toBeInTheDocument();
  });

  it('remembers the choice', () => {
    const first = render(<CommandConsole />);
    act(() => {
      screen.getByTitle('Hide the console').click();
    });
    first.unmount();

    render(<CommandConsole />);
    expect(screen.getByTitle('Show the console')).toBeInTheDocument();
  });

  it('says so when an error arrives while it is shut', () => {
    render(<CommandConsole />);
    act(() => {
      screen.getByTitle('Hide the console').click();
    });

    log('info', 'nothing much');
    expect(screen.queryByText(/new error/)).not.toBeInTheDocument();

    log('error', 'Endstop triggered during move on J3');
    expect(screen.getByText('1 new error')).toBeInTheDocument();

    log('error', 'and another');
    expect(screen.getByText('2 new errors')).toBeInTheDocument();
  });

  it('stops calling them new once they have been seen', () => {
    render(<CommandConsole />);
    act(() => {
      screen.getByTitle('Hide the console').click();
    });
    log('error', 'something went wrong');
    expect(screen.getByText('1 new error')).toBeInTheDocument();

    // Opening it is reading it.
    act(() => {
      screen.getByText('1 new error').click();
    });
    act(() => {
      screen.getByTitle('Hide the console').click();
    });

    expect(screen.queryByText(/new error/)).not.toBeInTheDocument();
    // Still counted, just no longer news.
    expect(screen.getByText('1 error')).toBeInTheDocument();
  });

  it('does not call an error new when it arrived while the console was open', () => {
    render(<CommandConsole />);
    log('error', 'seen while open');

    act(() => {
      screen.getByTitle('Hide the console').click();
    });
    expect(screen.queryByText(/new error/)).not.toBeInTheDocument();
    expect(screen.getByText('1 error')).toBeInTheDocument();
  });
});
