import React from 'react';
import { fireEvent, render, screen } from '@testing-library/react';
import App from './App';

// The 3D viewer needs a WebGL context, which jsdom does not provide. Stubbing it
// keeps this a smoke test of the app shell: that every module imports, the store
// initialises, and nothing throws on first render.
jest.mock('./components/RobotViewer3D', () => ({
  RobotViewer3D: () => <div data-testid="robot-viewer-stub" />
}));

test('opens on the jog tab, with the console always to hand', () => {
  render(<App />);

  expect(screen.getByTestId('robot-viewer-stub')).toBeInTheDocument();

  // Jog is the default, because that is what bench work starts with.
  expect(screen.getByRole('heading', { name: /^jog$/i })).toBeInTheDocument();
  expect(screen.getByRole('heading', { name: /console/i })).toBeInTheDocument();
  expect(screen.getByText(/state:/i)).toBeInTheDocument();
});

test('the other panels are reachable by tab', () => {
  render(<App />);

  fireEvent.click(screen.getByRole('button', { name: /^cartesian$/i }));
  expect(
    screen.getByRole('heading', { name: /cartesian control/i })
  ).toBeInTheDocument();
  expect(screen.queryByRole('heading', { name: /^jog$/i })).not.toBeInTheDocument();

  fireEvent.click(screen.getByRole('button', { name: /^paths$/i }));
  expect(screen.getByRole('heading', { name: /path planner/i })).toBeInTheDocument();

  // The console stays put whichever tab is open.
  expect(screen.getByRole('heading', { name: /console/i })).toBeInTheDocument();
});

test('nothing can command the arm before a connection exists', () => {
  render(<App />);

  // Every control that moves the arm stays dead until there is a link, so a stray
  // click on a freshly loaded page cannot do anything.
  expect(screen.getByRole('button', { name: /turn motors on/i })).toBeDisabled();
  expect(screen.getByRole('button', { name: /move to target/i })).toBeDisabled();
  expect(screen.getByRole('button', { name: /home all/i })).toBeDisabled();
  expect(screen.getByRole('button', { name: /e-stop/i })).toBeDisabled();
});
