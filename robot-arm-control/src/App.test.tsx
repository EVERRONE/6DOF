import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

// The 3D viewer needs a WebGL context, which jsdom does not provide. Stubbing it
// keeps this a smoke test of the app shell: that every module imports, the store
// initialises, and nothing throws on first render.
jest.mock('./components/RobotViewer3D', () => ({
  RobotViewer3D: () => <div data-testid="robot-viewer-stub" />
}));

test('renders the control panels without a connected robot', () => {
  render(<App />);

  expect(screen.getByTestId('robot-viewer-stub')).toBeInTheDocument();
  expect(screen.getByText(/cartesian control/i)).toBeInTheDocument();
  expect(screen.getByText(/state:/i)).toBeInTheDocument();
});
