import React from 'react';
import { render, screen } from '@testing-library/react';

jest.mock('./components/RobotViewer3D', () => ({
  RobotViewer3D: () => <div data-testid="robot-viewer-mock">Robot Viewer Mock</div>
}));

import App from './App';

test('renders main control panels', () => {
  render(<App />);
  expect(screen.getByText(/Cartesian Control/i)).toBeInTheDocument();
  expect(screen.getByTestId('robot-viewer-mock')).toBeInTheDocument();
});
