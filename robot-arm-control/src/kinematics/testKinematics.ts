// FK/IK Round-Trip Accuracy Test
import { ForwardKinematics } from './ForwardKinematics';
import { InverseKinematics } from './InverseKinematics';
import { Vector3 } from './types';

/**
 * Test forward kinematics and inverse kinematics round-trip accuracy
 *
 * Test process:
 * 1. Start with known joint angles
 * 2. Compute FK to get XYZ position
 * 3. Compute IK to get back to joint angles
 * 4. Compare original angles with IK solution
 * 5. Verify position error is within tolerance
 */

interface TestResult {
  testName: string;
  success: boolean;
  jointError: number;    // Max joint angle error (degrees)
  positionError: number; // Position error (mm)
  originalAngles: number[];
  ikAngles: number[];
  position: Vector3;
  ikIterations?: number;
}

const ikSolver = new InverseKinematics({
  maxIterations: 100,
  tolerance: 0.001, // 1mm
  dampingFactor: 0.01
});

/**
 * Run FK/IK round-trip test for a given set of joint angles
 */
function testRoundTrip(
  testName: string,
  jointAngles: number[]
): TestResult {
  // 1. Compute forward kinematics
  const fkResult = ForwardKinematics.solve(jointAngles);

  if (!fkResult.success) {
    return {
      testName,
      success: false,
      jointError: Infinity,
      positionError: Infinity,
      originalAngles: jointAngles,
      ikAngles: [],
      position: { x: 0, y: 0, z: 0 }
    };
  }

  const position = fkResult.endEffectorPose.position;

  // 2. Compute inverse kinematics
  const ikResult = ikSolver.solvePosition(position, jointAngles);

  if (!ikResult.success) {
    return {
      testName,
      success: false,
      jointError: Infinity,
      positionError: Infinity,
      originalAngles: jointAngles,
      ikAngles: ikResult.jointAngles,
      position,
      ikIterations: ikResult.iterations
    };
  }

  // 3. Verify FK of IK solution
  const fkVerify = ForwardKinematics.solve(ikResult.jointAngles);

  if (!fkVerify.success) {
    return {
      testName,
      success: false,
      jointError: Infinity,
      positionError: Infinity,
      originalAngles: jointAngles,
      ikAngles: ikResult.jointAngles,
      position,
      ikIterations: ikResult.iterations
    };
  }

  // 4. Compute errors
  const positionError = Math.sqrt(
    Math.pow(fkVerify.endEffectorPose.position.x - position.x, 2) +
    Math.pow(fkVerify.endEffectorPose.position.y - position.y, 2) +
    Math.pow(fkVerify.endEffectorPose.position.z - position.z, 2)
  ) * 1000; // Convert to mm

  let maxJointError = 0;
  for (let i = 0; i < 6; i++) {
    const error = Math.abs(ikResult.jointAngles[i] - jointAngles[i]);
    maxJointError = Math.max(maxJointError, error);
  }

  const success = positionError < 1.0 && maxJointError < 5.0; // 1mm, 5 degrees

  return {
    testName,
    success,
    jointError: maxJointError,
    positionError,
    originalAngles: jointAngles,
    ikAngles: ikResult.jointAngles,
    position,
    ikIterations: ikResult.iterations
  };
}

/**
 * Run all FK/IK accuracy tests
 */
export function runAllTests(): TestResult[] {
  const testCases = [
    {
      name: 'Home position (all zeros)',
      angles: [0, 0, 0, 0, 0, 0]
    },
    {
      name: 'J1 rotation only',
      angles: [30, 0, 0, 0, 0, 0]
    },
    {
      name: 'J2 shoulder pitch',
      angles: [0, 20, 0, 0, 0, 0]
    },
    {
      name: 'J3 elbow pitch',
      angles: [0, 0, 30, 0, 0, 0]
    },
    {
      name: 'Combined J2 + J3',
      angles: [0, 20, 30, 0, 0, 0]
    },
    {
      name: 'All joints moderate',
      angles: [15, 20, 25, 30, 40, 50]
    },
    {
      name: 'Reach forward',
      angles: [0, 30, 45, 0, 0, 0]
    },
    {
      name: 'Reach side',
      angles: [45, 20, 30, 0, 0, 0]
    },
    {
      name: 'Maximum reach',
      angles: [0, 45, 60, 0, 0, 0]
    },
    {
      name: 'Negative J1',
      angles: [-30, 20, 30, 0, 0, 0]
    }
  ];

  const results: TestResult[] = [];

  for (const testCase of testCases) {
    const result = testRoundTrip(testCase.name, testCase.angles);
    results.push(result);
  }

  return results;
}

/**
 * Print test results to console
 */
export function printTestResults(results: TestResult[]): void {
  console.log('\n========================================');
  console.log('FK/IK Round-Trip Accuracy Test Results');
  console.log('========================================\n');

  let passCount = 0;
  let failCount = 0;

  for (const result of results) {
    const status = result.success ? '✓ PASS' : '✗ FAIL';
    console.log(`${status} - ${result.testName}`);
    console.log(`  Position: (${result.position.x.toFixed(3)}, ${result.position.y.toFixed(3)}, ${result.position.z.toFixed(3)}) m`);
    console.log(`  Position error: ${result.positionError.toFixed(3)} mm`);
    console.log(`  Joint error: ${result.jointError.toFixed(3)}°`);
    console.log(`  IK iterations: ${result.ikIterations || 'N/A'}`);
    console.log(`  Original: [${result.originalAngles.map(a => a.toFixed(1)).join(', ')}]`);
    console.log(`  IK solution: [${result.ikAngles.map(a => a.toFixed(1)).join(', ')}]`);
    console.log('');

    if (result.success) {
      passCount++;
    } else {
      failCount++;
    }
  }

  console.log('========================================');
  console.log(`Total: ${results.length} tests`);
  console.log(`Passed: ${passCount}`);
  console.log(`Failed: ${failCount}`);
  console.log(`Success rate: ${((passCount / results.length) * 100).toFixed(1)}%`);
  console.log('========================================\n');
}

/**
 * Run tests in browser console
 * Call this from browser dev tools:
 * > import('./kinematics/testKinematics').then(m => m.runTestsInConsole())
 */
export function runTestsInConsole(): void {
  const results = runAllTests();
  printTestResults(results);
}
