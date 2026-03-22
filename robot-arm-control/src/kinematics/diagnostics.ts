import { ForwardKinematics } from './ForwardKinematics';

export interface JointSensitivity {
  jointIndex: number;
  positionNorm: number;
  orientationNorm: number;
}

export const computeJacobianSensitivity = (jointAnglesDeg: number[]): JointSensitivity[] => {
  const jacobian = ForwardKinematics.computeJacobian(jointAnglesDeg);
  if (jacobian.length !== 6) return [];

  const jointCount = jacobian[0]?.length ?? 0;
  const sensitivities: JointSensitivity[] = [];

  for (let i = 0; i < jointCount; i++) {
    const px = jacobian[0][i] ?? 0;
    const py = jacobian[1][i] ?? 0;
    const pz = jacobian[2][i] ?? 0;
    const ox = jacobian[3][i] ?? 0;
    const oy = jacobian[4][i] ?? 0;
    const oz = jacobian[5][i] ?? 0;

    sensitivities.push({
      jointIndex: i,
      positionNorm: Math.sqrt(px * px + py * py + pz * pz),
      orientationNorm: Math.sqrt(ox * ox + oy * oy + oz * oz)
    });
  }

  return sensitivities;
};

export const logJointSensitivity = (jointAnglesDeg: number[], label = 'IK Jacobian'): void => {
  const sensitivity = computeJacobianSensitivity(jointAnglesDeg);
  if (sensitivity.length === 0) return;

  console.groupCollapsed(`${label} sensitivity`);
  sensitivity.forEach((item) => {
    console.log(
      `J${item.jointIndex + 1}: |dPos/dq|=${item.positionNorm.toExponential(3)} ` +
      `|dOri/dq|=${item.orientationNorm.toExponential(3)}`
    );
  });
  console.groupEnd();
};
