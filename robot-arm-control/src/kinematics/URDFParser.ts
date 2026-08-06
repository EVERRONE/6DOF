// URDF Parser for extracting kinematic chain information
import { URDFJoint } from './types';

/**
 * Parses URDF XML string and extracts joint information
 */
export class URDFParser {
  private joints: Map<string, URDFJoint> = new Map();

  /**
   * Parse URDF XML content
   */
  parse(urdfContent: string): void {
    this.joints.clear();

    // Parse joints from URDF
    const jointMatches = urdfContent.matchAll(
      /<joint\s+name="([^"]+)"\s+type="([^"]+)">([\s\S]*?)<\/joint>/g
    );

    for (const match of Array.from(jointMatches)) {
      const jointName = match[1];
      const jointType = match[2] as 'revolute' | 'continuous' | 'fixed';
      const jointContent = match[3];

      // Extract origin
      const originMatch = jointContent.match(
        /<origin\s+rpy="([^"]+)"\s+xyz="([^"]+)"/
      );

      if (!originMatch) continue;

      const rpy = originMatch[1].split(' ').map(parseFloat);
      const xyz = originMatch[2].split(' ').map(parseFloat);

      // Extract parent and child
      const parentMatch = jointContent.match(/<parent\s+link="([^"]+)"/);
      const childMatch = jointContent.match(/<child\s+link="([^"]+)"/);

      if (!parentMatch || !childMatch) continue;

      // Extract axis
      const axisMatch = jointContent.match(/<axis\s+xyz="([^"]+)"/);
      const axis = axisMatch
        ? axisMatch[1].split(' ').map(parseFloat)
        : [0, 0, 1];

      // Extract limits (if present)
      const limitMatch = jointContent.match(
        /<limit\s+lower="([^"]+)"\s+upper="([^"]+)"\s+effort="([^"]+)"\s+velocity="([^"]+)"/
      );

      const joint: URDFJoint = {
        name: jointName,
        type: jointType,
        parent: parentMatch[1],
        child: childMatch[1],
        origin: {
          xyz: { x: xyz[0], y: xyz[1], z: xyz[2] },
          rpy: { roll: rpy[0], pitch: rpy[1], yaw: rpy[2] }
        },
        axis: { x: axis[0], y: axis[1], z: axis[2] }
      };

      if (limitMatch && jointType === 'revolute') {
        joint.limit = {
          lower: parseFloat(limitMatch[1]),
          upper: parseFloat(limitMatch[2]),
          effort: parseFloat(limitMatch[3]),
          velocity: parseFloat(limitMatch[4])
        };
      }

      this.joints.set(jointName, joint);
    }
  }

  /**
   * Get the kinematic chain starting from a root link
   * Returns ordered list of joints from base to end-effector
   */
  getKinematicChain(rootLink: string = 'linkB'): URDFJoint[] {
    const chain: URDFJoint[] = [];
    let currentLink = rootLink;

    // Build chain by following parent-child relationships
    // using deterministic map iteration to avoid closure capture pitfalls.
    while (true) {
      let foundNext = false;
      for (const joint of Array.from(this.joints.values())) {
        if (joint.parent === currentLink) {
          chain.push(joint);
          currentLink = joint.child;
          foundNext = true;
          break;
        }
      }
      if (!foundNext) break;
    }

    return chain;
  }

  /**
   * Get ordered list of revolute joints (excludes fixed joints)
   */
  getRevoluteJoints(): URDFJoint[] {
    const chain = this.getKinematicChain();
    return chain.filter(j => j.type === 'revolute' || j.type === 'continuous');
  }

  /**
   * Get joint by name
   */
  getJoint(name: string): URDFJoint | undefined {
    return this.joints.get(name);
  }

  /**
   * Get all joints
   */
  getAllJoints(): URDFJoint[] {
    return Array.from(this.joints.values());
  }
}

/**
 * Hardcoded kinematic chain for the 6DOF robot arm
 * Extracted from URDF.md
 *
 * Chain: linkB → (J1/link0_joint) → link0 → (J2/Joint2) → link1 →
 *        (J3/Joint3) → link2 → (J4/link3_joint) → link3 →
 *        (J5/Joint5) → link4 → (J6/Joint6) → link5
 */
export const ROBOT_KINEMATIC_CHAIN: URDFJoint[] = [
  {
    name: 'link0_joint',  // J1
    type: 'revolute',
    parent: 'linkB',
    child: 'link0',
    origin: {
      xyz: { x: 0.0, y: 0.0, z: 0.08 },
      rpy: { roll: 0.0, pitch: 0.0, yaw: 0.0 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: -2.8,
      upper: 2.8,
      effort: 10.0,
      velocity: 2.0
    }
  },
  {
    name: 'Joint2',  // J2
    type: 'revolute',
    parent: 'link0',
    child: 'link1',
    origin: {
      xyz: { x: -0.0375, y: 0.02, z: 0.05595 },
      rpy: { roll: -1.5708, pitch: 0.0, yaw: 0.0 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: -1.3,
      upper: 1.3,
      effort: 12.0,
      velocity: 1.6
    }
  },
  {
    name: 'Joint3',  // J3
    type: 'revolute',
    parent: 'link1',
    child: 'link2',
    origin: {
      xyz: { x: 0.00027, y: -0.16, z: 0.016 },
      rpy: { roll: -3.14159, pitch: 0.0, yaw: 0.0 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: -2.1,
      upper: 2.1,
      effort: 10.0,
      velocity: 2.2
    }
  },
  {
    name: 'link3_joint',  // J4
    type: 'revolute',
    parent: 'link2',
    child: 'link3',
    origin: {
      xyz: { x: -0.035, y: 0.0151, z: 0.0364 },
      rpy: { roll: -1.5708, pitch: 0.0, yaw: 1.5708 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: -2.5,
      upper: 2.5,
      effort: 6.0,
      velocity: 3.0
    }
  },
  {
    name: 'Joint5',  // J5
    type: 'revolute',
    parent: 'link3',
    child: 'link4',
    origin: {
      xyz: { x: 0.0, y: -0.01002, z: 0.10323 },
      rpy: { roll: -1.5708, pitch: 1.5708, yaw: 0.0 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: -2.5,
      upper: 2.5,
      effort: 5.0,
      velocity: 3.5
    }
  },
  {
    name: 'Joint6',  // J6
    type: 'continuous',
    parent: 'link4',
    child: 'link5',
    origin: {
      xyz: { x: -0.02677, y: 0.0, z: 0.00994 },
      rpy: { roll: 1.5708, pitch: 0.0, yaw: -1.5708 }
    },
    axis: { x: 0.0, y: 0.0, z: 1.0 },
    limit: {
      lower: 0,
      upper: 0,
      effort: 4.0,
      velocity: 4.0
    }
  }
];
