// Self-collision geometry, generated from the STL meshes in public/stl_meshes.
//
// Each link is fitted with three oriented boxes along its own longest axis,
// expressed in that link's joint frame. Boxes rather than spheres or capsules:
// these parts are printed plates and brackets, and a sphere fit around a 210 mm
// plate 30 mm thick is fat in the two directions that matter. Measured against
// the one collision known from the arm - J3 fouling at about 107 degrees - the
// sphere model predicted 83 and the box model predicts 109.
//
// DO NOT EDIT BY HAND. Regenerate if the meshes or the visual transforms in
// RobotModel3D.applyVisualTransform change.

import { Vector3 } from './types';

/** One oriented box: centre and axes in the link's frame, half-extents in metres. */
export interface OrientedBox {
  centre: Vector3;
  /** Three orthonormal axes, as rows. */
  axes: number[][];
  half: number[];
}

/** Boxes per link, indexed by joint. */
export const LINK_BOXES: OrientedBox[][] = [
  [
    {
      centre: { x: -0.045488, y: -0.005343, z: 0.050598 },
      axes: [[0.954768, 0.279807, 0.100625],
             [-0.217503, 0.887929, -0.405308],
             [-0.202755, 0.365089, 0.908625]],
      half: [0.025936, 0.036971, 0.049775]
    },
    {
      centre: { x: 0.004738, y: 0.007827, z: 0.052863 },
      axes: [[0.954768, 0.279807, 0.100625],
             [-0.217503, 0.887929, -0.405308],
             [-0.202755, 0.365089, 0.908625]],
      half: [0.025919, 0.028183, 0.050005]
    },
    {
      centre: { x: 0.056382, y: 0.017449, z: 0.052186 },
      axes: [[0.954768, 0.279807, 0.100625],
             [-0.217503, 0.887929, -0.405308],
             [-0.202755, 0.365089, 0.908625]],
      half: [0.025905, 0.021582, 0.031185]
    },
  ],
  [
    {
      centre: { x: 0.001288, y: -0.145097, z: 0.030696 },
      axes: [[0.999539, 0.028959, 0.009100],
             [-0.028758, 0.999357, -0.021430],
             [-0.009715, 0.021159, 0.999729]],
      half: [0.026982, 0.035047, 0.015461]
    },
    {
      centre: { x: 0.001032, y: -0.074969, z: 0.027255 },
      axes: [[0.999539, 0.028959, 0.009100],
             [-0.028758, 0.999357, -0.021430],
             [-0.009715, 0.021159, 0.999729]],
      half: [0.030941, 0.034903, 0.012225]
    },
    {
      centre: { x: -0.000658, y: -0.004760, z: 0.029730 },
      axes: [[0.999539, 0.028959, 0.009100],
             [-0.028758, 0.999357, -0.021430],
             [-0.009715, 0.021159, 0.999729]],
      half: [0.031081, 0.035011, 0.014732]
    },
  ],
  [
    {
      centre: { x: -0.027732, y: 0.010338, z: 0.031388 },
      axes: [[0.889758, -0.415760, 0.188346],
             [0.397139, 0.908575, 0.129503],
             [-0.224969, -0.040427, 0.973527]],
      half: [0.018111, 0.034115, 0.027606]
    },
    {
      centre: { x: 0.008977, y: 0.002834, z: 0.034113 },
      axes: [[0.889758, -0.415760, 0.188346],
             [0.397139, 0.908575, 0.129503],
             [-0.224969, -0.040427, 0.973527]],
      half: [0.018016, 0.043499, 0.028131]
    },
    {
      centre: { x: 0.043785, y: -0.007539, z: 0.037623 },
      axes: [[0.889758, -0.415760, 0.188346],
             [0.397139, 0.908575, 0.129503],
             [-0.224969, -0.040427, 0.973527]],
      half: [0.017883, 0.039289, 0.023247]
    },
  ],
  [
    {
      centre: { x: -0.000031, y: 0.001763, z: 0.019912 },
      axes: [[0.999452, -0.028741, -0.016397],
             [0.029852, 0.996949, 0.072124],
             [0.014274, -0.072574, 0.997261]],
      half: [0.018285, 0.022814, 0.020542]
    },
    {
      centre: { x: 0.000149, y: -0.004250, z: 0.061008 },
      axes: [[0.999452, -0.028741, -0.016397],
             [0.029852, 0.996949, 0.072124],
             [0.014274, -0.072574, 0.997261]],
      half: [0.018093, 0.028918, 0.020626]
    },
    {
      centre: { x: 0.000136, y: -0.000505, z: 0.102694 },
      axes: [[0.999452, -0.028741, -0.016397],
             [0.029852, 0.996949, 0.072124],
             [0.014274, -0.072574, 0.997261]],
      half: [0.018258, 0.024751, 0.020616]
    },
  ],
  [
    {
      centre: { x: -0.018072, y: 0.000000, z: 0.010006 },
      axes: [[0.997369, -0.000016, -0.072490],
             [-0.000004, 1.000000, -0.000280],
             [0.072490, 0.000279, 0.997369]],
      half: [0.002792, 0.011004, 0.013966]
    },
    {
      centre: { x: -0.001607, y: -0.000003, z: 0.010075 },
      axes: [[0.997369, -0.000016, -0.072490],
             [-0.000004, 1.000000, -0.000280],
             [0.072490, 0.000279, 0.997369]],
      half: [0.001900, 0.002498, 0.014102]
    },
    {
      centre: { x: 0.005703, y: 0.000000, z: 0.009931 },
      axes: [[0.997369, -0.000016, -0.072490],
             [-0.000004, 1.000000, -0.000280],
             [0.072490, 0.000279, 0.997369]],
      half: [0.005295, 0.011004, 0.014346]
    },
  ],
  [
    {
      centre: { x: -0.000000, y: -0.000000, z: 0.000000 },
      axes: [[0.995426, 0.095485, -0.003232],
             [-0.095472, 0.995424, 0.003951],
             [0.003595, -0.003625, 0.999987]],
      half: [0.001991, 0.001991, 0.000010]
    },
    {
      centre: { x: -0.000000, y: 0.000000, z: 0.005000 },
      axes: [[0.995426, 0.095485, -0.003232],
             [-0.095472, 0.995424, 0.003951],
             [0.003595, -0.003625, 0.999987]],
      half: [0.001991, 0.001991, 0.000010]
    },
  ],
];

/**
 * Pairs that are not worth checking, and why - the same reasoning MoveIt's setup
 * assistant uses. Built by sampling 3000 poses across the joint limits:
 *
 *   over 25% of poses  the boxes overlap by construction, not in fact. Adjacent
 *                      links always do, and link3's box reaches the wrist mount
 *                      at z=103mm where link4 and the tool are bolted on.
 *   never              they cannot reach each other anywhere in the joint range.
 *
 * That leaves two pairs carrying real information.
 *
 *   link0-link1  100.0%  overlaps by construction
 *   link0-link2    0.0%  never touches
 *   link0-link4    0.0%  never touches
 *   link0-link5    0.0%  never touches
 *   link1-link2  100.0%  overlaps by construction
 *   link1-link4    0.0%  never touches
 *   link1-link5    0.0%  never touches
 *   link2-link3  100.0%  overlaps by construction
 *   link2-link4    0.0%  never touches
 *   link2-link5    0.0%  never touches
 *   link3-link4  100.0%  overlaps by construction
 *   link3-link5   51.6%  overlaps by construction
 *   link4-link5    0.0%  never touches
 *
 *   link0-link3    0.4%  CHECKED
 *   link1-link3    0.9%  CHECKED
 *
 * The percentages above are for link5 as it stands here: a 4 mm stub sized for a
 * bare flange, which is why none of its pairs carry anything. A real tool is
 * added on top of this box at check time from `toolGeometry.ts`, and switches
 * the shoulder, upper arm and elbow pairs back on - see TOOL_PAIRS in
 * CollisionChecker.ts for the same table measured with a tool fitted.
 *
 * link3-link5 stays disabled either way, and the earlier note here had the
 * reason wrong. It said the pair was disabled only because the tool was a
 * placeholder and would matter most once something real was bolted on. Measuring
 * it says otherwise: the forearm's boxes stop about 75 mm past the flange,
 * because the third one encloses the wrist mount at z=103mm. The only region
 * where they can meet a tool is the region where they overlap it by
 * construction - 86% of poses in the first 25 mm, tool or no tool. Switching it
 * on would refuse nearly every pose and catch nothing.
 */
export const ALLOWED_PAIRS: ReadonlyArray<readonly [number, number]> = [
  [0, 1],
  [0, 2],
  [0, 4],
  [0, 5],
  [1, 2],
  [1, 4],
  [1, 5],
  [2, 3],
  [2, 4],
  [2, 5],
  [3, 4],
  [3, 5],
  [4, 5],
];

/**
 * Clearance added to every box, in metres.
 *
 * Two millimetres is the most the model tolerates: above about three, the wrist
 * parts - which genuinely sit that close - start reporting contact in poses the
 * arm holds happily. The matrix above is built from the TRUE geometry and this
 * margin applied only when checking; inflating first would disable the very
 * pairs the margin exists to protect.
 */
export const COLLISION_MARGIN_M = 0.002;
