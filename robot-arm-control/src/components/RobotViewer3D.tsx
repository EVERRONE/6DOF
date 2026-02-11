import React, { useEffect, useRef, useState, Suspense, useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera, Html, Line } from '@react-three/drei';
import * as THREE from 'three';
import { useRobotStore } from '../store/robotStore';
import {
  RobotModel3DBuilder,
  createWorkspaceBoundary,
  createTargetMarker,
  createGroundPlane,
  createGrid
} from '../viewer3d/RobotModel3D';
import { Robot3DModel } from '../viewer3d/types';

/**
 * Robot 3D Component
 * Loads and renders the robot with real-time FK updates
 */
const Robot3D: React.FC = () => {
  const { currentAngles } = useRobotStore();
  const [model, setModel] = useState<Robot3DModel | null>(null);
  const [loading, setLoading] = useState(true);
  const builderRef = useRef<RobotModel3DBuilder | null>(null);

  // Load robot meshes and build model
  useEffect(() => {
    let mounted = true;

    const loadRobot = async () => {
      try {
        const builder = new RobotModel3DBuilder();
        await builder.loadMeshes();

        if (!mounted) return;

        const robotModel = builder.buildModel();
        builderRef.current = builder;
        setModel(robotModel);
        setLoading(false);

        console.log('Robot 3D model loaded successfully');
      } catch (error) {
        console.error('Failed to load robot 3D model:', error);
        setLoading(false);
      }
    };

    loadRobot();

    return () => {
      mounted = false;
    };
  }, []);

  // Update robot pose when joint angles change
  useEffect(() => {
    if (builderRef.current && model) {
      const anglesArray = [
        currentAngles.J1,
        currentAngles.J2,
        currentAngles.J3,
        currentAngles.J4,
        currentAngles.J5,
        currentAngles.J6
      ];

      builderRef.current.updatePose(anglesArray);
    }
  }, [currentAngles, model]);

  if (loading) {
    return <LoadingIndicator />;
  }

  if (!model) {
    return <ErrorIndicator />;
  }

  return <primitive object={model.root} />;
};

/**
 * Workspace Boundary Visualization
 */
const WorkspaceBoundary: React.FC<{ visible: boolean }> = ({ visible }) => {
  const boundaryRef = useRef<THREE.Object3D>(null);

  useEffect(() => {
    const boundary = createWorkspaceBoundary(
      [-0.3, 0.3],  // X range: +/-300mm
      [-0.3, 0.3],  // Y range: +/-300mm
      [0, 0.4],     // Z range: 0-400mm
      0x00ff00,     // Green
      0.05          // Low opacity
    );

    boundaryRef.current = boundary;

    return () => {
      boundary.traverse((obj) => {
        if (obj instanceof THREE.Mesh) {
          obj.geometry.dispose();
          if (obj.material instanceof THREE.Material) {
            obj.material.dispose();
          }
        }
      });
    };
  }, []);

  if (!boundaryRef.current) return null;

  return <primitive object={boundaryRef.current} visible={visible} />;
};

/**
 * Target Position Marker
 */
const TargetMarker: React.FC = () => {
  const { targetPosition } = useRobotStore();
  const markerRef = useRef<THREE.Object3D>(null);

  useEffect(() => {
    const marker = createTargetMarker(0xff0000); // Red
    markerRef.current = marker;

    return () => {
      marker.traverse((obj) => {
        if (obj instanceof THREE.Mesh) {
          obj.geometry.dispose();
          if (obj.material instanceof THREE.Material) {
            obj.material.dispose();
          }
        }
      });
    };
  }, []);

  useEffect(() => {
    if (markerRef.current && targetPosition) {
      markerRef.current.position.set(
        targetPosition.x,
        targetPosition.y,
        targetPosition.z
      );
      markerRef.current.visible = true;
    } else if (markerRef.current) {
      markerRef.current.visible = false;
    }
  }, [targetPosition]);

  if (!markerRef.current) return null;

  return <primitive object={markerRef.current} />;
};

/**
 * Path Visualization - shows planned trajectory and waypoint markers in 3D
 */
const PathVisualization: React.FC<{ visible: boolean }> = ({ visible }) => {
  const { trajectoryPositions, waypoints } = useRobotStore();

  // Convert trajectory positions to THREE.Vector3 array for Line component
  const linePoints = useMemo(() => {
    if (!visible || trajectoryPositions.length < 2) return null;
    return trajectoryPositions.map(p => new THREE.Vector3(p.x, p.y, p.z));
  }, [trajectoryPositions, visible]);

  // Waypoint marker positions
  const waypointPositions = useMemo(() => {
    if (!visible) return [];
    return waypoints.map(wp => ({
      id: wp.id,
      position: new THREE.Vector3(wp.position.x, wp.position.y, wp.position.z),
      label: wp.label || ''
    }));
  }, [waypoints, visible]);

  if (!visible) return null;

  return (
    <group>
      {/* Trajectory path line */}
      {linePoints && linePoints.length >= 2 && (
        <Line
          points={linePoints}
          color="#00e676"
          lineWidth={2}
          dashed={false}
        />
      )}

      {/* Waypoint markers */}
      {waypointPositions.map((wp, index) => (
        <group key={wp.id} position={wp.position}>
          {/* Sphere marker */}
          <mesh>
            <sphereGeometry args={[0.008, 12, 12]} />
            <meshStandardMaterial
              color={
                index === 0 ? '#2196F3' :
                index === waypointPositions.length - 1 ? '#f44336' :
                '#4CAF50'
              }
              emissive={
                index === 0 ? '#2196F3' :
                index === waypointPositions.length - 1 ? '#f44336' :
                '#4CAF50'
              }
              emissiveIntensity={0.3}
            />
          </mesh>

          {/* Label */}
          <Html
            position={[0, 0.025, 0]}
            center
            distanceFactor={1}
            style={{ pointerEvents: 'none' }}
          >
            <div
              className="bg-white/90 px-1.5 py-0.5 rounded text-xs font-medium shadow whitespace-nowrap"
              style={{ fontSize: '10px' }}
            >
              #{index + 1} {wp.label}
            </div>
          </Html>
        </group>
      ))}
    </group>
  );
};

/**
 * Ground Plane and Grid
 */
const Ground: React.FC = () => {
  const planeRef = useRef<THREE.Mesh>(null);
  const gridRef = useRef<THREE.GridHelper>(null);

  useEffect(() => {
    const plane = createGroundPlane(2.0); // 2m x 2m
    planeRef.current = plane;

    const grid = createGrid(2.0, 40); // 2m, 40 divisions (50mm each)
    gridRef.current = grid;

    return () => {
      plane.geometry.dispose();
      if (plane.material instanceof THREE.Material) {
        plane.material.dispose();
      }
      grid.geometry.dispose();
      if (grid.material instanceof THREE.Material) {
        grid.material.dispose();
      }
    };
  }, []);

  return (
    <>
      {planeRef.current && <primitive object={planeRef.current} />}
      {gridRef.current && <primitive object={gridRef.current} />}
    </>
  );
};

/**
 * Scene Lighting
 */
const Lighting: React.FC = () => {
  return (
    <>
      {/* Ambient light for overall illumination */}
      <ambientLight intensity={0.4} />

      {/* Main directional light (sun) */}
      <directionalLight
        position={[5, 10, 5]}
        intensity={0.8}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-far={20}
        shadow-camera-left={-2}
        shadow-camera-right={2}
        shadow-camera-top={2}
        shadow-camera-bottom={-2}
      />

      {/* Fill light from opposite side */}
      <directionalLight position={[-5, 5, -5]} intensity={0.3} />

      {/* Hemisphere light for ambient color */}
      <hemisphereLight
        args={[0x87CEEB, 0x545454, 0.3]}
        position={[0, 50, 0]}
      />
    </>
  );
};

/**
 * Loading Indicator
 */
const LoadingIndicator: React.FC = () => {
  return (
    <mesh>
      <boxGeometry args={[0.1, 0.1, 0.1]} />
      <meshBasicMaterial color="#2196F3" wireframe />
    </mesh>
  );
};

/**
 * Error Indicator
 */
const ErrorIndicator: React.FC = () => {
  return (
    <mesh>
      <boxGeometry args={[0.1, 0.1, 0.1]} />
      <meshBasicMaterial color="#F44336" />
    </mesh>
  );
};

/**
 * Main RobotViewer3D Component
 */
export const RobotViewer3D: React.FC = () => {
  const [showWorkspace, setShowWorkspace] = useState(true);
  const [showPath, setShowPath] = useState(true);

  return (
    <div className="relative w-full h-full bg-gray-900">
      {/* 3D Canvas */}
      <Canvas
        shadows
        gl={{ antialias: true, alpha: false }}
        dpr={[1, 2]}
      >
        {/* Camera */}
        <PerspectiveCamera
          makeDefault
          position={[0.8, 0.6, 0.8]}
          fov={50}
        />

        {/* Orbit Controls */}
        <OrbitControls
          enableDamping
          dampingFactor={0.05}
          minDistance={0.3}
          maxDistance={3.0}
          maxPolarAngle={Math.PI / 2}
          target={[0, 0, 0.2]}
        />

        {/* Lighting */}
        <Lighting />

        {/* Ground and Grid */}
        <Ground />

        {/* Robot */}
        <Suspense fallback={<LoadingIndicator />}>
          <Robot3D />
        </Suspense>

        {/* Workspace Boundary */}
        <WorkspaceBoundary visible={showWorkspace} />

        {/* Target Marker */}
        <TargetMarker />

        {/* Path Visualization */}
        <PathVisualization visible={showPath} />

        {/* Coordinate Frame at Origin */}
        <axesHelper args={[0.1]} />
      </Canvas>

      {/* View Controls Overlay */}
      <div className="absolute top-4 right-4 bg-white p-3 rounded shadow-lg z-10">
        <h3 className="text-sm font-bold mb-2">View Options</h3>

        <div className="flex items-center gap-2 mb-1">
          <input
            type="checkbox"
            id="workspace-toggle"
            checked={showWorkspace}
            onChange={(e) => setShowWorkspace(e.target.checked)}
            className="w-4 h-4"
          />
          <label htmlFor="workspace-toggle" className="text-sm">
            Workspace
          </label>
        </div>

        <div className="flex items-center gap-2 mb-2">
          <input
            type="checkbox"
            id="path-toggle"
            checked={showPath}
            onChange={(e) => setShowPath(e.target.checked)}
            className="w-4 h-4"
          />
          <label htmlFor="path-toggle" className="text-sm">
            Path Preview
          </label>
        </div>

        <div className="text-xs text-gray-500 mt-3 border-t pt-2">
          <p className="font-semibold mb-1">Controls:</p>
          <p>Left drag: Rotate</p>
          <p>Right drag: Pan</p>
          <p>Scroll: Zoom</p>
        </div>
      </div>

      {/* Status Overlay */}
      <div className="absolute bottom-4 left-4 bg-white px-3 py-2 rounded shadow text-sm">
        <span className="text-gray-600">3D Viewer Active</span>
      </div>
    </div>
  );
};
