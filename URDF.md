<robot name="666" version="1.0">
  <joint name="Joint2" type="revolute">
    <limit lower="-1.30000" upper="1.30000" effort="12.00000" velocity="1.60000"/>
    <origin rpy="-1.57080 0.00000 0.00000" xyz="-0.03750 0.02000 0.05595"/>
    <parent link="link0"/>
    <child link="link1"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <joint name="Joint3" type="revolute">
    <limit lower="-2.10000" upper="2.10000" effort="10.00000" velocity="2.20000"/>
    <origin rpy="-3.14159 0.00000 0.00000" xyz="0.00027 -0.16000 0.01600"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <joint name="Joint5" type="revolute">
    <limit lower="-2.50000" upper="2.50000" effort="5.00000" velocity="3.50000"/>
    <origin rpy="-1.57080 1.57080 0.00000" xyz="0.00000 -0.01002 0.10323"/>
    <parent link="link3"/>
    <child link="link4"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <joint name="Joint6" type="continuous">
    <limit lower="0" upper="0" effort="4.00000" velocity="4.00000"/>
    <origin rpy="1.57080 0.00000 -1.57080" xyz="-0.02677 -0.00000 0.00994"/>
    <parent link="link4"/>
    <child link="link5"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <joint name="Joint_B" type="fixed">
    <origin rpy="0.00000 0.00000 0.00000" xyz="-0.11486 0.00000 0.00000"/>
    <parent link="linkB.000"/>
    <child link="linkB"/>
  </joint>
  <joint name="link0_joint" type="revolute">
    <limit lower="-2.80000" upper="2.80000" effort="10.00000" velocity="2.00000"/>
    <origin rpy="0.00000 0.00000 0.00000" xyz="0.00000 0.00000 0.08000"/>
    <parent link="linkB"/>
    <child link="link0"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <joint name="link3_joint" type="revolute">
    <limit lower="-2.50000" upper="2.50000" effort="6.00000" velocity="3.00000"/>
    <origin rpy="-1.57080 0.00000 1.57080" xyz="-0.03500 0.01510 0.03640"/>
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="0.00000 0.00000 1.00000"/>
  </joint>
  <link name="link0">
    <collision name="BaseWall_2_v2_collision">
      <origin rpy="1.57080 0.00000 1.57080" xyz="0.00250 -0.00750 0.05600"/>
      <geometry>
        <box size="0.05499 0.07999 0.15999"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="-0.00000" ixz="0.00000" iyy="0.00000" iyz="-0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="-0.01493 0.00121 0.04837"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="BaseWall_2_v2">
      <origin rpy="1.57080 0.00000 1.57080" xyz="-0.03750 0.02000 0.05595"/>
      <geometry>
        <mesh filename="../meshes/stl/BaseWall_2_v2.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="link1">
    <collision name="Arm1v2_collision">
      <origin rpy="0.00000 0.00000 3.14159" xyz="0.00016 -0.07500 0.03100"/>
      <geometry>
        <box size="0.06167 0.21000 0.03000"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="-0.00000" ixz="-0.00000" iyy="0.00000" iyz="-0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="0.00023 -0.06967 0.02643"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="Arm1v2">
      <origin rpy="0.00000 0.00000 3.14159" xyz="-0.00000 -0.00000 0.01600"/>
      <geometry>
        <mesh filename="../meshes/stl/Arm1v2.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="link2">
    <collision name="Arm2Mountv2_collision">
      <origin rpy="-1.83292 -1.57080 1.83292" xyz="0.00400 -0.00500 0.03500"/>
      <geometry>
        <box size="0.04600 0.08000 0.07800"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="0.00000" ixz="0.00000" iyy="0.00000" iyz="0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="0.00456 0.00814 0.03271"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="Arm2Mountv2">
      <origin rpy="-1.83292 -1.57080 1.83292" xyz="0.00000 0.00000 0.01200"/>
      <geometry>
        <mesh filename="../meshes/stl/Arm2Mountv2.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="link3">
    <collision name="Rotation arm V3_collision">
      <origin rpy="-0.00000 -0.00000 1.57080" xyz="-0.00000 0.02011 0.06100"/>
      <geometry>
        <box size="0.01000 0.03500 0.12200"/>
      </geometry>
    </collision>
    <collision name="Rotation arm V3_collision.000">
      <origin rpy="-0.00000 -0.00000 1.57080" xyz="-0.00000 -0.02415 0.06100"/>
      <geometry>
        <box size="0.02000 0.03500 0.12200"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="-0.00000" ixz="0.00000" iyy="0.00000" iyz="-0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="0.00001 0.00089 0.04283"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="Rotation arm V3">
      <origin rpy="-0.00000 -0.00000 1.57080" xyz="0.00000 0.00000 0.00000"/>
      <geometry>
        <mesh filename="../meshes/stl/Rotation arm V3.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="link4">
    <collision name="J6 housing_collision">
      <origin rpy="-1.30867 -1.57080 -1.83292" xyz="-0.00500 -0.00000 0.01000"/>
      <geometry>
        <box size="0.02800 0.02200 0.03000"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="-0.00000" ixz="0.00000" iyy="0.00000" iyz="-0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="-0.00712 0.00000 0.01000"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="J6 housing">
      <origin rpy="-1.30867 -1.57080 -1.83292" xyz="-0.02000 -0.00000 0.00994"/>
      <geometry>
        <mesh filename="../meshes/stl/J6 housing.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="link5">
    <collision name="Cylinder_collision">
      <origin rpy="3.14159 0.00000 3.14159" xyz="-0.00000 0.00000 -0.00427"/>
      <geometry>
        <box size="0.00400 0.00400 0.00500"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="-0.00000" ixz="-0.00000" iyy="0.00000" iyz="0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="-0.00000 0.00000 -0.00427"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="Cylinder">
      <origin rpy="3.14159 0.00000 3.14159" xyz="0.00000 0.00000 -0.00677"/>
      <geometry>
        <mesh filename="../meshes/stl/Cylinder.stl" scale="0.00200 0.00200 0.00250"/>
      </geometry>
    </visual>
  </link>
  <link name="linkB">
    <collision name="Baseplate_collision">
      <origin rpy="1.57080 0.00000 0.00000" xyz="0.03381 0.00011 0.04000"/>
      <geometry>
        <box size="0.18750 0.08000 0.12000"/>
      </geometry>
    </collision>
    <inertial>
      <inertia ixx="0.00000" ixy="0.00000" ixz="-0.00000" iyy="0.00000" iyz="0.00000" izz="0.00000"/>
      <origin rpy="0.00000 0.00000 0.00000" xyz="0.01683 0.00007 0.03368"/>
      <mass value="0.00100"/>
    </inertial>
    <visual name="Baseplate">
      <origin rpy="1.57080 0.00000 0.00000" xyz="0.00000 0.00000 0.08000"/>
      <geometry>
        <mesh filename="../meshes/stl/Baseplate.001.stl" scale="1.00000 1.00000 1.00000"/>
      </geometry>
    </visual>
  </link>
  <link name="linkB.000"/>
</robot>