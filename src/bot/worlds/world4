<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='my_bot'>
      <link name='base_link'>
        <inertial>
          <pose>0.173469 0 0.381361 0 -0 0</pose>
          <mass>29.4</mass>
          <inertia>
            <ixx>2.14622</ixx>
            <ixy>0</ixy>
            <ixz>0.773939</ixz>
            <iyy>5.30715</iyy>
            <iyz>0</iyz>
            <izz>5.76718</izz>
          </inertia>
        </inertial>
        <collision name='base_link_fixed_joint_lump__chassis_collision'>
          <pose>0 0 0.45 0 -0 0</pose>
          <geometry>
            <box>
              <size>1 0.8 0.5</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='base_link_fixed_joint_lump__camera_frame_collision_1'>
          <pose>0.5 0 0.71 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.05 0.1 0.02</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='base_link_fixed_joint_lump__caster_collision_2'>
          <pose>0.25 0 0.2 0 -0 0</pose>
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>0.001</mu>
                <mu2>0.001</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='base_link_fixed_joint_lump__fork_collision_3'>
          <pose>0.8 0 0.225 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.6 0.7 0.05</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='base_link_fixed_joint_lump__laser_frame_collision_4'>
          <pose>0 0 0.725 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.05</length>
              <radius>0.05</radius>
            </cylinder>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='base_link_fixed_joint_lump__chassis_visual'>
          <pose>0 0 0.45 0 -0 0</pose>
          <geometry>
            <box>
              <size>1 0.8 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/SkyBlue</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <visual name='base_link_fixed_joint_lump__camera_frame_visual_1'>
          <pose>0.5 0 0.71 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.05 0.1 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Black</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <visual name='base_link_fixed_joint_lump__caster_visual_2'>
          <pose>0.25 0 0.2 0 -0 0</pose>
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Black</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <visual name='base_link_fixed_joint_lump__fork_visual_3'>
          <pose>0.8 0 0.225 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.6 0.7 0.05</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <visual name='base_link_fixed_joint_lump__laser_frame_visual_4'>
          <pose>0 0 0.725 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.05</length>
              <radius>0.05</radius>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Red</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <sensor name='camera' type='camera'>
          <visualize>1</visualize>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>1.089</horizontal_fov>
            <image>
              <format>R8G8B8</format>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>8</far>
            </clip>
          </camera>
          <plugin name='camera_controller' filename='libgazebo_ros_camera.so'>
            <frame_name>camera_frame</frame_name>
          </plugin>
          <pose>0.5 0 0.71 0 -0 0</pose>
        </sensor>
        <sensor name='laser' type='ray'>
          <visualize>1</visualize>
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <min_angle>-3.14</min_angle>
                <max_angle>3.14</max_angle>
                <resolution>1</resolution>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.3</min>
              <max>12</max>
            </range>
          </ray>
          <plugin name='laser_controller' filename='libgazebo_ros_ray_sensor.so'>
            <ros>
              <argument>--ros-args --remap ~/out:=scan</argument>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>laser_frame</frame_name>
          </plugin>
          <pose>0 0 0.725 0 -0 0</pose>
        </sensor>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <joint name='left_wheel_joint' type='revolute'>
        <pose relative_to='base_link'>-0.2 0.45 0.2 -1.5707 0 0</pose>
        <parent>base_link</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1e+16</lower>
            <upper>1e+16</upper>
          </limit>
          <dynamics>
            <spring_reference>0</spring_reference>
            <spring_stiffness>0</spring_stiffness>
          </dynamics>
        </axis>
      </joint>
      <link name='left_wheel'>
        <pose relative_to='left_wheel_joint'>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>4</mass>
          <inertia>
            <ixx>0.08</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0433333</iyy>
            <iyz>0</iyz>
            <izz>0.0433333</izz>
          </inertia>
        </inertial>
        <collision name='left_wheel_collision'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.2</radius>
            </cylinder>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='left_wheel_visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.2</radius>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Red</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <joint name='right_wheel_joint' type='revolute'>
        <pose relative_to='base_link'>-0.2 -0.45 0.2 -1.5707 0 0</pose>
        <parent>base_link</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit>
            <lower>-1e+16</lower>
            <upper>1e+16</upper>
          </limit>
          <dynamics>
            <spring_reference>0</spring_reference>
            <spring_stiffness>0</spring_stiffness>
          </dynamics>
        </axis>
      </joint>
      <link name='right_wheel'>
        <pose relative_to='right_wheel_joint'>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>4</mass>
          <inertia>
            <ixx>0.08</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0433333</iyy>
            <iyz>0</iyz>
            <izz>0.0433333</izz>
          </inertia>
        </inertial>
        <collision name='right_wheel_collision'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.2</radius>
            </cylinder>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <friction>
              <ode/>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='right_wheel_visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <cylinder>
              <length>0.1</length>
              <radius>0.2</radius>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Red</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>0</static>
      <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.9</wheel_separation>
        <wheel_diameter>0.2</wheel_diameter>
        <max_wheel_torque>200</max_wheel_torque>
        <max_wheel_acceleration>10.0</max_wheel_acceleration>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <publish_odom>1</publish_odom>
        <publish_odom_tf>1</publish_odom_tf>
        <publish_wheel_tf>1</publish_wheel_tf>
      </plugin>
      <pose>0 0 0 0 -0 0</pose>
    </model>
    <model name='Map3'>
      <pose>4.4782 -2.78831 0 0 -0 0</pose>
      <link name='Wall_1'>
        <collision name='Wall_1_Collision'>
          <geometry>
            <box>
              <size>40 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_1_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>40 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>0.215224 -19.5317 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_11'>
        <collision name='Wall_11_Collision'>
          <geometry>
            <box>
              <size>28.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_11_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>28.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>6.0569 -8.66282 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_13'>
        <collision name='Wall_13_Collision'>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_13_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>17.7394 18.4817 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_14'>
        <collision name='Wall_14_Collision'>
          <geometry>
            <box>
              <size>2.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_14_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>18.9144 17.4317 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_16'>
        <collision name='Wall_16_Collision'>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_16_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-19.4652 -18.1327 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_17'>
        <collision name='Wall_17_Collision'>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_17_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-18.7902 -18.8077 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_19'>
        <collision name='Wall_19_Collision'>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_19_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-2.41027 0.053807 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_2'>
        <collision name='Wall_2_Collision'>
          <geometry>
            <box>
              <size>39 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_2_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>39 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>20.1402 -0.106693 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_20'>
        <collision name='Wall_20_Collision'>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_20_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-1.36027 -1.24619 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_21'>
        <collision name='Wall_21_Collision'>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_21_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-2.41027 -2.54619 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_22'>
        <collision name='Wall_22_Collision'>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_22_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-3.46027 -1.24619 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_3'>
        <collision name='Wall_3_Collision'>
          <geometry>
            <box>
              <size>40.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_3_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>40.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>0.090224 19.3183 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_4'>
        <collision name='Wall_4_Collision'>
          <geometry>
            <box>
              <size>39 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_4_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>39 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-19.9598 -0.106693 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_6'>
        <collision name='Wall_6_Collision'>
          <geometry>
            <box>
              <size>31 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_6_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>31 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-4.39239 7.6943 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_7'>
        <collision name='Wall_7_Collision'>
          <geometry>
            <box>
              <size>4 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_7_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>4 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>11.0326 5.76931 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_8'>
        <collision name='Wall_8_Collision'>
          <geometry>
            <box>
              <size>4.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_8_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>4.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>8.85761 3.8443 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_9'>
        <collision name='Wall_9_Collision'>
          <geometry>
            <box>
              <size>4 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_9_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>4 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.760784 0.662745 0.627451 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>6.68261 5.76931 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
    </model>
    <state world_name='default'>
      <sim_time>167 117000000</sim_time>
      <real_time>175 535368303</real_time>
      <wall_time>1706086044 211727253</wall_time>
      <iterations>167117</iterations>
      <model name='Map3'>
        <pose>4.4782 -2.78831 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='Wall_1'>
          <pose>4.69342 -22.32 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_11'>
          <pose>10.5351 -11.4511 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_13'>
          <pose>22.2176 15.6934 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_14'>
          <pose>23.3926 14.6434 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_16'>
          <pose>-14.987 -20.921 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_17'>
          <pose>-14.312 -21.596 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_19'>
          <pose>2.06793 -2.7345 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_2'>
          <pose>24.6184 -2.895 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_20'>
          <pose>3.11793 -4.0345 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_21'>
          <pose>2.06793 -5.3345 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_22'>
          <pose>1.01793 -4.0345 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_3'>
          <pose>4.56842 16.53 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_4'>
          <pose>-15.4816 -2.895 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_6'>
          <pose>0.08581 4.90599 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_7'>
          <pose>15.5108 2.981 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_8'>
          <pose>13.3358 1.05599 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_9'>
          <pose>11.1608 2.981 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='my_bot'>
        <pose>0.19485 0.013229 2e-06 0 7e-06 0.013693</pose>
        <scale>1 1 1</scale>
        <link name='base_link'>
          <pose>0.19485 0.013229 2e-06 0 7e-06 0.013693</pose>
          <velocity>0.002494 8.8e-05 -0.000968 2.9e-05 -0.003874 8.6e-05</velocity>
          <acceleration>-0.300886 -0.020332 -0.592373 -0.110792 -1.46697 0.04374</acceleration>
          <wrench>-8.84606 -0.597773 -17.4158 0 -0 0</wrench>
        </link>
        <link name='left_wheel'>
          <pose>-0.011291 0.460448 0.200003 -1.5707 -0.06518 0.013687</pose>
          <velocity>0.001656 8.8e-05 -0.001765 0.000273 -0.003871 -3.8e-05</velocity>
          <acceleration>1.05213 0.033652 -3.53073 0.587853 -0.677166 -0.267464</acceleration>
          <wrench>4.20851 0.134608 -14.1229 0 -0 0</wrench>
        </link>
        <link name='right_wheel'>
          <pose>0.001032 -0.439468 0.200003 -1.5707 -0.041296 0.013689</pose>
          <velocity>0.001744 9e-05 -0.001731 0.000268 -0.003871 7e-06</velocity>
          <acceleration>1.0791 0.035509 -3.46283 0.579136 -0.965548 -0.168533</acceleration>
          <wrench>4.31642 0.142035 -13.8513 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-75.5129 24.4913 53.1895 0 0.534798 -0.311998</pose>
        <view_controller>ortho</view_controller>
        <projection_type>orthographic</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
