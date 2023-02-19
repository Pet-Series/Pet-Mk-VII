# Pet-Mk-VII

<h1 align="center">Welcome to the Pet-Mk-VII repository</h1>
<h1 align="center">The ROS2 Ackermann Vehicle/Robot</h1>

ROS repository in the https://github.com/Pet-Series Git-Organizations.</br>
Containing multiply ROS1/ROS2-packages.

# Software/Setup Pet series micro robots #
The main objective/scope for this repository is to control the part of the software (and parameter settings) that is unique for this Pet Mark VII (seven) robot. A.k.a "The Ackermann".

ROS2 (Robot Operating System v2) is used as middleware.
Ubuntu is used as operating system.

# The journey is the goal
<table>
    <tr>Pet-Mk.VIII early iterations
      <td>..image 1...<br><img src="./doc/Pet-Mk.VII_build_phase_00(design_iterations).png" width="400px"></td>
      <td>..image 1...<br><img src="./doc/Pet-Mk.VII_build_phase_01(dummy).png" width="400px"></td>
    </tr>
</table>

# SETUP ROS2 WORKSPACE
## Create source directory
`Ubuntu Shell`
```
~$ mkdir -p ~/ws_ros2/src
~$ cd ~/ws_ros2/src
```
## Clone git repositories
`Ubuntu Shell`
```
~/ws_ros2/src$ git clone https://github.com/Pet-Series/Pet-Mk-VII
   ...
~/ws_ros2/src$ git clone https://github.com/Pet-Series/pet_ros2_lightbeacon_pkg.git
   ...
~/ws_ros2/src$ git clone https://github.com/Pet-Series/pet_ros2_ir_remote_pkg.git
   ...
~/ws_ros2/src$ git clone https://github.com/Pet-Series/pet_ros2_lcd_pkg.git
   ...
~/ws_ros2/src$
```
