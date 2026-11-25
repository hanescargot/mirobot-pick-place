# Glossary

## FK (Forward Kinematics)
- **Definition**: Joint angle을 알 때 end-effector(robot 손끝)의 position과 orientation을 계산하는 것
- **Input**: Joint angle (매번 바뀌는 variable) + link length (robot마다 고정된 constant, spec이나 kinematic calibration으로 구함)
- **Analogy**: 어깨와 팔꿈치를 몇 도 굽혔는지 알면, 눈을 감고도 손이 어디 있는지 계산할 수 있음
- **Example** (2-link planar arm):
  ```
  x = L1·cos(θ1) + L2·cos(θ1 + θ2)
  y = L1·sin(θ1) + L2·sin(θ1 + θ2)
  ```
- **Note**: Solution이 항상 하나로 정해져서 계산이 쉬움. 3D robot은 joint마다 homogeneous transformation matrix를 곱해서 계산
- **Related**: IK, DH parameter, homogeneous transformation matrix

## IK (Inverse Kinematics)
- **Definition**: End-effector의 target position과 orientation이 주어질 때, 거기에 도달하기 위한 joint angle을 계산하는 것
- **Analogy**: 컵을 잡으려고 할 때, 어깨와 팔꿈치를 각각 몇 도 굽혀야 하는지 거꾸로 알아내는 것
- **Example**: `arm.set_tool_pose(200, 20, 230)`을 call하면 Mirobot firmware 내부에서 IK로 6개 joint angle을 계산
- **Note**: Solution이 여러 개(예: 팔꿈치를 위로 vs 아래로)이거나, workspace 밖이면 solution이 없을 수 있어서 FK보다 어려움
- **Related**: FK, workspace, singularity
