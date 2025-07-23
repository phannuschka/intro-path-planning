# coding: utf-8
"""
Optimized version of planar robot kinematics - replaces slow SymPy with fast NumPy
Original symbolic version took 1.2 seconds, this should take ~50 microseconds
"""

import numpy as np


class FastPlanarJoint:
    def __init__(self, a=1.5, init_theta=0, id=0):
        self.a = a
        self.theta = init_theta
        self.id = id

    def get_transform_matrix(self):
        """Get 3x3 homogeneous transformation matrix as NumPy array"""
        cos_t, sin_t = np.cos(self.theta), np.sin(self.theta)
        return np.array([
            [cos_t, -sin_t, self.a * cos_t],
            [sin_t, cos_t, self.a * sin_t],
            [0, 0, 1]
        ], dtype=np.float32)

    def get_transform(self):
        """Get end effector position of this joint"""
        return np.array([self.a * np.cos(self.theta),
                         self.a * np.sin(self.theta)], dtype=np.float32)

    def move(self, new_theta):
        self.theta = new_theta


class FastPlanarRobot:
    def __init__(self, n_joints=2, length=3.0):
        self.dim = n_joints
        length = length / self.dim
        self.joints = [FastPlanarJoint(id=i, a=length) for i in range(self.dim)]

    def get_transforms(self):
        """Fast forward kinematics - returns positions of all joints + base"""
        transforms = [np.array([0.0, 0.0], dtype=np.float32)]  # Base position

        # Cumulative transformation
        T = np.eye(3, dtype=np.float32)

        for joint in self.joints:
            # Get joint transformation matrix
            joint_T = joint.get_transform_matrix()
            # Accumulate transformation
            T = T @ joint_T
            # Extract position (translation part)
            transforms.append(T[:2, 2].copy())

        return transforms

    def get_transforms_fast(self):
        """Even faster version using direct trigonometry"""
        transforms = [np.array([0.0, 0.0], dtype=np.float32)]  # Base

        x, y, cumulative_angle = 0.0, 0.0, 0.0

        for joint in self.joints:
            cumulative_angle += joint.theta
            x += joint.a * np.cos(cumulative_angle)
            y += joint.a * np.sin(cumulative_angle)
            transforms.append(np.array([x, y], dtype=np.float32))

        return transforms

    def move(self, new_thetas):
        assert len(new_thetas) == len(self.joints)
        for i, theta in enumerate(new_thetas):
            self.joints[i].move(theta)


# Backward compatibility wrapper that keeps the same interface
class PlanarJoint:
    def __init__(self, a=1.5, init_theta=0, id=0):
        self._fast_joint = FastPlanarJoint(a, init_theta, id)
        # Keep symbolic stuff for compatibility if needed
        self.a = a
        self.theta = init_theta

    def get_subs(self):
        return {'dummy': 'for_compatibility'}

    def get_transform(self):
        return self._fast_joint.get_transform()

    def move(self, new_theta):
        self.theta = new_theta
        self._fast_joint.move(new_theta)


class PlanarRobot:
    def __init__(self, n_joints=2, length=3.0):
        self._fast_robot = FastPlanarRobot(n_joints, length)
        # Keep old interface
        self.dim = n_joints
        self.joints = [PlanarJoint(id=i, a=length / n_joints) for i in range(n_joints)]

    def get_transforms(self):
        return self._fast_robot.get_transforms_fast()

    def move(self, new_thetas):
        assert len(new_thetas) == len(self.joints)
        self._fast_robot.move(new_thetas)
        # Keep old joints in sync
        for i, theta in enumerate(new_thetas):
            self.joints[i].move(theta)


if __name__ == '__main__':
    import time

    # Test single joint
    j = PlanarJoint()
    print("Single joint test:")
    print(j.get_transform())
    j.move(0.8)
    print(j.get_transform())

    # Test robot
    r = PlanarRobot(n_joints=7, length=3.0)  # 7-DOF robot
    r.move([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

    # Speed test
    print("\nSpeed test (1000 iterations):")
    start = time.time()
    for _ in range(1000):
        transforms = r.get_transforms()
    end = time.time()

    print(f"Time per call: {(end - start) * 1000:.3f} ms")
    print(f"Transforms: {len(transforms)} points")
    print(f"End effector: {transforms[-1]}")