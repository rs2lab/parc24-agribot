from .posenode import PoseNode, Position, Orientation

import pandas as pd
import os


def d(x: float, y: float) -> float:
    return (x**2 + y**2) ** 0.5


class PathMap:
    def __init__(self, poses: list[PoseNode] = []) -> None:
        self.poses = poses

    @classmethod
    def load_path_map(cls, path: str):
        orientations = pd.read_csv(os.path.join(path, "path_orientations.csv"))
        positions = pd.read_csv(os.path.join(path, "path_positions.csv"))
        assert len(orientations) == len(positions)
        poses = []
        for i in range(positions):
            position = positions[0]
            orientation = orientations[0]
            pose = PoseNode(
                position=Position(
                    x=position["x"],
                    y=position["y"],
                    z=position["z"],
                    timestamp=position["timestamp"],
                ),
                orientation=Orientation(
                    x=orientation["x"],
                    y=orientation["y"],
                    z=orientation["z"],
                    w=orientation["w"],
                    timestamp=orientation["timestamp"],
                ),
            )
            poses.append(pose)
        return PathMap(poses)

    @property
    def poses(self) -> list:
        return self.poses
