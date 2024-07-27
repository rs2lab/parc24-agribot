from .posenode import PoseNode, Position, Orientation

import pandas as pd
import os


class PathMap:
    def __init__(self, poses: list[PoseNode] = []) -> None:
        self._poses = poses

    @classmethod
    def load_path_map(cls, path: str):
        orientations = pd.read_csv(os.path.join(path, "path_orientations.csv")).to_numpy()
        positions = pd.read_csv(os.path.join(path, "path_positions.csv")).to_numpy()
        assert len(orientations) == len(positions)
        poses = []
        for i in range(len(positions)):
            position = positions[i]
            orientation = orientations[i]
            pose = PoseNode(
                position=Position(
                    x=position[1],
                    y=position[2],
                    z=position[3],
                    timestamp=position[4],
                ),
                orientation=Orientation(
                    x=orientation[1],
                    y=orientation[2],
                    z=orientation[3],
                    w=orientation[4],
                    timestamp=orientation[5],
                ),
            )
            poses.append(pose)
        return PathMap(poses)

    @property
    def poses(self) -> list:
        return self._poses
