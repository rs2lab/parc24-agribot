import enum

AGRIBOT_AGENT_NODE_NAME = "sunicv_agribot_agent"

PACKAGE_NAME = "parc24_agribot"

DEFAULT_QoS_PROFILE_VALUE = 10


class RotationType(enum.Enum):
    ANTICLOCKWISE = 1
    CLOCKWISE = -1
