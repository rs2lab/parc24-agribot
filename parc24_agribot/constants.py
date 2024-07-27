import enum

AGRIBOT_AGENT_NODE_NAME = "sunicv_agribot_agent"
AGRIBOT_BASE_NAVIGATOR_NAME = "sunicv_base_navigator"
AGRIBOT_YE_NODE_NAME = "sunicv_agribot_yield_detector"
AGRIBOT_CLOUD_SAVER_NODE_NAME = "sunicv_agribot_cloud_saver"

PACKAGE_NAME = "parc24_agribot"

DEFAULT_QoS_PROFILE_VALUE = 10


class RotationType(enum.Enum):
    ANTICLOCKWISE = 1
    CLOCKWISE = -1
