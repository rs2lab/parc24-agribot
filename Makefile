.SILENT: clean

PACKAGE_NAME = parc24_agribot
FOLDER_NAME = parc24-agribot

build: clean update-build

navigation: update-build
	-ros2 run $(PACKAGE_NAME) start_navigation_agent

cloud-saver: update-build
	-ros2 run $(PACKAGE_NAME) cloud_saver

yield-estimation: update-build
	-ros2 run $(PACKAGE_NAME) yield_estimator

cloud-to-laser: update-build
	-ros2 launch $(PACKAGE_NAME) pointcloud_to_laserscan_launch.py

img-capture: update-build
	-ros2 run $(PACKAGE_NAME) img_capture

clean:
	-@rm -rv ~/ros2_ws/build/$(PACKAGE_NAME)
	-@rm -rv ~/ros2_ws/install/$(PACKAGE_NAME)

update-build:
	@cd ~/ros2_ws/ && colcon build --paths ~/ros2_ws/src/$(FOLDER_NAME)
