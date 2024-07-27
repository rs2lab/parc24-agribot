.SILENT: clean

PACKAGE_NAME = parc24_agribot
FOLDER_NAME = parc24-agribot
NAVIGATION_CMD = start_navigation_agent

build: clean update_build

navigation: update_build
	-ros2 run $(PACKAGE_NAME) $(NAVIGATION_CMD)

clean:
	-@rm -rv ~/ros2_ws/build/$(PACKAGE_NAME)
	-@rm -rv ~/ros2_ws/install/$(PACKAGE_NAME)

update_build:
	@cd ~/ros2_ws/ && colcon build --paths ~/ros2_ws/src/$(FOLDER_NAME)
