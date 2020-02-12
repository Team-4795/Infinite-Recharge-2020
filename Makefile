GRADLEW=JAVA_HOME='/home/eastbots/wpilib/2020/jdk' ./gradlew

help:
	@echo 'make build  - check and compile the source code'
	@echo 'make deploy - deploy the code to the robot'

gradlew:
	$(GRADLEW) $(TASK)
build:
	$(GRADLEW) build
deploy:
	$(GRADLEW) deploy


.PHONY: help gradlew build deploy
