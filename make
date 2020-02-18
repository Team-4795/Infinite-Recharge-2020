#!/bin/bash

if [ -d ~/wpilib/2020/jdk ]; then
    JAVA_HOME="$HOME/wpilib/2020/jdk"
elif [ -d /c/Users/Public/wpilib/2020/jdk ]; then
    JAVA_HOME="/c/Users/Public/wpilib/2020/jdk"
fi

JAVA_HOME=$JAVA_HOME ./gradlew $1
