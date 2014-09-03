#!/bin/bash

# Creates a new project based on the template
# Usage: ./create-project.sh [PROJECT_NAME]

# check if there is an argument
if [ -z "$1" ]
then
	echo "Usage: ./create-project.sh [PROJECT_NAME]"
	exit
fi

PROJECT="$1"
echo "Creating project: $PROJECT..."

# copy over template directory
cp -r _template $PROJECT

# update directory structure
mv $PROJECT/src/pl/dagothar/template $PROJECT/src/pl/dagothar/$PROJECT

# update source files
sed -i "s/template/$PROJECT/g" $PROJECT/src/pl/dagothar/$PROJECT/Main.java
sed -i "s/template/$PROJECT/g" $PROJECT/src/pl/dagothar/$PROJECT/MainWindow.java
sed -i "s/template/$PROJECT/g" $PROJECT/src/pl/dagothar/$PROJECT/MainMenu.java

# update build file
sed -i "s/template/$PROJECT/g" $PROJECT/build.xml

echo "Done."
