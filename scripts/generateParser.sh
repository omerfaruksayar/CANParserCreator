#!/bin/sh

db_name=$1
dbc_path=$2
pckg_path=$3
sbs_topic=$4

. env/bin/activate
cd $pckg_path
catkin create pkg $db_name --catkin-deps std_msgs roscpp can_msgs message_generation
cd $db_name/include
rm -rf $db_name
cd ../src
touch $db_name'_parser.cpp'
chmod +x $db_name'_parser.cpp'
cantools generate_c_source --database-name $db_name $dbc_path
mv $db_name.c $db_name.cpp
mv $db_name.h ../include
cd ..
mkdir launch
cd launch
touch $db_name'_parser.launch'
echo '<launch>
   <node name="'$db_name'_node" pkg="'$db_name'" type="'$db_name'_node" />
</launch>' >> $db_name'_parser.launch'
cd ..
mkdir msg
sed -i '17,209d' CMakeLists.txt
echo 'add_message_files(
   FILES
)' >> CMakeLists.txt

 echo "generate_messages(
   DEPENDENCIES
   can_msgs
   std_msgs
)" >> CMakeLists.txt

 echo "catkin_package(
  INCLUDE_DIRS include
  LIBRARIES $db_name
  CATKIN_DEPENDS can_msgs roscpp std_msgs
#  DEPENDS system_lib
)" >> CMakeLists.txt

echo 'include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)' >> CMakeLists.txt

echo 'add_library(${PROJECT_NAME}
   src/'$db_name'.cpp
)' >> CMakeLists.txt

echo 'add_executable('$db_name'_node src/'$db_name'_parser.cpp src/'$db_name'.cpp)' >> CMakeLists.txt

echo 'add_dependencies('$db_name'_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})' >> CMakeLists.txt

echo 'target_link_libraries('$db_name'_node
   ${catkin_LIBRARIES}
)' >> CMakeLists.txt

echo 'install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)' >> CMakeLists.txt

echo 'install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)' >> CMakeLists.txt

deactivate