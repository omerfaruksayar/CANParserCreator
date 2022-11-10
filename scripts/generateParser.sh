#!/bin/sh

db_name=$1
dbc_path=$2
pckg_path=$3
msg_name=$4
sbs_topic=$5
pbs_topic=$6

. env/bin/activate
cd $pckg_path
ros2 pkg create --build-type ament_cmake $db_name
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
cd msg
touch $msg_name.msg
cd ..
sed -i '11 a \  <depend>rclcpp</depend>\n  <build_depend>rosidl_default_generators</build_depend>\n  <exec_depend>rosidl_default_runtime</exec_depend>\n  <member_of_group>rosidl_interface_packages</member_of_group>' package.xml

sed -i '10 a find_package(rosidl_default_generators REQUIRED)\nfind_package(rclcpp REQUIRED)\n\nadd_executable('$db_name'_node src/'$db_name'_parser.cpp src/'$db_name'.cpp)\nament_target_dependencies('$db_name'_node rclcpp)\n\ninstall(TARGETS\n  '$db_name'_node\n  DESTINATION lib/${PROJECT_NAME})\n\nrosidl_generate_interfaces(${PROJECT_NAME}\n  "msg/'$msg_name'.msg"\n  DEPENDENCIES std_msgs)' CMakeLists.txt

deactivate