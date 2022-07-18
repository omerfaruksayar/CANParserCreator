# CANParserCreator for ROS1

## Requeriments

```
    catkin_tools 0.9.0
    python3
```

## About
A tool for parsing CAN DBCs using cantools (https://cantools.readthedocs.io/en/latest/)
Uses the header and source file and generates a ros package that contains a rosnode which subscribes to given topic name and publish the ros message is created by the tool to another given topic name.

## Usage

Program takes 6 parameters, 
package name = Name of the package will be created in the given WS path
dbc path = path for dbc file containing .dbc file
package path = Path for WS/src
package message name (optional) = Message file name will be created in the package
subscribing topic name = Topic for can data
publisher topic name = Topic name to publish our message    

```
python3 canparsercreator.py <package name> <dbc path> <package path> <package message name> <subscribing topic name for can messages> <publisher topic name>
```