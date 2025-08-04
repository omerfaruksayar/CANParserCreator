# Program takes 4 parameters,
# package name = Name of the package will be created in the given WS path
# dbc path = path for dbc file containing .dbc file
# package path = Path for WS/src
# subscribing topic name = Topic for can data
# Usage: python3 canparsercreator.py <package name> <dbc path> <package path> <subscribing topic name for can messages>

from curses.ascii import isupper
import re
import sys
import os
import json
import cantools


class SignalGroupStruct:
    signals = []

    def __init__(self, name, id, header):
        self.name = name
        self.frameId = id
        self.header = header

    def printSGC(self):
        print(self.name, self.frameId)
        print("Signals: ")
        for s in self.signals:
            print(s)


def to_snake_case(name):
    """
    Convert signal/message name to match cantools C output style.
    Handles:
      - CamelCase / PascalCase → snake_case
      - Digit-acronym like 'R2D' stays together
      - No underscore before digits, always underscore after digits
    """
    # Step 0: Always underscore AFTER a digit if followed by letter
    name = re.sub(r'(?<=\d)(?=[A-Za-z])', '_', name)
    # Step 1: Lowercase-to-Uppercase transition
    name = re.sub(r'(?<=[a-z])(?=[A-Z])', '_', name)
    # Step 2: Uppercase-to-Uppercase+lowercase transition
    name = re.sub(r'(?<=[A-Z])(?=[A-Z][a-z])', '_', name)
    # Step 3: Normalize multiple underscores
    name = re.sub(r'__+', '_', name)

    return name.lower()


def load_messages_from_dbc(dbc_path, filter_file_path):
    db = cantools.database.load_file(dbc_path)

    with open(filter_file_path) as f:
        filter_data = json.load(f)
        filter_list = filter_data.get(
            "inbound", []) + filter_data.get("outbound", [])
        filter_list_snake = [to_snake_case(name) for name in filter_list]

    structs = []
    for msg in db.messages:
        name_snake = to_snake_case(msg.name)
        if filter_list and name_snake not in filter_list_snake:
            continue

        sg = SignalGroupStruct(name=f"{name_snake}_t", id=hex(
            msg.frame_id), header=f"{name_snake.upper()}_FRAME_ID")
        sg.signals = [to_snake_case(sig.name) for sig in msg.signals]
        structs.append(sg)

    return structs


# Determines the ROS type based on the signal properties
def get_ros_type(sig):
    """Determine the most appropriate ROS type for a cantools signal."""
    # Treat 1-bit unsigned as bool
    if sig.length == 1 and not sig.is_signed:
        return "bool"

    # Heuristic: treat as float if scaled or offset
    is_float = (
        (sig.scale is not None and sig.scale != 1.0) or
        (sig.offset is not None and sig.offset != 0.0)
    )

    if is_float:
        # For simplicity, use float32 unless precision requires float64
        return "float32" if sig.length <= 32 else "float64"

    # Otherwise, choose an appropriate int type
    if sig.is_signed:
        if sig.length <= 8:
            return "int8"
        elif sig.length <= 16:
            return "int16"
        elif sig.length <= 32:
            return "int32"
        else:
            return "int64"
    else:
        if sig.length <= 8:
            return "uint8"
        elif sig.length <= 16:
            return "uint16"
        elif sig.length <= 32:
            return "uint32"
        else:
            return "uint64"

# Fills the message file with the signals from the DBC


def fillMessage(structs, msg_path, dbc_path, pkgName):
    db = cantools.database.load_file(dbc_path)

    for msg in db.messages:
        name_snake = to_snake_case(msg.name)
        struct_name = f"{name_snake}_t"
        if not any(s.name == struct_name for s in structs):
            continue

        file_path = os.path.join(msg_path, name_snake + '_msg.msg')
        with open(file_path, 'w') as mf:
            mf.write("std_msgs/Header header\n\n")
            mf.write(
                f"# {pkgName.upper()}_{struct_name.upper()} {hex(msg.frame_id)}u\n")

            for sig in msg.signals:
                ros_type = get_ros_type(sig)
                sig_name = to_snake_case(sig.name)
                mf.write(f"{ros_type} {sig_name}\n")


def get_msg_name(struct_name):
    """Extract the ROS msg name from a full struct name."""
    parts = struct_name.split('_')
    return '_'.join(parts[:-1])  # remove package and '_t'


def is_in_filter(struct, filter_list):
    return not filter_list or struct.name in filter_list


def writeCpp(structs, src_path, pkg_name, sub_topic, pub_topic, filter_file_path):
    import json

    # Load filters
    with open(filter_file_path) as file:
        filters = json.load(file)
        inbound_filter = [f"{to_snake_case(x)}_t" for x in filters.get("inbound", [])]
        outbound_filter = [f"{to_snake_case(x)}_t" for x in filters.get("outbound", [])]

    # Prepare includes
    headers = (
        '#include <ros/ros.h>\n'
        '#include "can_msgs/Frame.h"\n'
        f'#include "{pkg_name}.h"\n'
    )
    headers_msg = ''.join([
        f'#include "{pkg_name}/{get_msg_name(struct.name)}_msg.h"\n'
        for struct in structs
    ])

    # Namespace
    namespace = 'using namespace std;'

    # Start class
    class_name = f'{pkg_name.upper()}Feedback'
    class_def = f'class {class_name} {{\npublic:\n\t{class_name}() {{\n'
    class_def += '\t\tros::NodeHandle private_nh;\n'
    class_def += f'\t\tcan_sub = private_nh.subscribe("{sub_topic}", 1, &{class_name}::canCallback, this);\n'

    # Publishers
    for struct in structs:
        if is_in_filter(struct, inbound_filter):
            msg_name = get_msg_name(struct.name)
            class_def += f'\t\t{msg_name}_pub = private_nh.advertise<{pkg_name}::{msg_name}_msg>("{pkg_name}/{struct.name[:-2]}", 1);\n'

    class_def += f'\t\tcan_pub = private_nh.advertise<can_msgs::Frame>("{pub_topic}", 1);\n'

    # Subscribers
    for struct in structs:
        if is_in_filter(struct, outbound_filter):
            msg_name = get_msg_name(struct.name)
            class_def += f'\t\t{msg_name}_sub = private_nh.subscribe("{pkg_name}/{struct.name[:-2]}", 1, &{class_name}::{msg_name}Callback, this);\n'

    class_def += f'\t}}\n\t~{class_name}(){{}}\n'

    # Private section
    private_section = '\nprivate:\n\tros::Subscriber can_sub;\n'
    for struct in structs:
        if is_in_filter(struct, inbound_filter):
            private_section += f'\tros::Publisher {get_msg_name(struct.name)}_pub;\n'
    private_section += '\tros::Publisher can_pub;\n'
    for struct in structs:
        if is_in_filter(struct, outbound_filter):
            private_section += f'\tros::Subscriber {get_msg_name(struct.name)}_sub;\n'

    for struct in structs:
        private_section += f'\t{pkg_name}_{struct.name}* {get_msg_name(struct.name)} = new {pkg_name}_{struct.name};\n'

    # Outbound callbacks
    outbound_callbacks = ''
    for struct in structs:
        if is_in_filter(struct, outbound_filter):
            msg_name = get_msg_name(struct.name)
            outbound_callbacks += f'\n\tvoid {msg_name}Callback(const {pkg_name}::{msg_name}_msg msg) {{\n'
            outbound_callbacks += '\t\tcan_msgs::Frame can_frame;\n'
            for sig in struct.signals:
                outbound_callbacks += f'\t\t{msg_name}->{sig} = {pkg_name}_{struct.name[:-2]}_{sig}_encode(msg.{sig});\n'
            outbound_callbacks += f'\t\t{pkg_name}_{struct.name[:-2]}_pack(can_frame.data.data(), {msg_name}, 8);\n'
            outbound_callbacks += f'\t\tcan_frame.id = {pkg_name.upper()}_{struct.header};\n'
            outbound_callbacks += f'\t\tcan_frame.dlc = {pkg_name.upper()}_{struct.header[:-9]}_LENGTH;\n'
            outbound_callbacks += '\t\tcan_frame.header.stamp = ros::Time::now();\n'
            outbound_callbacks += '\t\tcan_frame.header.seq++;\n'
            outbound_callbacks += '\t\tcan_pub.publish(can_frame);\n'
            outbound_callbacks += '\t}\n'

    # Inbound callback
    inbound_callback = '\n\tvoid canCallback(const can_msgs::Frame msg) {\n'
    inbound_callback += '\t\tuint id = (msg.id > 2147483647) ? msg.id ^ 0x80000000 : msg.id;\n'
    for struct in structs:
        if is_in_filter(struct, inbound_filter):
            msg_name = get_msg_name(struct.name)
            inbound_callback += f'\t\t{pkg_name}::{msg_name}_msg {msg_name}_msg;\n'

    inbound_callback += '\n\t\tswitch(id) {\n'
    for struct in structs:
        if is_in_filter(struct, inbound_filter):
            msg_name = get_msg_name(struct.name)
            unpack_func = f"{pkg_name}_" + \
                struct.header.lower().replace('_frame_id', '_unpack')
            inbound_callback += f'\t\tcase {pkg_name.upper()}_{struct.header}:\n'
            inbound_callback += f'\t\t\t{unpack_func}({msg_name}, msg.data.data(), 8);\n'
            for sig in struct.signals:
                decoder_prefix = '_'.join(struct.name.split('_')[:-1])
                inbound_callback += f'\t\t\t{msg_name}_msg.{sig} = {pkg_name}_{decoder_prefix}_{sig}_decode({msg_name}->{sig});\n'
            inbound_callback += f'\t\t\t{msg_name}_msg.header.stamp = ros::Time::now();\n'
            inbound_callback += f'\t\t\t{msg_name}_msg.header.seq++;\n'
            inbound_callback += f'\t\t\t{msg_name}_pub.publish({msg_name}_msg);\n'
            inbound_callback += '\t\t\tbreak;\n\n'
    inbound_callback += '\t\tdefault:\n\t\t\tROS_DEBUG("Unknown CAN_FRAME_ID");\n\t\t\tbreak;\n\t\t}\n\t}\n'

    # Class end + main
    class_end = '};\n'
    main_func = (
        f'\nint main(int argc, char *argv[]) {{\n'
        f'\tros::init(argc, argv, "{pkg_name}_feedback");\n'
        f'\t{class_name} {pkg_name};\n'
        f'\tros::spin();\n'
        f'\treturn 0;\n}}\n'
    )

    # Write to file
    with open(src_path, 'w') as cpp:
        cpp.write(headers)
        cpp.write(headers_msg)
        cpp.write(namespace + '\n\n')
        cpp.write(class_def)
        cpp.write(private_section)
        cpp.write(outbound_callbacks)
        cpp.write(inbound_callback)
        cpp.write(class_end)
        cpp.write(main_func)


def update_CMakeLists(structs, srcPath, pkgName):
    lines = []
    with open(srcPath+'/CMakeLists.txt', 'r') as file:
        lines = file.readlines()

        for struct in structs:
            pName = '_'.join(struct.name.split('_')[:-1])
            lines.insert(18, '   '+pName+'_msg.msg\n')

    with open(srcPath+'/CMakeLists.txt', 'w') as file:
        file.writelines(lines)


def main():
    if len(sys.argv) == 7:
        pkg_name = sys.argv[1]
        dbc_path = sys.argv[2]
        pckg_path = sys.argv[3]
        filter_file_path = sys.argv[4]
        sbs_topic = sys.argv[5]
        pub_topic = sys.argv[6]

    else:
        print("Invalid Arguments!")
        print("Usage: python3 canparsercreator.py <package name> <dbc path> <package path> <filter file path> <subscribing topic name for can messages> <publishing topic name for can messages>")
        return -1

    files = os.listdir('.')
    file_directory = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(file_directory, 'scripts')
    if not 'env' in files:
        os.system(f"chmod +x {script_path}/install.sh")
        os.system(f"{script_path}/install.sh")

    os.system(f"chmod +x {script_path}/generateParser.sh")
    os.system(f"{script_path}/generateParser.sh " + pkg_name +
              " " + dbc_path + " " + pckg_path + " " + sbs_topic)
    structs = load_messages_from_dbc(dbc_path, filter_file_path)
    fillMessage(structs, pckg_path+'/'+pkg_name+'/msg/', dbc_path, pkg_name)
    update_CMakeLists(structs, pckg_path+'/'+pkg_name, pkg_name)
    writeCpp(structs, pckg_path+'/'+pkg_name+'/src/'+pkg_name +
             '_parser.cpp', pkg_name, sbs_topic, pub_topic, filter_file_path)
    print(f'Can parser [{pkg_name}] was created SUCCESSFULLY!\n')


if __name__ == '__main__':
    exit(main())
