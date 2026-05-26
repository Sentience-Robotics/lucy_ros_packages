"""
The main entry point for the Lucy CLI TUI Actuator Node.
...
"""
import sys
import argparse
import time
import rclpy
import math
import yaml
from .ros_interface import LucyROSInterface
from .tui import (
    clear_screen, get_user_input, display_help_screen,
    display_main_menu, display_joint_menu, get_new_joint_value
)

def run_tui(ros: LucyROSInterface, actuators: dict, autorefresh: bool):
    """
    The main application loop.

    Args:
        ros: The initialized LucyROSInterface instance.
        actuators: The dynamic actuator configuration dictionary.
        autorefresh: Boolean indicating if auto-refresh is enabled by default.
    """
    current_autorefresh = autorefresh

    while rclpy.ok():
        clear_screen()
        
        state = {
            'autorefresh': current_autorefresh,
            'client_count': ros._client_count,
            'has_control': ros.has_control(),
            'active_controller': ros._active_controller_id,
            'actuator_groups': list(actuators.keys()),
            'actuators': actuators
        }
        
        display_main_menu(state)
        
        timeout = 1.0 if state['autorefresh'] else 3600.0
        choice = get_user_input("> ", timeout=timeout)
        
        if choice is None or choice == '':
            continue
            
        choice = choice.lower()
        if choice == 'q':
            break
        elif choice == 'h':
            display_help_screen()
        elif choice == 'a':
            current_autorefresh = not current_autorefresh
            print(f"Auto-refresh {'enabled' if current_autorefresh else 'disabled'}.")
            time.sleep(0.7)
        elif choice == 't' and not state['has_control']:
            ros.take_control()
            print("Requesting control...")
            time.sleep(0.5)
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(state['actuator_groups']):
                    group_name = state['actuator_groups'][idx]
                    current_autorefresh = handle_group_menu(ros, actuators, group_name, current_autorefresh)
                else:
                    raise ValueError
            except ValueError:
                print("Invalid input. Refreshing...")
                time.sleep(1)

def handle_group_menu(ros: LucyROSInterface, actuators: dict, group_name: str, autorefresh: bool) -> bool:
    """Handles the sub-menu for a specific actuator group."""
    current_autorefresh = autorefresh
    
    while rclpy.ok():
        clear_screen()
        
        state = {
            'autorefresh': current_autorefresh,
            'client_count': ros._client_count,
            'has_control': ros.has_control(),
            'active_controller': ros._active_controller_id,
            'actuator_groups': list(actuators.keys()),
            'actuators': actuators
        }
        
        display_joint_menu(state, group_name)
        
        timeout = 1.0 if state['autorefresh'] else 3600.0
        choice = get_user_input("> ", timeout=timeout)
        
        if choice is None or choice == '':
            continue
            
        choice = choice.lower()
        if choice == 'b':
            break
        elif choice == 'q':
            ros.shutdown()
            rclpy.try_shutdown()
            sys.exit(0)
        elif choice == 'h':
            display_help_screen()
        elif choice == 'a':
            current_autorefresh = not current_autorefresh
            print(f"Auto-refresh {'enabled' if current_autorefresh else 'disabled'}.")
            time.sleep(0.7)
        elif state['has_control']:
            try:
                joints = state['actuators'][group_name]['joints']
                joint_idx = int(choice) - 1
                if 0 <= joint_idx < len(joints):
                    selected_joint = joints[joint_idx]
                    new_val = get_new_joint_value(selected_joint['name'])
                    if new_val is not None:
                        if selected_joint['min'] <= new_val <= selected_joint['max']:
                            selected_joint['value'] = new_val
                            
                            names = [j['name'] for j in joints]
                            positions = [math.radians(j['value']) for j in joints]
                            topic = state['actuators'][group_name]['topic']
                            
                            ros.publish_joint_trajectory(topic, names, positions)
                            print("Value updated successfully. Refreshing...")
                        else:
                            print(f"Value out of range ({selected_joint['min']} - {selected_joint['max']}).")
                        time.sleep(1)
                else:
                    raise ValueError
            except ValueError:
                print("Invalid input.")
                time.sleep(1)
                
    return current_autorefresh

def parse_config_yaml(yaml_string: str) -> dict:
    """Parses the hardware config YAML and builds the ACTUATORS dictionary."""
    if not yaml_string: return {}
    try:
        doc = yaml.safe_load(yaml_string)
        if not isinstance(doc, dict): return {}
        boards, actuators_list = doc.get('boards', {}), doc.get('actuators', [])
        parsed_actuators = {}
        for board_id, bdef in boards.items():
            if not isinstance(bdef, dict): continue
            ctrl_name = bdef.get('controller', {}).get('name')
            if not ctrl_name: continue
            board_actuators = sorted([a for a in actuators_list if isinstance(a, dict) and a.get('board') == board_id], key=lambda a: a.get('virtual_pin', 0))
            joints = []
            for actuator in board_actuators:
                urdf_joint = actuator.get('urdf_joint')
                if not urdf_joint: continue
                joints.append({
                    "name": urdf_joint,
                    "min": float(actuator.get('servo_min_deg', 0.0)),
                    "max": float(actuator.get('servo_max_deg', 180.0)),
                    "value": float(actuator.get('servo_default_deg', 90.0)),
                })
            if joints:
                group_name = board_id.replace("rp2040_", "").replace("_", " ").title()
                parsed_actuators[group_name] = {"topic": f"/{ctrl_name}/joint_trajectory", "joints": joints}
        return parsed_actuators
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}")
        return {}

def main(args=None):
    parser = argparse.ArgumentParser(description='Lucy TUI Actuator Node')
    parser.add_argument('-a', '--autorefresh', action='store_true', help='Enable auto-refresh for the TUI screen')
    
    sys_args = sys.argv[1:] if args is None else args
    parsed_args, ros_args = parser.parse_known_args(sys_args)
    
    rclpy.init(args=ros_args)
    ros_interface = LucyROSInterface()

    print("Fetching active hardware configuration...")
    config_yaml = ros_interface.get_hardware_config_yaml()
    if not config_yaml:
        print("Could not retrieve hardware configuration. Is lucy_config_pipeline running?")
        ros_interface.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    actuators = parse_config_yaml(config_yaml)
    if not actuators:
        print("Failed to parse hardware configuration or no actuators found.")
        ros_interface.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    print("Fetching initial client count...")
    ros_interface.get_initial_client_count()

    try:
        run_tui(ros_interface, actuators, parsed_args.autorefresh)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        ros_interface.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()