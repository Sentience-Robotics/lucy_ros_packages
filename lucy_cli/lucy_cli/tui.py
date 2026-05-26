"""
This module contains all the logic for rendering the Text-based User Interface (TUI).

It is designed to be completely decoupled from ROS or any other backend logic.
It receives data structures to display and uses callbacks to notify the main
application of user input.

Responsibilities:
- Clearing the screen.
- Drawing menus and lists.
- Handling raw keyboard input with timeouts.
- Maintaining a persistent input buffer across screen refreshes.
"""
import os
import sys
import select
import time

# Global buffer to hold unsubmitted user input, allowing it to persist
# across screen refreshes in auto-refresh mode.
INPUT_BUFFER = ""

def clear_screen():
    """Clears the terminal screen using ANSI escape codes for a flicker-free update."""
    if os.name == 'posix':
        sys.stdout.write("\033[H\033[J")
        sys.stdout.flush()
    else:
        os.system('cls')

def get_user_input(prompt: str, timeout: float = 1.0) -> str | None:
    """
    Waits for user input with a timeout, preserving typed characters.

    This function uses non-blocking, character-by-character input on POSIX
    systems to provide a smooth TUI experience, even when the screen is
    auto-refreshing.

    Args:
        prompt: The input prompt to display to the user.
        timeout: The time in seconds to wait for input before returning.

    Returns:
        - The user's full line of input if they press Enter.
        - An empty string ("") if the user typed but did not press Enter.
        - None if the timeout was reached with no user input.
    """
    global INPUT_BUFFER
    sys.stdout.write(prompt + INPUT_BUFFER)
    sys.stdout.flush()

    if os.name != 'posix':
        # Fallback for non-POSIX systems (like Windows) which lacks tty/termios.
        # This will not have the non-blocking, char-by-char benefits.
        res = input()
        return res

    import termios, tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], timeout)[0]:
            while True:
                char = sys.stdin.read(1)
                if char == '\n' or char == '\r': # Enter pressed
                    result = INPUT_BUFFER
                    INPUT_BUFFER = ""
                    sys.stdout.write('\n')
                    return result
                elif char == '\x7f' or char == '\b': # Backspace
                    if INPUT_BUFFER:
                        INPUT_BUFFER = INPUT_BUFFER[:-1]
                        # Move cursor back, write space, move back again
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif char == '\x03': # Ctrl+C
                    raise KeyboardInterrupt
                elif char.isprintable():
                    INPUT_BUFFER += char
                    sys.stdout.write(char)
                    sys.stdout.flush()
                
                # Check if more characters are available to be read immediately
                if not select.select([sys.stdin], [], [], 0)[0]:
                    break
            # User typed, but didn't press Enter. Redraw will handle the buffer.
            return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    # Timeout occurred
    return None

def display_help_screen():
    """Displays a static help message and waits for user confirmation."""
    clear_screen()
    print("--- TUI Help ---")
    print("\n[General Commands]")
    print("  'a'         - Toggle auto-refresh mode on/off.")
    print("  'h'         - Display this help screen.")
    print("  'q'         - Quit the application.")
    print("\n[Main Menu]")
    print("  <number>  - Select an actuator group to view and edit its joints.")
    print("  't'         - (When available) Forcibly take control from another client.")
    print("\n[Joint Menu]")
    print("  <number>  - Select a joint to modify its angle.")
    print("  'b'         - Go back to the main menu.")
    print("\nPress Enter to return...")
    input()

def display_main_menu(state: dict):
    """
    Renders the main menu screen.

    Args:
        state: A dictionary containing the current UI state, including:
               'client_count', 'autorefresh', 'has_control', 'active_controller',
               and 'actuator_groups'.
    """
    print("--- Robot Actuator TUI ---")
    print(f"Control Panel Connection Count: {state.get('client_count', 'N/A')}")
    if state.get('autorefresh'):
        print("[Auto-Refresh: ON]")
    
    if not state.get('has_control') and state.get('active_controller'):
        print(f"!! CONTROLLED BY: {state['active_controller']} !!\n")
        print("Press 't' to take control.")
    
    print("\nSelect an actuator group:")
    for i, name in enumerate(state.get('actuator_groups', [])):
        print(f"{i+1}. {name}")
    print("\nEnter 'q' to quit or 'h' for help.")

def display_joint_menu(state: dict, group_name: str):
    """
    Renders the menu for editing joints within a specific group.

    Args:
        state: A dictionary containing the current UI state.
        group_name: The name of the actuator group being displayed.
    """
    print(f"--- {group_name} ---")
    if not state.get('has_control'):
        print(f"!! READ-ONLY (Controlled by {state.get('active_controller')}) !!\n")

    joints = state.get('actuators', {}).get(group_name, {}).get('joints', [])
    for i, joint in enumerate(joints):
        print(f"{i+1}. {joint['name']}: {joint['value']:.1f}° (Min: {joint['min']:.0f}, Max: {joint['max']:.0f})")
    
    if state.get('has_control'):
        print("\nEnter joint number to edit, 'b' to go back, 'h' for help, or 'q' to quit.")
    else:
        print("\nEnter 'b' to go back, 'h' for help, or 'q' to quit.")

def get_new_joint_value(joint_name: str) -> float | None:
    """
    Prompts the user for a new joint value.

    This uses standard blocking input as it's a specific data entry task.

    Args:
        joint_name: The name of the joint being edited.

    Returns:
        The new value as a float, or None if input is invalid.
    """
    global INPUT_BUFFER
    INPUT_BUFFER = "" # Clear buffer before this prompt
    try:
        value_str = input(f"Enter new value for {joint_name} (in degrees): ")
        return float(value_str)
    except (ValueError, TypeError):
        print("Invalid input. Please enter a number.")
        time.sleep(1)
        return None