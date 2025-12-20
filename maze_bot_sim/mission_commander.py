#! /usr/bin/env python3

# Import the ROS 2 Python client library. This library allows us to create nodes, 
# publishers, subscribers, and other ROS entities in Python.
import rclpy

# Import the sys module to access command-line arguments passed to the script.
import sys

# Import the argparse module to make it easier to handle command-line arguments.
import argparse

# Import the custom debugger classes for Stage 1 and Stage 2.
# These classes contain the specific logic for each part of the maze challenge.
# We assume these files are located in the 'maze_bot_sim' package.
from maze_bot_sim.debug_stage1 import Stage1Debugger
from maze_bot_sim.debug_stage2 import Stage2Debugger

def main():
    """
    The main entry point of the script.
    This function initializes ROS 2, parses command-line arguments to decide 
    which stage to run, and then executes the corresponding logic.
    """

    # Initialize the ROS 2 communication system.
    # This must be called before creating any ROS nodes.
    rclpy.init()

    # Create an argument parser to handle the user's input from the command line.
    # 'description' provides a help message when the user runs the script with --help.
    parser = argparse.ArgumentParser(description='Mission Commander for Maze Bot')

    # Add a required argument named 'stage'.
    # The 'choices' parameter restricts the input to either 'stage1' or 'stage2'.
    # This ensures the user cannot run the script with an invalid stage name.
    parser.add_argument('stage', choices=['stage1', 'stage2'], help='The stage to run (stage1 or stage2)')
    
    # ROS 2 passes some internal arguments to nodes (like --ros-args).
    # The argparse library might get confused by these if we don't filter them out.
    # Here, we create a list 'clean_args' that contains only the arguments meant for our script,
    # ignoring any that start with '--ros'.
    clean_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros')]
    
    # Parse the filtered arguments.
    # If the arguments are invalid (e.g., missing stage), the script will exit here with an error message.
    args = parser.parse_args(clean_args)
    
    # Initialize a variable to hold our ROS node.
    node = None
    
    try:
        # Check which stage the user requested.
        if args.stage == 'stage1':
            # If the user requested 'stage1', we instantiate the Stage1Debugger class.
            print("Launching Stage 1 Debugger...")
            node = Stage1Debugger()
            
        elif args.stage == 'stage2':
            # If the user requested 'stage2', we instantiate the Stage2Debugger class.
            print("Launching Stage 2 Debugger...")
            node = Stage2Debugger()
            
        # If a node was successfully created (which should always happen if we get here),
        # we call its 'run' method to start the mission logic.
        if node:
            node.run()
            
    except KeyboardInterrupt:
        # This block catches the 'KeyboardInterrupt' exception, which is raised
        # when the user presses Ctrl+C in the terminal.
        # We print a friendly message instead of a scary error trace.
        print("Mission Commander interrupted by user.")
        
    finally:
        # The 'finally' block always runs, whether the code succeeded or failed.
        # It is the perfect place to clean up resources.
        
        if node:
            # Destroy the node to free up resources.
            node.destroy_node()
            
        # Shutdown the ROS 2 client library.
        # This ensures all ROS communications are properly closed.
        rclpy.shutdown()

# This standard Python check ensures that the main() function is only called
# if this file is executed directly (not imported as a module).
if __name__ == '__main__':
    main()
