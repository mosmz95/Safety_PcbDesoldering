#!/usr/bin/env python3
import os
import rospy
from datetime import datetime
import tkinter as tk
from tkinter import simpledialog, messagebox
import json
import geometry_msgs.msg
from desoldering_pcb.moveit_interface import robotplanninginterface

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
folder_path = os.path.join(parent_dir, "data_base")

def on_enter_pressed(event):
    if event.keysym == "Return":  # Check if Enter key is pressed
        root.destroy()  # Close the window after Enter is pressed

def create_input_window(prompt):
    global root

    # Create the main window
    root = tk.Tk()
    root.title("Press Enter")

    # Calculate the screen width and height
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()

    # Calculate the position to center the window
    x = (screen_width - root.winfo_reqwidth()) / 2
    y = (screen_height - root.winfo_reqheight()) / 2

    # Set the window position and size
    root.geometry(f"400x100+{int(x)}+{int(y)}")

    # Create and place the label with the prompt
    label = tk.Label(root, text=prompt)
    label.pack(pady=10)

    # Bind the Return event to the on_enter_pressed function
    root.bind('<Return>', on_enter_pressed)

    # Start the main event loop
    root.mainloop()

def get_user_input(prompt):
    # Create the main window
    root = tk.Tk()
    root.withdraw()  # Hide the root window

    # Show a dialog box to get user input
    user_input = simpledialog.askstring("Input", prompt)

    # Destroy the root window after getting input
    root.destroy()

    return user_input
def create_filename():
    
    current_time = datetime.now()
    timestamp = current_time.strftime("%Y%m%d_%H%M%S")
    
    filename = f"data_{timestamp}.json"
    return filename

def save_data_to_json(data, folder_path, filename):
    # Ensure the folder exists
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    # Create the full file path
    file_path = os.path.join(folder_path, filename)
    
    # Write the data to a JSON file
    with open(file_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)
    
    print(f"Data saved to {file_path}")





def save_data(data, filename):
    # Write the data to a JSON file
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)


def main():
    ur5e=robotplanninginterface("manipulator")
    # ur5e.display_basic_info()
    # ur5e.move_group.get_current_joint_values()
    data={}
    data["pcb_components"]=[]
    
    component_number = int(get_user_input("Enter the number of components you want to get their positions:"))

    for counter in range(component_number):
        cmp_data = {}
        cmp_data["name"] = f"component_{counter +1}"
        cmp_data["cobot_JointSpace"]=[]
        cmp_data["tool_position_xyz"]=[]
        cmp_data["tool_orientation_xyzw"]=[]
        cmp_data["Heat_duration"] = 3
        create_input_window(f"Press Enter to save the {counter +1}_th current position of Cobot...")
        cobot_pose = ur5e.move_group.get_current_pose().pose
        cmp_data["tool_position_xyz"] = [ cobot_pose.position.x, cobot_pose.position.y, cobot_pose.position.z ]
        cmp_data["tool_orientation_xyzw"] = [cobot_pose.orientation.x, cobot_pose.orientation.y, cobot_pose.orientation.z,cobot_pose.orientation.w]
        cmp_data["cobot_JointSpace"] = ur5e.move_group.get_current_joint_values()

        data["pcb_components"].append(cmp_data)

        counter +=1


    safety_record = get_user_input("Do you want to save a new safety configuration?(type Yes/No)")
 
    if safety_record.upper()=="YES":
        data["Safety_configs"] = []
        
        cb_safety_config_number = int(get_user_input("Enter the number of safety config you want to record:"))
        for sf_counter in range(cb_safety_config_number):
            create_input_window(f"Press Enter to save the {sf_counter}_th current position of Cobot...")
            cb_safety_config = {}
            cb_safety_config["name"] = f"safety_{sf_counter + 1}"
            cobot_pose = ur5e.move_group.get_current_pose().pose
            cb_safety_config["safety_tool_position_xyz"] = [ cobot_pose.position.x, cobot_pose.position.y, cobot_pose.position.z ]
            cb_safety_config["safety_tool_orientation_xyzw"] = [cobot_pose.orientation.x, cobot_pose.orientation.y, cobot_pose.orientation.z,cobot_pose.orientation.w]
            cb_safety_config["safety_cobot_JointSpace"] = ur5e.move_group.get_current_joint_values()
            
            data["Safety_configs"].append(cb_safety_config)
            sf_counter +=1

        filename =  create_filename()
        save_data_to_json(data,folder_path,filename)

    else:
        print("Saving ....")
        filename =  create_filename()
        save_data_to_json(data,folder_path,filename)

if __name__=="__main__":
    rospy.init_node("saveing_data_node", anonymous=True) 
    main()