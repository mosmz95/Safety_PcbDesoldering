#!usr/bin/env python3
import os
import json

class Pcb_component:
    '''
        this class stores the position of pcb component
    '''
    def __init__(self,name,position,orientatin,cobot_config,heattime) -> None:
        
        self.name_ = name
        self.tool_position_ = position
        self.tool_orientation_ = orientatin
        self.cobot_config_ = cobot_config
        self.heatduration = heattime
        self.is_heated_ = False

    def component_name(self)-> str:
        return self.name_
    
    def got_heated(self):
        self.is_heated_= True

    def removal_failure(self):
        self.is_heated_= False

    def heat_status(self):
        return self.is_heated_
    
    def toolposition(self):
        return self.tool_position_
    
    def toolorientation(self):
        return self.tool_orientation_
    
    def cobotconfig(self):
        return self.cobot_config_
    
    def toolpose(self):
        return self.tool_position_.append(self.tool_orientation_)
    

class SafeRobotConfig:
    '''
        This class creates an object representing the robot safe configuration and its corresponding tool position in workspace
    '''
    def __init__(self,position,orientatin,cobot_config) -> None:
        self.tool_position_ = position
        self.tool_orientation_ = orientatin
        self.cobot_config_ = cobot_config
        
    
    def safecobot_config(self):
        return self.cobot_config_
    
    def toolpose(self):
        return self.tool_position_.append(self.tool_orientation_)
    


def home():
    return [1.5271029472351074,
                -1.5869223080077113,
                2.188160244618551,
                -2.152525564233297,
                -1.6478660742389124,
                3.8190863132476807]
    
def main():

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    target_dir = os.path.join(parent_dir, "data_base")
    json_path = os.path.join(target_dir, "data_20240528_105239.json")
    with open(json_path, "r") as json_file:
        raw_data = json.load(json_file)

    print(len(raw_data["pcb_components"]))
    list_of_components =[]
    for i in range(len(raw_data["pcb_components"])):
        raw_component = raw_data["pcb_components"][i]
        temporary_component = Pcb_component(position=raw_component["tool_position_xyz"],
                                            orientatin=raw_component["tool_orientation_xyzw"],
                                            cobot_config=raw_component["cobot_JointSpace"],
                                            heattime=raw_component["Heat_duration"])
        list_of_components.append(temporary_component)

   

    if "Safety_configs" in raw_data.keys():
        list_of_safetyconfigs =[]
        for j in range(len(raw_data["Safety_configs"])):
            raw_safety = raw_data["Safety_configs"][j]
            temporary_safety = SafeRobotConfig(position=raw_safety["safety_tool_position_xyz"],
                                            orientatin=raw_safety["safety_tool_orientation_xyzw"],
                                            cobot_config=raw_safety["safety_cobot_JointSpace"])
            list_of_safetyconfigs.append(temporary_safety)


    print(raw_data.keys())

if __name__=="__main__":
    main()
 