
# TODO 
# create layer panels for each child
#  - pull mass, type, axis
# export as dae/stl
#  - recenter and export
# figure out a way to get inertia moments here ..

bl_info = {
    "name" : "urdf_creator",
    "author" : "Phil Ring",
    "description" : "URDF Creator",
    "blender" : (2, 82, 0),
    "version" : (0, 0, 1),
    "location" : "View3D",
    "warning" : "",
    "category" : "Generic"
}

import os
import bpy
from bpy.types import Operator
from bpy.props import FloatVectorProperty
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from mathutils import Vector

#################

# from parent to child
def getTransform(parent, child):
    return (child.location - parent.location), \
            Vector( [(child.rotation_euler[0] - parent.rotation_euler[0]), 
                     (child.rotation_euler[1] - parent.rotation_euler[1]), 
                     (child.rotation_euler[2] - parent.rotation_euler[2])])


def treeTraversal(parent):
    for child in parent.children:
        # link
        mass = 10
        inertia = [0,0,0,0,0,0] # no blender built-in for this .. will need meshlab :/
        mesh_filename = child.name+'.dae'
        generate_link(child.name, mass, inertia, mesh_filename)
        
        # joint
        pos, euler = getTransform(parent,child)
        type = "continuous"
        axis = "1 0 0"
        generate_joint(parent.name, child.name, pos, euler, type, axis)
        
        isLeaf = (child.children == ())
        if not isLeaf:
            treeTraversal(child)


urdf_joints = []         
urdf_links = []
urdf_str = ""
   
def generate_joint(parent, child, oxyz, orpy, type="continuous", axis="1 0 0"):
    global urdf_joints
    joint_name = "\""+parent+"_"+child+"_joint\""
    type   = "\""+type+"\""
    oxyz   = "\""+str(oxyz[0])+" "+str(oxyz[1])+" "+str(oxyz[2])+"\""
    orpy   = "\""+str(orpy[0])+" "+str(orpy[1])+" "+str(orpy[2])+"\""
    parent = "\""+parent+"\""
    child  = "\""+child+"\""
    axis   = "\""+axis+"\""
    joint_str =   """  <joint name={} type={}>  
                        <origin xyz={} 
                                rpy={} />  
                        <parent link={} />  
                        <child link={} />  
                        <axis xyz={} />   
                    </joint>""".format( joint_name, type, oxyz, orpy, parent, child, axis )
        
    urdf_joints.append(joint_str)          
    
def generate_link(link_name, mass, inertia, mesh_filename):
    global urdf_links
    link_name = "\""+link_name+"\""
    mass      = "\""+str(mass)+"\""
    inertia   = " ixx=\""+str(inertia[0])+"\"" + \
                " ixy=\""+str(inertia[1])+"\"" + \
                " ixz=\""+str(inertia[2])+"\"" + \
                " iyy=\""+str(inertia[3])+"\"" + \
                " iyz=\""+str(inertia[4])+"\"" + \
                " izz=\""+str(inertia[5])+"\""
    mesh_filename = "\""+mesh_filename+"\""
    link_str = """  <link name={}>
                        <inertial>
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <mass value={} />
                          <inertia {} />
                        </inertial>
                        <visual>
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <geometry>
                            <mesh filename={} />
                          </geometry>
                        </visual>
                        <collision>
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <geometry>
                            <mesh filename={} />
                          </geometry>
                        </collision>
                      </link>""".format(link_name, mass, inertia, mesh_filename, mesh_filename)
                      
    urdf_links.append(link_str)
                      
def generate_urdf_str():
    global urdf_str
    
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name 
    if robot_name == '':
        print('Name doesn\'t exist. Please insert name')
        return
    robot_name = "\""+robot_name+"\""
    
    urdf_str = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
    urdf_str += "<robot name={}>".format(robot_name)
    urdf_str += "\n\n <!-- Links --> \n\n"
    for link in urdf_links:
        urdf_str += link+"\n"
    urdf_str += "\n\n <!-- Joints --> \n\n"
    for joint in urdf_joints:
        urdf_str += joint+"\n"
    urdf_str += "</robot>"
       
                      
def save_urdf_to_file():    
    basedir = os.path.dirname(bpy.data.filepath)
    file = open(basedir+'/test.urdf', 'w')
    file.write(urdf_str)
    file.close()
    print('saved file')


def print(data): # for printing in blender console
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type == 'CONSOLE':
                override = {'window': window, 'screen': screen, 'area': area}
                bpy.ops.console.scrollback_append(override, text=str(data), type="OUTPUT")      
                
##############
          
          
class URDF_properties(bpy.types.PropertyGroup):
    file_path: bpy.props.StringProperty(name="File path",
                                        description="Some elaborate description",
                                        default="",
                                        maxlen=1024,
                                        subtype="FILE_PATH")    
    robot_name: bpy.props.StringProperty(name="Name",
                                        description="Some elaborate description",
                                        default="",
                                        maxlen=1024)
                                                                               
                              
# Panel display
class URDF_PT_PANEL(bpy.types.Panel):
    bl_idname = "URDF_PT_PANEL"
    bl_label = "URDF Creator"
    bl_category = "URDF Creator"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout

        layout.label(text="1. Select Root Object")
        
        layout.label(text="2. Name the Robot")   
        row = layout.row()
        URDF_properties_tools = context.scene.URDF_properties_tools
        row.prop(URDF_properties_tools, "robot_name")
        
        layout.label(text="3. Generate URDF")
        row = layout.row()
        row.scale_y = 2.0
        row.operator(".generate_urdf", text="Generate URDF")
        

class GenerateURDF_Operator(bpy.types.Operator):
    bl_idname = ".generate_urdf"
    bl_label = "Simple operator"
    bl_description = "Generate .urdf links and joints through tree traversal, "

    def execute(self, context):
        print('Executing test')
        
        root = bpy.context.selected_objects[0]
        treeTraversal(root)
        
        generate_link("base_link", 10, [0,1,2,3,4,5], 'base.dae')
        #generate_joint("base_link", "chile", [0,1,2], [3,4,5])
        generate_urdf_str()
        save_urdf_to_file()
        
        return {'FINISHED'}
   

def register():
    bpy.utils.register_class(URDF_PT_PANEL)
    bpy.utils.register_class(GenerateURDF_Operator)
    bpy.utils.register_class(URDF_properties)
    bpy.types.Scene.URDF_properties_tools = bpy.props.PointerProperty(type=URDF_properties)
     
def unregister():
    bpy.utils.unregister_class(URDF_PT_PANEL)
    bpy.utils.unregister_class(GenerateURDF_Operator)
    bpy.utils.unregister_class(URDF_properties)
    del bpy.types.Scene.URDF_properties_tools
    
if __name__=='__main__':
    register()



