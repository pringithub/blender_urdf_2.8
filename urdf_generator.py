
# TODO 
# create layer panels for each child
#  - pull mass, type, axis
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
def getTransform(child, parent, parent_joint_loc):
    child_joint_loc = getJointPos()
    print('child joint loc')
    print(child_joint_loc)
    print('parent joint loc')
    print(parent_joint_loc)
    pos = (child_joint_loc - parent_joint_loc) 
    print(pos)
    euler = Vector( [(child.rotation_euler[0] - parent.rotation_euler[0]), 
                     (child.rotation_euler[1] - parent.rotation_euler[1]), 
                     (child.rotation_euler[2] - parent.rotation_euler[2])])
    return pos, euler
            
def getJointPos():
    bpy.ops.view3d.snap_cursor_to_selected()
    pos = bpy.context.scene.cursor.location
    return pos.copy()

urdf_joints = []
urdf_links  = []
urdf_transmissions = []
joint_controller_gains = []
def generate_root_link(root, export_type='obj'):
    # base_link
    mesh_filename = root.name+'.'+export_type
    link_str = generate_link(root.name, mesh_filename)
    urdf_links.append(link_str)
    center_and_export(root, export_type)
    
def treeTraversal(parent, export_type='obj'):
    global urdf_joints, urdf_links, urdf_transmissions, joint_controller_gains
    
    print('---')
    parent_joint_loc = getJointPos()
    print(parent_joint_loc)
    # child links, joints
    for child in parent.children:
        print(child)
        
        #deselect all but just one object and make it active
        bpy.ops.object.select_all(action='DESELECT')
        child.select_set(state=True)
        
        # link
        mesh_filename = child.name+'.'+export_type
        link_str = generate_link(child.name, mesh_filename)
        urdf_links.append(link_str)
        
        # joint
        pos, euler = getTransform(child, parent, parent_joint_loc)
        type = "continuous"
        axis = "0 0 1"
        joint_str = generate_joint(parent.name, child.name, pos, euler, type, axis)
        urdf_joints.append(joint_str) 
        
        # transmission
        transmission_str = generate_gazebo_transmission(parent.name, child.name)
        urdf_transmissions.append(transmission_str)
        # joint controller gains
        gains_str = generate_joint_controller_gains(parent.name, child.name)
        print(gains_str)
        joint_controller_gains.append(gains_str)
        
        # export
        center_and_export(child, export_type)
        
        isLeaf = (child.children == ())
        print(isLeaf)
        if not isLeaf:
            treeTraversal(child, export_type)

             
    
def generate_link(link_name, mesh_filename):
    
    mass = 10
    inertia = [1,0,0,1,0,1] # no blender built-in for this .. will need meshlab :/
    
    link_name = link_name
    mass      = str(mass)
    inertia   = " ixx=\""+str(inertia[0])+"\"" + \
                " ixy=\""+str(inertia[1])+"\"" + \
                " ixz=\""+str(inertia[2])+"\"" + \
                " iyy=\""+str(inertia[3])+"\"" + \
                " iyz=\""+str(inertia[4])+"\"" + \
                " izz=\""+str(inertia[5])+"\""
    basedir = bpy.context.scene.URDF_properties_tools.file_path
    rospkg = basedir.split('/')[-2:-1][0] # penultimate string is rospkg
    mesh_filename = "package://" + rospkg + "/models/" + mesh_filename
    link_str = """  <link name="{}">
                        <inertial>
                          <mass value="{}" />
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <inertia {} />
                        </inertial>
                        <visual>
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <geometry>
                            <mesh filename="{}" />
                          </geometry>
                        </visual>
                        <collision>
                          <origin xyz="0 0 0" rpy="0 0 0" />
                          <geometry>
                            <mesh filename="{}" />
                          </geometry>
                        </collision>
                      </link>""".format(link_name, mass, inertia, mesh_filename, mesh_filename)
                      
    return link_str

def generate_joint(parent, child, oxyz, orpy, type="continuous", axis="0 0 1"):
    joint_name = parent+"_"+child+"_joint"
    oxyz   = str(oxyz[0])+" "+str(oxyz[1])+" "+str(oxyz[2])
    orpy   = str(orpy[0])+" "+str(orpy[1])+" "+str(orpy[2])
    joint_str =   """  <joint name="{}" type="{}">  
                        <origin xyz="{}" 
                                rpy="{}" />  
                        <parent link="{}" />  
                        <child link="{}" />  
                        <axis xyz="{}" />   
                    </joint>""".format( joint_name, type, oxyz, orpy, parent, child, axis )
        
    return joint_str

def generate_gazebo_transmission(parent, child):
    name_base = parent+"_"+child
    joint_name = name_base+"_joint"
    transmission_name = name_base+"_trans"
    actuator_name = name_base+"_actuator"
    
    transmission_type = "transmission_interface/SimpleTransmission"
    hardware_interface = "hardware_interface/EffortJointInterface"
    mechanical_reduction = 25
    
    transmission_str = """<transmission name="{}">
                  <type>"{}"</type>
                  <joint name="{}">
                          <hardwareInterface>{}</hardwareInterface>
                  </joint>
                  <actuator name="{}">
                          <hardwareInterface>{}</hardwareInterface>
                  </actuator>
                  <mechanicalReduction>{}</mechanicalReduction>
                </transmission>""".format(transmission_name, transmission_type, joint_name,
                        hardware_interface, actuator_name, hardware_interface, str(mechanical_reduction))

    return transmission_str

def generate_joint_controller_gains(parent, child):
    name_base = parent+"_"+child
    joint_name = name_base+"_joint"
    controller_name = name_base+"_controller"
    controller_type = "effort_controllers/JointPositionController"
    kp = 1000.0
    ki = 0.01
    kd = 1.0
    
    gains_str = """
          {}:
            type: {}
            joint: {}
            """.format(controller_name, controller_type, joint_name)
    gains = "p: {}, i: {}, d: {}".format(str(kp), str(ki), str(kd)) # python be trippin
    gains_str += "pid: {"+gains+"}"
            
    print(gains_str)
    return gains_str
            
    


                      
def generate_urdf_str(urdf_links, urdf_joints):
    
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name 
    if robot_name == '':
        print('Name doesn\'t exist. Please insert name')
        return
    robot_name = "\""+robot_name+"\""
    
    urdf_str = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
    urdf_str += """<robot name={}
       xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
       xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://ros.org/wiki/xacro">""".format(robot_name)
    urdf_str
    urdf_str += "\n\n <!-- Links --> \n\n"
    for link in urdf_links:
        urdf_str += link+"\n"
    urdf_str += "\n\n <!-- Joints --> \n\n"
    for joint in urdf_joints:
        urdf_str += joint+"\n"
    urdf_str += "\n\n <!-- Transmissions --> \n\n"
    for transmission in urdf_transmissions:
        urdf_str += "\t"+transmission+"\n\n"
    urdf_str += generate_gazebo_ros_control()+"\n"
    urdf_str += "</robot>"
       
    return urdf_str

                      
def save_urdf_to_file(urdf_str):
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    if robot_name == '':
        print('ERROR: Name the Robot first!')
        return
    
    basedir = bpy.context.scene.URDF_properties_tools.file_path #os.path.dirname(bpy.data.filepath)
    file = open(basedir+'urdf/'+robot_name+'.urdf', 'w')
    file.write(urdf_str)
    file.close()
    print('saved file')




def center_and_export(ob, export_type='obj', decimate_ratio=1.00):
    bpy.context.view_layer.objects.active = ob
     
    #store object location then zero it out
    location = ob.location.copy()
    bpy.ops.view3d.snap_cursor_to_center() ## weird location thing .. offset ?
    bpy.ops.view3d.snap_selected_to_cursor(use_offset=False)

    #bpy.ops.object.location_clear()
    name = ob.name
    #name = re.sub("[.].*?$", '', name)
    #scale = ob.scale.copy()
    #ob.scale = (1,1,1)
    
    rotx = ob.rotation_euler[0]
    roty = ob.rotation_euler[1]
    rotz = ob.rotation_euler[2]
    ob.rotation_euler[0] = 0
    ob.rotation_euler[1] = 0
    ob.rotation_euler[2] = 0
    
    decimate_ratio = bpy.context.scene.URDF_properties_tools.decimate_ratio
    if decimate_ratio is not 1.00:
        print('Decimating')
        decimate_object(ob, decimate_ratio)
    
    path = bpy.context.scene.URDF_properties_tools.file_path
    export(path, ob, export_type)
    
    if decimate_ratio is not 1.00:
        print('Removing decimate')
        undecimate_object(ob)
    
    #restore location
    ob.location = location
    ob.rotation_euler[0] = rotx 
    ob.rotation_euler[1] = roty
    ob.rotation_euler[2] = rotz
    #ob.scale = scale
    
def decimate_object(obj, ratio=1.00):
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.context.object.modifiers["Decimate"].ratio = ratio
def undecimate_object(obj):
    bpy.ops.object.modifier_remove(modifier="Decimate")

    
# exports selected objects
def export(path, obj, type='obj', ):
    path = (path+'/') if path[-1:]!='/' else path # ensure ending '/'
    obj.select_set(True)
    filename = path + "models/" + obj.name + '.' + type
    if type=='stl':
        bpy.ops.export_mesh.stl(filepath=filename, use_selection=True)
    elif type=='obj':
        bpy.ops.export_scene.obj(filepath=filename, use_selection=True, axis_forward='Y', axis_up='Z')
    else:
        print('ERROR: not impl yet')

def print(data): # for printing in blender console
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type == 'CONSOLE':
                override = {'window': window, 'screen': screen, 'area': area}
                bpy.ops.console.scrollback_append(override, text=str(data), type="OUTPUT")      
                
##############                
                
# ROS exports

def generate_rviz_roslaunch(rospkg):
                    
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    rospkg = rospkg
    urdf_file = "$(find {})/urdf/{}.urdf".format(rospkg, robot_name)
    
    str = """<?xml version="1.0" encoding="utf-8"?>
            <launch>
              <arg
                name="{}" />
              <arg
                name="gui"
                default="True" />
              <param
                name="robot_description"
                textfile="{}" />
              <param
                name="use_gui"
                value="$(arg gui)" />
              <node
                name="joint_state_publisher"
                pkg="joint_state_publisher"
                type="joint_state_publisher" />
              <node
                name="robot_state_publisher"
                pkg="robot_state_publisher"
                type="state_publisher" />
              <node
                name="rviz"
                pkg="rviz"
                type="rviz"
                args="-d $(find {})/config/urdf.rviz" />
            </launch>""".format(robot_name, urdf_file, rospkg)      
                
    return str

def save_rviz_roslaunch():
    basedir = bpy.context.scene.URDF_properties_tools.file_path
    rospkg = basedir.split('/')[-2:-1][0] # penultimate string is rospkg
    str = generate_rviz_roslaunch(rospkg)

    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    file = open(basedir+'launch/'+robot_name+'_rviz.launch', 'w') 
    file.write(str)
    file.close()
    print('saved rviz roslaunch file')       
    
#---
    
def generate_gazebo_roslaunch(rospkg, xacro=True):
    
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    rospkg = rospkg
    urdf_file = "$(find {})/urdf/{}.urdf".format(rospkg, robot_name)
    
    str = """<?xml version="1.0" encoding="utf-8"?>
            <launch>
              <include  file="$(find gazebo_ros)/launch/empty_world.launch">
                <arg name="paused" value="true"/>
              </include>
              <node
                name="tf_footprint_base"
                pkg="tf"
                type="static_transform_publisher"
                args="0 0 0 0 0 0 {} base_footprint 40" />""".format(robot_name)
    if not xacro:
        str += """
              <param
                name="robot_description"
                textfile="$(find {})/urdf/{}.urdf" />
              <node
                name="spawn_model"
                pkg="gazebo_ros"
                type="spawn_model"
                args="-param robot_description -urdf -model {}"
                output="screen" />""".format(rospkg, robot_name, robot_name)
    else:
        str += """
              <param
                name="robot_description"
                command="$(find xacro)/xacro $(find {})/urdf/{}.xacro" />
              <node
                name="spawn_model"
                pkg="gazebo_ros"
                type="spawn_model"
                args="-param robot_description -urdf -model {}"
                output="screen" />""".format(rospkg, robot_name, robot_name)

    str += """
              <node
                name="joint_state_publisher"
                pkg="joint_state_publisher"
                type="joint_state_publisher" />
              <node
                name="robot_state_publisher"
                pkg="robot_state_publisher"
                type="state_publisher" />
              <node
                name="rviz"
                pkg="rviz"
                type="rviz"
                args="-d $(find {})/config/urdf.rviz" />
            </launch>""".format(rospkg)
            
    return str

def generate_gazebo_ros_control():
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    str = """<gazebo>
            <plugin name="gazebo_ros_control" 
                    filename="libgazebo_ros_control.so">
              <robotNamespace>/{}</robotNamespace>
              <robotParam>/robot_description</robotParam>
              <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
            </plugin>
          </gazebo>""".format(robot_name)
    return str
       
def save_gazebo_roslaunch():
    basedir = bpy.context.scene.URDF_properties_tools.file_path
    rospkg = basedir.split('/')[-2:-1][0] # penultimate string is rospkg
    str = generate_gazebo_roslaunch(rospkg, xacro=False)

    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    file = open(basedir+'launch/'+robot_name+'_gazebo.launch', 'w') 
    file.write(str)
    file.close()
    print('saved gazebo roslaunch file') 
    
    
def generate_joint_controller_gains_config():

    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    
    str = """{}:
          # Publish all joint states -----------------------------------
          joint_state_controller:
            type: joint_state_controller/JointStateController
            publish_rate: 5""".format(robot_name)
    str += """

          # Position Controllers ---------------------------------------
          """
          
    for gains in joint_controller_gains:
        str += gains+"\n"
            
    return str

def save_joint_controller_gains_config():
    basedir = bpy.context.scene.URDF_properties_tools.file_path
    path = basedir+'/config/joint_controller_gains.yaml'
    
    print(joint_controller_gains)
    
    str = generate_joint_controller_gains_config()
    
    file = open(path, 'w')
    file.write(str)
    file.close()
    print('saved joint_controller_gains file') 
    
#--

def generate_gazebo_spawner_roslaunch(rospkg, xacro=True):
    
    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    rospkg = rospkg
    urdf_file = "$(find {})/urdf/{}.urdf".format(rospkg, robot_name)

    str = """<?xml version="1.0" encoding="utf-8"?>
            <launch>
              <!-- Start position of robot in scene. -->
              <arg name="init_pose" default="-x 0 -y 0 -z 0"/>
              <arg name="robot_name" default="{}" />

              <!-- Load joint controller configurations from YAML file to parameter server -->
              <rosparam file="$(find {})/config/joint_controller_gains.yaml" command="load"/>

              <!-- load the controllers -->
              <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
                output="screen" ns="{}" args="joint_state_controller""".format(robot_name, rospkg, robot_name)
    for joint_str in joint_controller_gains: 
        joint = joint_str.split(': ')[2].split('_joint')[0] # jankkky
        str += "\n\t\t\t\t\t\t\t\t\t "+joint+"_controller"
    str += """ "/>\n """

    if not xacro:
        str += """
              <param
                name="robot_description"
                textfile="$(find {})/urdf/{}.urdf" />
              <node
                name="spawn_model"
                pkg="gazebo_ros"
                type="spawn_model"
                args="-param robot_description -urdf -model {}"
                output="screen" />""".format(rospkg, robot_name, robot_name)
    else:
        str += """
              <param
                name="robot_description"
                command="$(find xacro)/xacro $(find {})/urdf/{}.xacro" />
              <node
                name="spawn_model"
                pkg="gazebo_ros"
                type="spawn_model"
                args="-param robot_description -urdf -model {}"
                output="screen" />""".format(rospkg, robot_name, robot_name)
    
    str+=  """<!-- Publish carre tf's. -->
              <node pkg="robot_state_publisher" type="robot_state_publisher" name="elevate_state_publisher" />

              <!--node pkg="interactive_marker_twist_server" type="marker_server" name="twist_marker_server">
                <remap from="~cmd_vel" to="cmd_vel" />
              </node-->

            </launch>"""         
            
    return str        
                
def save_gazebo_spawner_roslaunch():
    basedir = bpy.context.scene.URDF_properties_tools.file_path
    rospkg = basedir.split('/')[-2:-1][0] # penultimate string is rospkg
    str = generate_gazebo_spawner_roslaunch(rospkg, xacro=False)

    robot_name = bpy.context.scene.URDF_properties_tools.robot_name
    file = open(basedir+'launch/'+'spawner_gazebo.launch', 'w') 
    file.write(str)
    file.close()
    print('saved gazebo spawn roslaunch file') 
                
##############
          
          
class URDF_properties(bpy.types.PropertyGroup):
    file_path: bpy.props.StringProperty(name="Path",
                                        description="Some elaborate description",
                                        default="",
                                        maxlen=1024,
                                        subtype="FILE_PATH")    
    robot_name: bpy.props.StringProperty(name="Name",
                                        description="Some elaborate description",
                                        default="",
                                        maxlen=1024)
    export_type: bpy.props.EnumProperty(
        name="Selection",
        items=(
               ('stl', 'stl', 'Export objects in stl files'),
               ('obj', 'obj', 'Export objects in obj files'),
            ),
        )
    decimate_ratio: bpy.props.FloatProperty(name="Decimate Ratio",
                             description="Percentage to decimate object by before exporting",
                             default=1.00,
                             min=0.00,
                             max=1.00)
#    save_rviz: bpy.props.BoolProperty(name="RViz Roslaunch",
#                                      description="Create and save rviz roslaunch file")
#    save_gazebo: bpy.props.BoolProperty(name="Gazebo Roslaunch",
#                                      description="Create and save gazebo roslaunch file")
#    save_gazebo_spawner: bpy.props.BoolProperty(name="Gazebo Spawner",
#                                      description="Create and save gazebo spawner roslaunch file")                                                                    
                                                                               
                              
# Panel display
class URDF_PT_PANEL(bpy.types.Panel):
    bl_idname = "URDF_PT_PANEL"
    bl_label = "URDF Creator"
    bl_category = "URDF Creator"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        URDF_properties_tools = context.scene.URDF_properties_tools
        
        layout.label(text="1. Name the Robot")   
        row = layout.row()
        row.prop(URDF_properties_tools, "robot_name")
        
        layout.label(text="2. Select save location (abspath)")
        row = layout.row()
        row.prop(URDF_properties_tools, "file_path")
   
        layout.label(text="3. Choose Export Type")     
        row = layout.row()
        row.prop(URDF_properties_tools, "export_type")
        
        layout.label(text="4. Set Decimate Ratio")     
        row = layout.row()
        row.prop(URDF_properties_tools, "decimate_ratio")

        layout.label(text="5. Select Root Object")
        
        #layout.label(text="4. Select Additional Files to Save")
        #row = layout.row()
        #row.prop(URDF_properties_tools, "save_rviz")
        #row = layout.row()
        #row.prop(URDF_properties_tools, "save_gazebo")
        #row = layout.row()
        #row.prop(URDF_properties_tools, "save_gazebo_spawner")
        
        layout.label(text="6. Generate URDF")
        row = layout.row()
        row.scale_y = 2.0
        row.operator(".generate_urdf", text="Generate URDF")
        

class GenerateURDF_Operator(bpy.types.Operator):
    bl_idname = ".generate_urdf"
    bl_label = "Simple operator"
    bl_description = "Generate .urdf links and joints through tree traversal, "

    def execute(self, context):
        global urdf_links, urdf_joints, urdf_transmissions, joint_controller_gains
        urdf_links = []
        urdf_joints = [] # need this here to zero-out global vals
        urdf_transmissions = []
        joint_controller_gains = []
        
        root = bpy.context.selected_objects[0]
        export_type = bpy.context.scene.URDF_properties_tools.export_type
        generate_root_link(root, export_type)
        treeTraversal(root, export_type)
        #print(urdf_links)
        
        urdf_str = generate_urdf_str(urdf_links, urdf_joints)
        save_urdf_to_file(urdf_str)
        save_rviz_roslaunch()
        save_gazebo_roslaunch()
        save_joint_controller_gains_config()
        save_gazebo_spawner_roslaunch()
        
        
        #if bpy.context.scene.URDF_properties_tools.save_rviz:           save_rviz_roslaunch()
        #if bpy.context.scene.URDF_properties_tools.save_gazebo:         save_gazebo_roslaunch()
        #if bpy.context.scene.URDF_properties_tools.save_gazebo_spawner: save_gazebo_spawner_roslaunch()
        
        bpy.ops.object.select_all(action='DESELECT')
        root.select_set(True)
        bpy.ops.view3d.snap_cursor_to_selected()
        
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
    



