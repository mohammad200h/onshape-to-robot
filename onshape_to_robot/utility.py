from typing import List,Dict,Any
from .components import (Body,Geom,Joint,Inertia,
                        BodyElements,
                        Material,
                        MujocoGraphState)
import math
import numpy as np
from .load_robot import \
     config, client, tree, occurrences, getOccurrence, frames


from .onshape_mjcf import ( Entity,
                            EntityType,
                            Assembly,
                            Part)

from uuid import uuid4,UUID

import requests

def translate_joint_type_to_mjcf(j_type):
    mj_j_type = {
        "revolute":"hinge"
    }
    return mj_j_type[j_type]

def convert_to_snake_case(s):
    s = s.lower()
    s = s.replace(" ", "_")
    return s

#####color api########
def get_color_name(rgb):
    url = f"https://www.thecolorapi.com/id?rgb=rgb({rgb[0]},{rgb[1]},{rgb[2]})"
    response = requests.get(url)
    data = response.json()
    color_name = data['name']['value']
    return convert_to_snake_case(color_name)


####### copied form onshape_to_robot.py #############
partNames = {}

def partIsIgnore(name):
    if config['whitelist'] is None:
        return name in config['ignore']
    else:
        return name not in config['whitelist']
# Adds a part to the current robot link

def addPart(occurrence, matrix):
    part = occurrence['instance']
    if part['suppressed']:
        return
    if part['partId'] == '':
        print(Fore.YELLOW + 'WARNING: Part '+part['name']+' has no partId'+Style.RESET_ALL)
        return
    # Importing STL file for this part
    justPart, prefix = extractPartName(part['name'], part['configuration'])
    extra = ''
    if occurrence['instance']['configuration'] != 'default':
        extra = Style.DIM + ' (configuration: ' + \
            occurrence['instance']['configuration']+')'
    symbol = '+'
    if partIsIgnore(justPart):
        symbol = '-'
        extra += Style.DIM + ' / ignoring visual and collision'
    print(Fore.GREEN + symbol+' Adding part ' +
        occurrence['instance']['name']+extra + Style.RESET_ALL)
    if partIsIgnore(justPart):
        stlFile = None
    else:
        stlFile = prefix.replace('/', '_')+'.stl'
        # shorten the configuration to a maximum number of chars to prevent errors. Necessary for standard parts like screws
        if len(part['configuration']) > 40:
            shortend_configuration = hashlib.md5(
                part['configuration'].encode('utf-8')).hexdigest()
        else:
            shortend_configuration = part['configuration']
        stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'],
                                    part['partId'], shortend_configuration)
        with open(config['outputDirectory']+'/'+stlFile, 'wb') as stream:
            stream.write(stl)
        stlMetadata = prefix.replace('/', '_')+'.part'
        with open(config['outputDirectory']+'/'+stlMetadata, 'w', encoding="utf-8") as stream:
            json.dump(part, stream, indent=4, sort_keys=True)
        stlFile = config['outputDirectory']+'/'+stlFile
    # Import the SCAD files pure shapes
    shapes = None
    if config['useScads']:
        scadFile = prefix+'.scad'
        if os.path.exists(config['outputDirectory']+'/'+scadFile):
            shapes = csg.process(
                config['outputDirectory']+'/'+scadFile, config['pureShapeDilatation'])
    # Obtain metadatas about part to retrieve color
    if config['color'] is not None:
        color = config['color']
    else:
        metadata = client.part_get_metadata(
            part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
        color = [0.5, 0.5, 0.5]
        # XXX: There must be a better way to retrieve the part color
        for entry in metadata['properties']:
            if 'value' in entry and type(entry['value']) is dict and 'color' in entry['value']:
                rgb = entry['value']['color']
                color = np.array(
                    [rgb['red'], rgb['green'], rgb['blue']])/255.0
    # Obtain mass properties about that part
    if config['noDynamics']:
        mass = 0
        com = [0]*3
        inertia = [0]*12
    else:
        if prefix in config['dynamicsOverride']:
            entry = config['dynamicsOverride'][prefix]
            mass = entry['mass']
            com = entry['com']
            inertia = entry['inertia']
        else:
            if part['isStandardContent']:
                massProperties = client.standard_cont_mass_properties(
                    part['documentId'], part['documentVersion'], part['elementId'], part['partId'],config['documentId'], part['configuration'])
            else:
                massProperties = client.part_mass_properties(
                    part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
            if part['partId'] not in massProperties['bodies']:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no dynamics (maybe it is a surface)' + Style.RESET_ALL)
                return
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']
            if abs(mass) < 1e-9:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)
    pose = occurrence['transform']
    if robot.relative:
        pose = np.linalg.inv(matrix)*pose
    robot.addPart(pose, stlFile, mass, com, inertia, color, shapes, prefix)

def extractPartName(name, configuration):
    parts = name.split(' ')
    del parts[-1]
    basePartName = '_'.join(parts).lower()
    # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
    if configuration != 'default' and len(configuration) < 40:
        parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]
    return basePartName, '_'.join(parts).lower()

def processPartName(name, configuration, overrideName=None):
      if overrideName is None:
          global partNames
          _, name = extractPartName(name, configuration)
          if name in partNames:
              partNames[name] += 1
          else:
              partNames[name] = 1
          if partNames[name] == 1:
              return name
          else:
              return name+'_'+str(partNames[name])
      else:
          return overrideName

####### end::copied form onshape_to_robot.py #############

######## copied form robot_description.py ##########
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
######## END::copied form robot_description.py ##########

def rotationMatrixToQuatAngles(transform):
     # Extract rotation matrix
    R = transform[:3, :3]

    # Compute the trace of the rotation matrix
    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S=4*q_w
        q_w = 0.25 * S
        q_x = (R[2, 1] - R[1, 2]) / S
        q_y = (R[0, 2] - R[2, 0]) / S
        q_z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*q_x
        q_w = (R[2, 1] - R[1, 2]) / S
        q_x = 0.25 * S
        q_y = (R[0, 1] + R[1, 0]) / S
        q_z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*q_y
        q_w = (R[0, 2] - R[2, 0]) / S
        q_x = (R[0, 1] + R[1, 0]) / S
        q_y = 0.25 * S
        q_z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*q_z
        q_w = (R[1, 0] - R[0, 1]) / S
        q_x = (R[0, 2] + R[2, 0]) / S
        q_y = (R[1, 2] + R[2, 1]) / S
        q_z = 0.25 * S

    quaternion = np.array([q_w, q_x, q_y, q_z])

    return quaternion

######## copied form onshape_to_robot.py ##########
def extractPartName(name, configuration):
    parts = name.split(' ')
    del parts[-1]
    basePartName = '_'.join(parts).lower()
    # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
    if configuration != 'default' and len(configuration) < 40:
        parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]
    return basePartName, '_'.join(parts).lower()

def get_color(part):
    # Obtain metadatas about part to retrieve color
    if config['color'] is not None:
        color = config['color']
    else:
        # print(f"get_color::part::keys::{part.keys()}")
        # print(f"get_color::part::{part}")
        metadata = client.part_get_metadata(
            part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
        color = [0.5, 0.5, 0.5]
        # XXX: There must be a better way to retrieve the part color
        for entry in metadata['properties']:
            if 'value' in entry and type(entry['value']) is dict and 'color' in entry['value']:
                rgb = entry['value']['color']
                color = np.array(
                    [rgb['red'], rgb['green'], rgb['blue']])/255.0

    return color.tolist()+[1]


######## END::copied form onshape_to_robot.py ##########

def pos_form_trasform(transform):
    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]

    # print(f"pos_form_trasform::transform::shape::{transform.shape}")

    return [x,y,z]

def transform_to_pos_and_euler(transform):
    rpy = rotationMatrixToEulerAngles(transform)
    xyz = pos_form_trasform(transform)
    quat = rotationMatrixToQuatAngles(transform)
    # print(f"quat::{quat}")
    return xyz,rpy,quat

def getMeshName(occurrence):
    part = occurrence['instance']
    justPart, prefix = extractPartName(part['name'], part['configuration'])
    # print(f"getMeshName::part::{part}")
    # print(f"getMeshName::justPart::{justPart}")
    # print(f"getMeshName::prefix::{prefix}")

    return justPart,prefix,part

def get_inetia_prop(prefix,part):
    # Obtain mass properties about that part
    if config['noDynamics']:
        mass = 0
        com = [0]*3
        inertia = [0]*12
    else:
        if prefix in config['dynamicsOverride']:
            entry = config['dynamicsOverride'][prefix]
            mass = entry['mass']
            com = entry['com']
            inertia = entry['inertia']
        else:
            if part['isStandardContent']:
                massProperties = client.standard_cont_mass_properties(
                    part['documentId'], part['documentVersion'], part['elementId'], part['partId'],config['documentId'], part['configuration'])
            else:
                massProperties = client.part_mass_properties(
                    part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
            if part['partId'] not in massProperties['bodies']:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no dynamics (maybe it is a surface)' + Style.RESET_ALL)
                return
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']
            if abs(mass) < 1e-9:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)



    return mass,inertia,com

def compute_inertia(matrix,com,inertia):
    # this is taken from addLinkDynamics
     # Inertia
        I = np.matrix(np.reshape(inertia[:9], (3, 3)))
        R = matrix[:3, :3]
        # Expressing COM in the link frame
        com = np.array(
            (matrix*np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
        # Expressing inertia in the link frame
        inertia = R*I*R.T

        return {
            'com': com,
            'inertia': inertia
        }

def get_body(tree)->(dict,dict):
    """
    input:tree
    ouput:
        - body_dic at the root of tree
        - return child branch of tree
    """
    body_dic={
        "joint":None,
        "geom":None,
        "inertia":None
    }
    ##getting joint if there is any
    if "dof_name" in tree.keys():
        j_dic = {
           "axis_frame": tree["axis_frame"],
           "z_axis": tree["z_axis"],
           "dof_name": tree["dof_name"],
           "jointType": tree["jointType"],
           "jointLimits": tree["jointLimits"],

        }
        body_dic["joint"]=j_dic

    ##getting geom
    id = tree["id"]
    occurrence = getOccurrence([id])
    transform =occurrence["transform"]
    xyz,rpy,quat = transform_to_pos_and_euler(transform)
    instance = occurrence['instance']
    link_name = processPartName(
    instance['name'], instance['configuration'], occurrence['linkName'])
    justPart, prefix,part = getMeshName(occurrence)
    g_dic = {
        "xyz":xyz,
        "rpy":rpy,
        "name":link_name,
        "mesh":justPart
    }
    body_dic["geom"]=g_dic
    ##getting inertia
    xyz,rpy,mass,inertia = get_inetia_prop(prefix,part)
    i_dic = {
        "xyz":xyz,
        "rpy":rpy,
        "mass":mass,
        "matrix":inertia
    }
    body_dic["inertia"] = i_dic
    children = None
    if "children" in tree.keys():
        if len(tree["children"])>0:
            children = tree["children"]

    return body_dic,children

def dict_to_tree(tree: Dict[str, Any],graph_state:MujocoGraphState,matrix,body_pose) -> Body:
    """Converts a dictionary representation of a tree into a Node structure."""
    # getting general info
    id = tree['id']
    occurrence = getOccurrence([id])
    pose =occurrence["transform"]
    pose = np.linalg.inv(matrix)*pose
    xyz,rpy,quat = transform_to_pos_and_euler(pose)


    # getting info useful for geom
    instance = occurrence['instance']
    link_name = processPartName(
    instance['name'], instance['configuration'], occurrence['linkName'])
    justPart, prefix,part = getMeshName(occurrence)

    graph_state.assets.add_mesh(justPart+".stl")

    rgba = get_color(part)
    # print(f"dict_to_tree::rgba::type::{type(rgba)}")

    c_name = get_color_name(rgba)
    # print(f"c_name::{c_name}")
    # print(f"dict_to_tree::color::{color}")
    graph_state.assets.add_material(c_name,rgba)



    geom = Geom(
        id = uuid4(),
        name = justPart,
        pos = tuple(xyz),
        euler = tuple(rpy),
        mesh = justPart,
        material = Material(
            name = c_name,
            rgba = rgba)
    )

    graph_state.geom_state.add(geom.to_dict(),geom)

    # getting inertia
    mass,intertia_props,com = get_inetia_prop(prefix,part)
    i_prop_dic = compute_inertia(matrix,com,intertia_props)
    inertia = Inertia(
        pos= i_prop_dic["com"],
        mass = mass,
        fullinertia=i_prop_dic["inertia"]
    )
    # getting joint if any
    joint= None
    # print(f"dof_name in tree.keys()::{'dof_name' in tree.keys()}")
    # print(f"dict_to_tree::uuid4()::{uuid4()}")
    if 'dof_name' in tree.keys():
        # print(f"dict_to_tree::{tree['z_axis']}")
        joint = Joint(
            name = tree["dof_name"],
            j_type=translate_joint_type_to_mjcf(tree["jointType"]),
            j_range=tree["jointLimits"],
            axis=tuple(tree["z_axis"].tolist()),
            id = uuid4()

        )
        graph_state.joint_state.add(joint.to_dict(),joint)
    body_elem = BodyElements(inertia,geom,joint)

    node = Body(prop=body_elem,name=justPart,position=tuple(body_pose[:3]),euler=tuple(body_pose[3:]))

    for child in tree.get('children', []):
        ###############
        worldAxisFrame = child['axis_frame']
        axisFrame = np.linalg.inv(matrix)*worldAxisFrame
        childMatrix = worldAxisFrame
        ###############
        xyz,rpy,quat = transform_to_pos_and_euler(axisFrame)
        # print(f"the thing::xyz::{xyz}")
        # print(f"the thing::rpy::{rpy}")
        child_node = dict_to_tree(child,graph_state,childMatrix, list(xyz)+list(rpy) )
        node.add_child(child_node)
    return node

def dic_to_assembly(assembly_dic:dict,sub_assemblies:dict,root_assemby:Assembly):

    root_instances = assembly_dic["instances"]
    root_features  = assembly_dic["features"]

    for instance in root_instances:
        if instance["type"]=="Part":
            part = Part(
                e_id = instance["id"],
                e_type = instance["type"],
                name =  instance["name"],
                element_id = instance["elementId"],
                document_id = instance["documentId"],
                documentMicroversion = instance["documentMicroversion"],
                configuration =  instance["configuration"]
            )
            root_assemby.parts.append(part)
        elif instance["type"]=="Assembly":
            assembly =  Assembly(
                e_id = instance["id"],
                e_type =  instance["type"],
                name = instance["name"],
                element_id = instance["elementId"],
                document_id = instance["documentId"]
            )
            for sub in sub_assemblies:
                # print(f"sub::{sub}")
                # print(f"sub::documentId::{sub['documentId']}")
                # print(f"sub::instance::{instance['id']}")
                if sub["elementId"] == instance["elementId"]:
                    sub_assemblies = assembly_dic["subAssemblies"] if "subAssemblies" in assembly_dic.keys() else []
                    dic_to_assembly(sub,sub_assemblies,assembly)
                    break
            root_assemby.assemblies.append(assembly)


