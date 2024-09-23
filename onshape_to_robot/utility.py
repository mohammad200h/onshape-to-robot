from typing import List,Dict,Any
from .components import (Body,Geom,Joint,Inertia,
                        BodyElements,
                        Material,
                        MujocoGraphState)
import math
import numpy as np
from .load_robot import \
     config, client, tree, occurrences, getOccurrence, frames,get_T_part_mate

from .features import readParameterValue

from .onshape_mjcf import ( Entity,
                            EntityType,
                            Assembly,
                            Part,
                            OnshapeState,
                            EntityNode,
                            JointType,
                            JointData
                            )

from .features import init as features_init

from uuid import uuid4,UUID

from colorama import Fore, Back, Style

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

def find_occerance_key(occurrences,instance_id):
    for key in occurrences.keys():
        if instance_id in key:
            return key
    return None

def dic_to_assembly(state:OnshapeState,assembly_dic:dict,occurrences:dict,
                    sub_assemblies:dict,root_assemby:Assembly):

    root_instances = assembly_dic["instances"]
    root_features  = assembly_dic["features"]
    root_assemby.features = root_features

    for instance in root_instances:
        if instance["type"]=="Part":

            key = find_occerance_key(occurrences,instance["id"])
            occerance = occurrences[key]
            justPart,prefix,part = getMeshName(occerance)
            color = get_color(part)
            mass,inertia,com = get_inetia_prop(prefix,part)

            part = Part(
                e_id = instance["id"],
                e_type = EntityType.PART,
                joint = JointData(),
                name =  instance["name"],
                element_id = instance["elementId"],
                document_id = instance["documentId"],
                documentMicroversion = instance["documentMicroversion"],
                configuration =  instance["configuration"],
                occerance = occerance,
                instance = instance,
                mesh_info = {"justPart":justPart,"prefix":prefix,"part":part},
                inertia_info = {"mass":mass,"inertia":inertia,"com":com},
                color = color,
                transform = occerance["transform"],

            )
            state.add_part(part)
            root_assemby.add_part(part)
        elif instance["type"]=="Assembly":
            occerance = occurrences[key]
            # print("type::assembly")
            # print(f"dic_to_assembly::instance::name::{instance['name']}")
            # print(f"dic_to_assembly::occurrences::{occurrences.keys()}")
            # print(f"dic_to_assembly::instance::id::{instance['id']}")
            # print(f"dic_to_assembly::occerance::transform::{occerance['transform']}")

            # print(f"dic_to_assembly::assembly::{assembly}")

            assembly =  Assembly(
                e_id = instance["id"],
                e_type =  EntityType.Assembly,
                joint = JointData(),
                name = instance["name"],
                element_id = instance["elementId"],
                document_id = instance["documentId"],
                occerance = occerance,
                instance = instance,
                features = None,
                fullConfiguration = instance["fullConfiguration"]

            )
            state.add_assembly(assembly)
            for sub in sub_assemblies:
                # print(f"sub::{sub}")
                # print(f"sub::documentId::{sub['documentId']}")
                # print(f"sub::instance::{instance['id']}")
                if sub["elementId"] == instance["elementId"]:
                    sub_assemblies = assembly_dic["subAssemblies"] if "subAssemblies" in assembly_dic.keys() else []
                    dic_to_assembly(state,sub,occurrences,sub_assemblies,assembly)
                    break
            root_assemby.add_assembly(assembly)

def set_parnt_child_relation_for_assembly(assembly:Assembly,onshape_state:OnshapeState):
    relations = []
    counter = 0
    # print(f"assembly.features::{assembly.features}")
    for feature in assembly.features:
        # print(f"set_parnt_child_relation_for_assembly::feature::{feature}")
        if feature['featureType']=='mate':
            if feature['featureData']['mateType'] == 'REVOLUTE':
                # print(f"feature['featureData']::{feature['featureData']}\n")
                feature_id = feature['id']
                mated_entities = feature['featureData']['matedEntities']
                counter +=1
                relation = {
                    "child":{
                        "part":None,
                        "assembly":None,
                        "joint":{
                            "type":JointType(feature['featureData']['mateType'].lower()),
                            "name": feature['featureData']['name'],
                            "z_axis":mated_entities[0]['matedCS']['zAxis'],
                            "assembly_owning_feature":assembly
                        },
                        "mated_entity":mated_entities[0]
                    },
                    "parent":{
                        "part":None,
                        "assembly":None
                    }

                }

                # first element is child
                # first mate should be made on child link
                # this is an assumption that is made and
                # explained in onshape Doc
                child = mated_entities[0]['matedOccurrence']

                # second element is parnt
                parent = mated_entities[1]['matedOccurrence']


                # in case part is part of assembly the list
                # will contain both part and assembly
                # if the part is with in root assembly the
                # list will have one element which is the part

                for e_id in child:
                    entity = onshape_state.get_entity(e_id)
                    if entity["type"]=="part":
                        relation["child"]["part"] = entity
                    else:
                        relation["child"]["assembly"] = entity
                for e_id in parent:
                    entity = onshape_state.get_entity(e_id)
                    if entity["type"]=="part":
                        relation["parent"]["part"] = entity
                    else:
                        relation["parent"]["assembly"] = entity
                relations.append(relation)

    # print(f"set_parnt_child_relation_for_assembly::counter::{counter}")
    assembly.relations = relations

def get_base_entity(assembly_tree:Assembly):
    for relation in assembly_tree.relations:
        parent_name = relation["parent"]["part"]["name"]
        if "base" in parent_name or "Base" in parent_name:
            return relation["parent"]['part']['entity']

    return None

def find_relation(relations:List,parent:Entity):
    children = []
    # print(f"find_relation::parent::{parent}")
    # print(f"find_relation::relations::len::{len(relations)}")
    for relation in relations:
        p = relation['parent']
        # print(f"find_relation::p::{p}")

        if p['part'] and p['part']['entity'].e_id == parent.e_id:
            children.append( relation["child"])
        elif p['assembly'] and p['assembly']['entity'].e_id == parent.e_id:
            children.append( relation["child"])
    return children

def cunstruct_relation_tree(entity_node:EntityNode,current_assembly:Assembly,onshape_state:OnshapeState):
    # print(f"cunstruct_relation_tree::EntityNode::e_type::{entity_node.e_type}")
    children = []
    if entity_node.e_type == EntityType.Assembly:
        parent = entity_node.entity.root_entity
        children = find_relation(current_assembly.relations,parent)
        # print(f"cunstruct_relation_tree::assembly::children::len::{len(children)}")


    elif entity_node.e_type == EntityType.PART:
        parent = entity_node.entity
        # print(f"cunstruct_relation_tree:: entity_node.e_type == EntityType.PART::parent::{parent.name}")
        children = find_relation(current_assembly.relations,parent)
        # print(f"cunstruct_relation_tree::part::children::len::{len(children)}")

    for child in children:
        if child['assembly']:
            # print("child is assembly")
            child['assembly']['entity'].joint =JointData(
                child['joint']['name'],
                child['joint']['type'],
                child['joint']['z_axis'],
                child['mated_entity'],
                child['joint']['assembly_owning_feature']
            )
            child_entitiy_node =EntityNode (
                child['assembly']['entity'],
                child['assembly']['entity'].e_type,
            )
            entity_node.add_child(child_entitiy_node)
            # print(f"child is assembly::child['part']['entity']::{child['part']['entity']}")
            #set root of the assembly
            child['assembly']['entity'].root_entity = child['part']['entity']
            set_parnt_child_relation_for_assembly(child['assembly']['entity'],onshape_state)
            cunstruct_relation_tree(child_entitiy_node,child['assembly']['entity'],onshape_state)
            continue
        elif child['part']:
            # print("child is part")
            print(f"MJCF::child::joint::mated_entity::{child['mated_entity']}")
            child['part']['entity'].joint = JointData(
                child['joint']['name'],
                child['joint']['type'],
                child['joint']['z_axis'],
                child['mated_entity'],
                child['joint']['assembly_owning_feature']
            )
            child_entitiy_node = EntityNode(
                child['part']['entity'],
                child['part']['entity'].e_type
            )
            entity_node.add_child(child_entitiy_node)
            cunstruct_relation_tree(child_entitiy_node,current_assembly,onshape_state)

def get_worldAxisFrame2(part):


    T_world_part = part.transform
    print(f"MJCF::T_world_part::{T_world_part}")


    print(f"get_worldAxisFrame2::part.joint.feature['featureData']::{part.joint.feature['featureData']}")
    T_part_mate = get_T_part_mate(part.joint.feature['featureData']['matedEntities'][0])
    # T_world_part = transform
    print(f"MJCF::T_part_mate::{T_part_mate}")
    print(f"MJCF::T_world_part::{T_world_part}")
    # The problem is T_world_part which is different for URDF
    T_world_mate = T_world_part * T_part_mate
    # T_world_mate = T_world_part * T_part_mate
    worldAxisFrame = T_world_mate

    return worldAxisFrame

def get_worldAxisFrame(occurrences,matedEntity):
    print(f"get_worldAxisFrame::matedEntity::{matedEntity}")
    matedOccurrence = tuple(matedEntity["matedOccurrence"])
    root_id ='030d70476fdd8a6377c2a5dc'
    occ_key = root_id+"_"+"_".join(matedOccurrence)
    print(f"get_worldAxisFrame::occ_key::{occ_key}")
    print("\n")
    for key,value in occurrences.items():
        if key == occ_key:
            print(f"Found it::{matedOccurrence}")
            print(f"occurrenc::{occurrences[occ_key]}")


    print("\n")
    print(f"get_worldAxisFrame:occurrences::keys::{occurrences.keys()}")
    print(f"get_worldAxisFrame::occurrences[occ_key]::{occurrences[occ_key]}")

    T_world_part = occurrences[occ_key]["transform"]
    print(f"MJCF::T_world_part::{T_world_part}")


    print(f"get_worldAxisFrame::matedEntity::id::{matedEntity}")
    T_part_mate = get_T_part_mate(matedEntity)
    # T_world_part = transform
    print(f"MJCF::T_part_mate::{T_part_mate}")
    print(f"MJCF::T_world_part::{T_world_part}")
    # The problem is T_world_part which is different for URDF
    T_world_mate = T_world_part * T_part_mate
    # T_world_mate = T_world_part * T_part_mate
    worldAxisFrame = T_world_mate

    return worldAxisFrame

def getLimits(jointType, name,joint_features):

    print(f"joint_features::{joint_features}")

    enabled = False
    minimum, maximum = 0, 0
    for feature in joint_features['features']:
        # Find coresponding joint
        if name == feature['message']['name']:
            # Find min and max values
            for parameter in feature['message']['parameters']:
                if parameter['message']['parameterId'] == "limitsEnabled":
                    enabled = parameter['message']['value']

                if jointType == 'revolute':
                    if parameter['message']['parameterId'] == 'limitAxialZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitAxialZMax':
                        maximum = readParameterValue(parameter, name)
                elif jointType == 'prismatic':
                    if parameter['message']['parameterId'] == 'limitZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitZMax':
                        maximum = readParameterValue(parameter, name)
    if enabled:
        return (minimum, maximum)
    else:
        if jointType != 'continuous':
            print(Fore.YELLOW + 'WARNING: joint ' + name + ' of type ' +
                jointType + ' has no limits ' + Style.RESET_ALL)
        return None

def feature_init(fullConfiguration, workspaceId, assemblyId):
    # Load joint features to get limits later
    if config['versionId'] == '':
        joint_features = client.get_features(
            config['documentId'], workspaceId, assemblyId)
    else:
        joint_features = client.get_features(
            config['documentId'], config['versionId'], assemblyId, type='v')

    # Retrieving root configuration parameters
    configuration_parameters = {}
    parts = fullConfiguration.split(';')
    # print(f"init::parts::{parts}")
    for part in parts:
        kv = part.split('=')
        if len(kv) == 2:
            configuration_parameters[kv[0]] = kv[1].replace('+', ' ')
    return configuration_parameters, joint_features

def get_joint_limit(joint:JointData):
    assembly = joint.owner
    assemblyId = assembly.element_id
    fullConfiguration = assembly.fullConfiguration
    document = client.get_document(assembly.document_id).json()
    workspaceId = document["defaultWorkspace"]["id"]

    configuration_parameters, joint_features = feature_init(fullConfiguration, workspaceId, assemblyId)

    limit = getLimits(joint.j_type,joint.name,joint_features)
    # print(f"get_joint_limit::joint.name::{joint.name}:: limit::{limit}")
    return limit

def get_joint_limit2(joint):
    assembly_info = joint.assemblyInfo
    document = client.get_document(assembly_info['documentId']).json()
    workspaceId = document["defaultWorkspace"]["id"]

    configuration_parameters, joint_features = feature_init(
        assembly_info['fullConfiguration'],
        workspaceId,
        assembly_info['assemblyId']
        )
    # print(f"get_joint_limit2::joint_features::{joint_features}")

    limit = getLimits(joint.j_type.lower(),joint.name,joint_features)
    return limit

def entity_node_to_node(entity_node:EntityNode,
                        graph_state:MujocoGraphState,
                        matrix,body_pose,
                        occurences):
    entity = None
    if entity_node.e_type == EntityType.PART:
        entity = entity_node.entity
        # print(f"part::entity::{entity.name}")
    elif entity_node.e_type == EntityType.Assembly:
        # print(f"assembly::entity::{entity_node.entity.name}")
        entity = entity_node.entity.root_entity
        if entity_node.entity.joint.name:
            # print(f"assembly::entity::joint::name::{entity_node.entity.joint.name}")
            # print(f"assembly::entity::joint::owner::{entity_node.entity.joint.owner}")
            limit = get_joint_limit(entity_node.entity.joint)
            # print(f"assembly::entity::limit::{limit}")

        # print(f"part::root_entity::{entity.name}")
        # print(f"entity_node_to_node::EntityType.Assembly::entity::{entity}")
    #urdf -> base link pose
    pose = entity.occerance["transform"]
    pose = np.linalg.inv(matrix)*pose
    xyz,rpy,quat = transform_to_pos_and_euler(pose)
    # print("\n")
    # print(f"urdf::body::entity_node_to_node::pose::{pose}")
    # print(f"urdf::body::entity_node_to_node::xyz::{xyz}")

    # getting info useful for geom
    instance = entity.occerance["instance"]
    link_name = processPartName(
        instance['name'], instance['configuration'],
        entity.occerance['linkName']
    )
    justPart, prefix,part = getMeshName(entity.occerance)

    graph_state.assets.add_mesh(justPart+".stl")

    rgba = get_color(part)

    c_name = get_color_name(rgba)
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
    if entity_node.e_type == EntityType.PART:
        if entity.joint.name:
            # print(f"entity.joint::{entity.joint}")
            # print(f"there is a joint::{entity.joint.name:}")
            limits = get_joint_limit(entity.joint)
            # print(f"limits::{limits}")
            joint = Joint(
                name = entity.joint.name,
                j_type=translate_joint_type_to_mjcf(entity.joint.j_type.value),
                j_range=limits,
                axis=tuple(entity.joint.z_axis),
                id = uuid4()
            )
    elif entity_node.e_type == EntityType.Assembly:
        if entity_node.entity.joint.name:
            limits = get_joint_limit(entity_node.entity.joint)
            joint = Joint(
                name = entity_node.entity.joint.name,
                j_type=translate_joint_type_to_mjcf(entity_node.entity.joint.j_type.value),
                j_range=limits,
                axis=tuple(entity_node.entity.joint.z_axis),
                id = uuid4()
            )
    if joint:
        graph_state.joint_state.add(joint.to_dict(),joint)

    body_elem = BodyElements(inertia,geom,joint)
    node = Body(prop=body_elem,name=link_name,position=tuple(body_pose[:3]),euler=tuple(body_pose[3:]))

    for child in entity_node.children:
        ###############
        worldAxisFrame = get_worldAxisFrame(occurences,child.entity.joint.mated_entity)
        # print(f"worldAxisFrame::{worldAxisFrame}\n\n")
        axisFrame = np.linalg.inv(matrix)*worldAxisFrame
        childMatrix = worldAxisFrame
        ###############
        xyz,rpy,quat = transform_to_pos_and_euler(axisFrame)

        child_node = entity_node_to_node(child,graph_state,childMatrix, list(xyz)+list(rpy),occurences )
        node.add_child(child_node)

    return node

def part_trees_to_node(part,matrix,body_pose,graph_state:MujocoGraphState):

    pose = part.transform
    pose = np.linalg.inv(matrix)*pose
    xyz,rpy,quat = transform_to_pos_and_euler(pose)

    instance = part.occurence["instance"]
    link_name = processPartName(
        instance['name'], instance['configuration'],
        part.occurence['linkName']
    )
    justPart, prefix,part_ = getMeshName(part.occurence)

    graph_state.assets.add_mesh(justPart+".stl")

    rgba = get_color(part_)

    c_name = get_color_name(rgba)
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

    # getting inertia
    mass,intertia_props,com = get_inetia_prop(prefix,part_)
    i_prop_dic = compute_inertia(matrix,com,intertia_props)
    inertia = Inertia(
        pos= i_prop_dic["com"],
        mass = mass,
        fullinertia=i_prop_dic["inertia"]
    )
    joint= None
    if part.joint:
        limits = get_joint_limit2(part.joint)
        joint = Joint(
                name = part.joint.name,
                j_type=translate_joint_type_to_mjcf(part.joint.j_type.lower()),
                j_range=limits,
                axis=tuple(part.joint.z_axis),
                id = uuid4()
            )
        graph_state.joint_state.add(joint.to_dict(),joint)

    body_elem = BodyElements(inertia,geom,joint)
    node = Body(prop=body_elem,name=link_name,position=tuple(body_pose[:3]),euler=tuple(body_pose[3:]))

    for child in part.children:
        ###############
        worldAxisFrame = get_worldAxisFrame2(child)
        # print(f"worldAxisFrame::{worldAxisFrame}\n\n")
        axisFrame = np.linalg.inv(matrix)*worldAxisFrame
        childMatrix = worldAxisFrame
        ###############
        xyz,rpy,quat = transform_to_pos_and_euler(axisFrame)

        child_node = part_trees_to_node(child,childMatrix, list(xyz)+list(rpy),graph_state )
        node.add_child(child_node)

    return node