
from .load_robot import (
        config, client, tree,
        occurrences, getOccurrence,
        frames,findInstance
)

from .utility import (transform_to_pos_and_euler,
                        getMeshName,
                        get_inetia_prop,
                        get_body,
                        dict_to_tree,
                        dic_to_assembly,
                        set_parnt_child_relation_for_assembly,
                        get_base_entity,
                        find_relation,
                        cunstruct_relation_tree,
                        entity_node_to_node,
                        create_body_and_joint_poses
                        )

import xml.dom.minidom

from .components import (MujocoGraphState,
                        refactor_joint,
                        refactor_geom,
                        Default,
                        Tree
                        )

from .onshape_mjcf import (Entity,
                           EntityType,
                           Assembly,
                           Part,
                           OnshapeState,
                           EntityNode,
                           JointType,
                           JointData
                           )
import numpy as np



def create_mjcf(tree:dict)->str:

    # sotres defaults and assents
    mj_state = MujocoGraphState()

    #Transfrom: Rotation and translation matrix
    matrix = np.matrix(np.identity(4))
    # base pose
    b_pose = [0]*6

    # create a tree and returns root node (root body)
    root_body = dict_to_tree(tree,mj_state,matrix,b_pose)

    j_attribiutes_common_in_all_elements,j_classes = refactor_joint(root_body,mj_state)
    g_attribiutes_common_in_all_elements,g_classes = refactor_geom(root_body,mj_state)

    # create super default
    super_joint_default = Default(
        name=None,
        element_type="joint",
        attrbutes = j_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.joint_state.get_element(id) \
            for id in j_attribiutes_common_in_all_elements[0]
        ]
    )
    super_geom_default = Default(
        name=None,
        element_type="geom",
        attrbutes = g_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.geom_state.get_element(id) \
            for id in g_attribiutes_common_in_all_elements[0]
        ]
    )
    # creating named defaults
    named_defaults = []
    for j_class, ids_attributes_tuple in  j_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=j_class,
                element_type="joint",
                attrbutes = attributes,
                elements = [
                    mj_state.joint_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    for g_class, ids_attributes_tuple in  g_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=g_class,
                element_type="geom",
                attrbutes = attributes,
                elements = [
                    mj_state.geom_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    # creating tree
    tree = Tree(
        root = root_body,
        super_defaults = [super_joint_default,super_geom_default],
        named_defaults = named_defaults,
        state = mj_state
    )

    # assining classes and cleaning up tree
    tree.refactor()


    # Parse the XML string
    dom = xml.dom.minidom.parseString(tree.xml())

    # Pretty print the XML string
    pretty_xml_as_string = dom.toprettyxml()

    # Remove the XML declaration
    pretty_xml_as_string = '\n'.join(pretty_xml_as_string.split('\n')[1:])
    # print(pretty_xml_as_string)


    file_path = 'iiwa14/model.xml'

    with open(file_path, 'w') as file:
        file.write(pretty_xml_as_string)


def create_mjcf_assembly_tree(assembly:dict):

    # print(f"config::{config}")

    state = OnshapeState()

    root = assembly["rootAssembly"]
    subassemblies = None
    if len(assembly["subAssemblies"])>0:
        subassemblies = assembly["subAssemblies"]
    # print(f"root_features::{root_features}")

    # for feature in root_features:
    #     print("\n\n")
    #     print(f"feature::{feature}")
    # return

    # print("\n\n")
    # print(f"root::keys::{root.keys()}")

    ####### This code was  copy pasted from load_robot.py ########
    # Collecting occurrences, the path is the assembly / sub assembly chain
    occurrences = {}
    for occurrence in root["occurrences"]:
        occurrence["instance"] = findInstance(occurrence["path"])
        occurrence["transform"] = np.matrix(np.reshape(occurrence["transform"], (4, 4)))

        occurrences[tuple(occurrence["path"])] = occurrence
    ###### END ########

    root_assemby = Assembly(
        e_id = "root",
        e_type = EntityType.Assembly,
        joint = JointData(),
        name = config["assemblyName"],
        element_id  = root["elementId"],
        document_id = root["documentId"],
        occerance = None,
        instance = None,
        features = None,
        fullConfiguration = root['fullConfiguration']
    )
    state.add_assembly(root_assemby)
    dic_to_assembly(state,root,occurrences,subassemblies,root_assemby)

    # for part in root_assemby.parts:
    #     print(f"root_assemby::part::name::{part.name}")

    # for part in root_assemby.assemblies[0].parts:
        # print(f"root_assemby.assemblies[0]::part::name::{part.name}")

    # print(f"state::parts::keys::{state.parts.keys()}")
    # print(f"state::assemblies::keys::{state.assemblies.keys()}")

    return root_assemby,state

def create_models(assembly_tree:Assembly,onshape_state:OnshapeState):
    print("\n")
    # print(f"assembly_tree::part::id::{[part.e_id for part in assembly_tree.parts]}")
    # print(f"assembly_tree::assembly::id::{[assembly.e_id for assembly in assembly_tree.assemblies]}")

    # print(f"onshape_state::parts_id::{onshape_state.parts_id}")
    # print(f"onshape_state::part_name::{[ onshape_state.get_part(id).name for id in  onshape_state.parts_id ]}")
    # print(f"onshape_state::assemblies_id::{onshape_state.assemblies_id}")
    # print(f"onshape_state::assembly_name::{[ onshape_state.get_assembly(id).name for id in  onshape_state.assemblies_id ]}")
    print("\n")

    # print(f"assembly_tree::features::len::{len(assembly_tree.features)}")

    set_parnt_child_relation_for_assembly(assembly_tree,onshape_state)
    base_entity = get_base_entity(assembly_tree)
    # print(f"base_entity::{base_entity}")
    assembly_tree.root_entity = base_entity

    # print(f"assembly_tree::relations::len::{len(assembly_tree.relations)}")




    #cunstruct relations for assemblies
    entitiy_node = EntityNode(
        assembly_tree,
        assembly_tree.e_type
    )

    cunstruct_relation_tree(entitiy_node,assembly_tree,onshape_state)

    # print(f"entitiy_node.children::{[ch.entity.name for ch in entitiy_node.children]}")
    # print(f"base::link1::link2::{entitiy_node.children[1].children[0].entity.name}")


    # sotres defaults and assents
    mj_state = MujocoGraphState()

    #Transfrom: Rotation and translation matrix
    matrix = np.matrix(np.identity(4))
    # base pose
    b_pose = [0]*6

    # create_body_and_joint_poses(entitiy_node,mj_state,matrix,b_pose)

    # return
    root_node = entity_node_to_node(entitiy_node,mj_state,matrix,b_pose)

    # print(f"root_node::name::{root_node.name}\n")
    # print(f"root_node::prop::{root_node.prop}\n")
    # print(f"root_node::children::{[child.name for child in root_node.children]}\n")
    # print(f"root_node::children[0]::children::{[child.name for child in root_node.children[0].children]}\n")
    # print(f"root_node::children[1]::children::{[child.name for child in root_node.children[1].children]}\n")

    j_attribiutes_common_in_all_elements,j_classes = refactor_joint(root_node,mj_state)
    g_attribiutes_common_in_all_elements,g_classes = refactor_geom(root_node,mj_state)

    # create super default
    super_joint_default = Default(
        name=None,
        element_type="joint",
        attrbutes = j_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.joint_state.get_element(id) \
            for id in j_attribiutes_common_in_all_elements[0]
        ]
    )
    super_geom_default = Default(
        name=None,
        element_type="geom",
        attrbutes = g_attribiutes_common_in_all_elements[1],
        elements = [
            mj_state.geom_state.get_element(id) \
            for id in g_attribiutes_common_in_all_elements[0]
        ]
    )
    # creating named defaults
    named_defaults = []
    for j_class, ids_attributes_tuple in  j_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=j_class,
                element_type="joint",
                attrbutes = attributes,
                elements = [
                    mj_state.joint_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    for g_class, ids_attributes_tuple in  g_classes.items():
        ids,attributes = ids_attributes_tuple
        named_defaults.append(
            Default(
                name=g_class,
                element_type="geom",
                attrbutes = attributes,
                elements = [
                    mj_state.geom_state.get_element(id) \
                    for id in ids
                ]
            )
        )

    # creating tree
    tree = Tree(
        root = root_node,
        super_defaults = [super_joint_default,super_geom_default],
        named_defaults = named_defaults,
        state = mj_state
    )

     # assining classes and cleaning up tree
    tree.refactor()


    # Parse the XML string
    dom = xml.dom.minidom.parseString(tree.xml())

    # Pretty print the XML string
    pretty_xml_as_string = dom.toprettyxml()

    # Remove the XML declaration
    pretty_xml_as_string = '\n'.join(pretty_xml_as_string.split('\n')[1:])
    # print(pretty_xml_as_string)


    file_path = './model.xml'

    with open(file_path, 'w') as file:
        file.write(pretty_xml_as_string)

    # print(tree.xml())





















