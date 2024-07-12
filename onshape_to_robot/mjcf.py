
from .load_robot import \
        config, client, tree, occurrences, getOccurrence, frames

from .utility import (transform_to_pos_and_euler,
                        getMeshName,
                        get_inetia_prop,
                        get_body,
                        dict_to_tree
                        )

import xml.dom.minidom

from .components import (MujocoGraphState,
                        refactor_joint,
                        refactor_geom,
                        Default,
                        Tree
                        )
import numpy as np



def create_mjcf(tree:dict)->str:
    # print(f"tree.keys()::{tree.keys()}")
    print("######## tree ###########")
    print(f"tree::{tree}")
    print("######## End::tree ###########")

    mj_state = MujocoGraphState()
    matrix = np.matrix(np.identity(4))
    b_pose = [0]*6
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

    # print(f"tree::{tree.xml()}")

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


