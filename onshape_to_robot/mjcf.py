
from .load_robot import \
        config, client, tree, occurrences, getOccurrence, frames

from .utility import (transform_to_pos_and_euler,
                        getMeshName,
                        get_inetia_prop,
                        get_body,
                        dict_to_tree
                        )

import xml.dom.minidom

from .components import  MujocoGraphState,refactor_joint,refactor_geom



def create_mjcf(tree:dict)->str:
    # print(f"tree.keys()::{tree.keys()}")
    # print("######## tree ###########")
    # print(f"tree::{tree}")
    # print("######## End::tree ###########")
    mj_state = MujocoGraphState()
    root_body = dict_to_tree(tree,mj_state)

    attribiutes_common_in_all_elements,classes = refactor_joint(root_body,mj_state)
    print(f"joint::attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    print(f"joint::classes::{classes}")


    attribiutes_common_in_all_elements,classes = refactor_geom(root_body,mj_state)
    print(f"geom::attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    print(f"geom::classes::{classes}")



    # Parse the XML string
#     dom = xml.dom.minidom.parseString(root_body.xml())

    # Pretty print the XML string
#     pretty_xml_as_string = dom.toprettyxml()

    # Remove the XML declaration
#     pretty_xml_as_string = '\n'.join(pretty_xml_as_string.split('\n')[1:])
#     print(pretty_xml_as_string)


