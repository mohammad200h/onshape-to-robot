
from .load_robot import \
        config, client, tree, occurrences, getOccurrence, frames

from .utility import (transform_to_pos_and_euler,
                        getMeshName,
                        get_inetia_prop,
                        get_body,
                        dict_to_tree
                        )

import xml.dom.minidom



def create_mjcf(tree:dict)->str:
    # print(f"tree.keys()::{tree.keys()}")
    # print("######## tree ###########")
    # print(f"tree::{tree}")
    # print("######## End::tree ###########")
    root_body = dict_to_tree(tree)
#     graph_state = MujocoGraphState()
#     root_body.refactor(graph_state)


    # Parse the XML string
    dom = xml.dom.minidom.parseString(root_body.xml())

    # Pretty print the XML string
    pretty_xml_as_string = dom.toprettyxml()

    # Remove the XML declaration
    pretty_xml_as_string = '\n'.join(pretty_xml_as_string.split('\n')[1:])
    print(pretty_xml_as_string)


