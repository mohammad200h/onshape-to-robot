
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
                        feature_init,
                        getLimits,
                        part_trees_to_node
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
                           Part2,
                           JointData2,
                           OnshapeState,
                           EntityNode,
                           JointType,
                           JointData
                           )
import numpy as np
from PrettyPrint import PrettyPrintTree


def get_part_relations(relations,instance_id,assemblyInstance):
    children = []
    print(f"instance_id::{instance_id}")
    print(f"assemblyInstance::{assemblyInstance}")

    for r in relations:
        print("\n")
        print(f"get_part_relations::r['parent'][0]::{r['parent'][0]}")
        print(f"get_part_relations::r::assemblyInstanceId::{r['assemblyInstanceId']}")
        print("\n")


        if r['parent'][0] == instance_id :
          if assemblyInstance == None:
            children.append(r)
          elif r['assemblyInstanceId'] == assemblyInstance:
            children.append(r)
    return children

def find_occurrence(occurences,occurence_path):
  # assuming that that we are dealing with a part that belong to sub assemblies:
  # occurence_path: [part_instance, assembly_instance]
  # assuming that we are in the root assembly
  # occurence path: [part_instance]


#   print("####### Understaidning Occurence #########")
#     for occ in occurences:
#         if occ['path']==['MWgdS8yELuEC57GO0']:
#             print(f"occ::MWgdS8yELuEC57GO0::{occ}")
#         elif

#   print("####### End::Understaidning Occurence #########")
#   print(f"occurence_path::{occurence_path}")
  for occ in occurences:
    # print(f"occ['path']::{occ['path']}")
    if occ["path"] == occurence_path:
      return occ

def find_occurence(occurences,instanceId:str):
    # I need to rewrite this so that it considers both part and sub-assembly instance.
    for occ in occurences:
      instance = findInstance(occ["path"])
      if instance["id"]== instanceId:
        return occ

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


def get_part_transforms_and_fetuses(assembly:dict):
    # It is possible to get transform of all the parts from root assembly
    # However it is not possible to get the features (mate features from) root assembly
    # Here we will get all the part information form root assembly
    # we will get all the avilable features from root assembly.
    # Then we will dig in the sub-assemblies to get the missing features.

    root = assembly["rootAssembly"]

    assembly_info = {
        'fullConfiguration':root['fullConfiguration'],
        'documentId':root['documentId'],
        'assemblyId':root['elementId']
    }

    if len(assembly["subAssemblies"])>0:
        subassemblies = assembly["subAssemblies"]
        # print(f"get_part_transforms_and_fetuses::subassemblies::{subassemblies}")

    occurences_in_root = {
        "robot_base":None,
        "sub-assemblies":{},
        "parts":[],
        # parts in root assenbly that belong to a sub assembly
        # this happens when a mate between subassembly and root assembly is made
        "sub_assembly_parts":[],
        "relations":None
    }

    for occurrence in root["occurrences"]:
        occurrence["instance"] = findInstance(occurrence["path"])
        typee = occurrence["instance"]['type']
        # assume that fixed link is the base
        is_fixed = occurrence['fixed']

        if is_fixed:
          occurences_in_root["robot_base"] = occurrence['instance']['id']

        if typee == "Assembly":
          data = {
            "documentId":occurrence['instance']['documentId'],
            "elementId":occurrence['instance']['elementId']
          }
          occurences_in_root['sub-assemblies'][occurrence['path'][0]] = data
        elif typee == 'Part':
          occurences_in_root['parts'].append(occurrence['path'])
    # print(f"occurences_in_root['sub-assemblies']::{occurences_in_root['sub-assemblies']}")
    # recording parts in root assembly that belong to sub assembly
    for subassembly in occurences_in_root['sub-assemblies']:
      for part in occurences_in_root['parts']:
        if subassembly in part :
          part_id = part[:]
          part_id.remove(subassembly)
          data = {
            "assembly":subassembly,
            "part_path":part,
            "part":part_id
          }
          occurences_in_root['sub_assembly_parts'].append(data)

    relations = []
    relations_that_belong_to_assembly = []
    # root features:

    # print(f"root['features']::len::{len(root['features'])}")

    for idx,feature in enumerate(root['features']):
        child = feature['featureData']['matedEntities'][0]['matedOccurrence']
        parent = feature['featureData']['matedEntities'][1]['matedOccurrence']
        assemblyInstanceId = None
        if len(child)>0:
            assemblyInstanceId = child[0]
        relation = {
          'child':child,
          'parent':parent,
          'feature':feature,
          'assemblyInfo':assembly_info,
          'assemblyInstanceId':assemblyInstanceId
        }
        # print(f"mjcf::parent::{parent}")
        # print(f"mjcf::child::{child}")

        relations.append(relation)
        # when two ids are in an array one belong to sub assembly
        # I think the first one represent the assembly
        # the second one represent the part
        child_is_part_of_subassembly = len(child)>1
        if child_is_part_of_subassembly:
        #   print(f"get_part_transforms_and_fetuses::feature::{feature}")
          for id in child:
            if id in occurences_in_root['sub-assemblies'].keys():
              root_part = child[:]
              root_part.remove(id)
              data = {
                'assemblyInstanceId':id,
                'assembly':occurences_in_root['sub-assemblies'][id],
                'relation':relation,
                'assembly_root_part':root_part,
                #This will be filled when going through subassembly features
                'replacement':None

              }
              relations_that_belong_to_assembly.append(data)

    # print(f"before::relations::{relations}")

    #get rest of the relations from sub-assemblies
    if len(relations_that_belong_to_assembly)>0:
      for idx,rbs in enumerate(relations_that_belong_to_assembly):
        subassembly_relations = []
        expected_element_id = rbs['assembly']['elementId']
        subassembly_root_part = rbs['assembly_root_part']
        expected_instance_id = rbs['assemblyInstanceId']
        subassembly = None
        for asm in assembly["subAssemblies"]:
          if expected_element_id == asm['elementId']:
            for feature in asm['features']:
              child = feature['featureData']['matedEntities'][0]['matedOccurrence']
              if len(child)>1:
                assemblyInstanceId = child[0]
              parent = feature['featureData']['matedEntities'][1]['matedOccurrence']
              subassembly_info = assembly_info.copy()
              subassembly_info['assemblyId']= asm['elementId']
              relation = {
                'child':child,
                'parent':parent,
                'feature':feature,
                'assemblyInfo':subassembly_info,
                'assemblyInstanceId':expected_instance_id
              }
              subassembly_relations.append(relation)
        relations_that_belong_to_assembly[idx]["replacement"] = subassembly_relations

    # replace relations in root with equivalent sub assemblies
    for rbs in relations_that_belong_to_assembly:
      original_relation = rbs['relation']
      replacement_relations = rbs ['replacement']
      insert_position = None
      for idx,r in enumerate(relations):
        if (r['child'] == original_relation['child'] and \
            r['parent'] == original_relation['parent']):
            #  record index of previous relation to be removed
            insert_position = idx
            break
      # insert new relations
      relations[insert_position+1:insert_position+1] = replacement_relations
      # correcting relation between assemblies
      # by removing assembly name form relation
      relations[insert_position]['child'] = relations[insert_position]['child'][1:]

    occurences_in_root["relations"] = relations
    # print(f"after::relations::len::{len(relations)}")
    # print(f"occurences_in_root['sub_assembly_parts']::len::{len(occurences_in_root['sub_assembly_parts'])}")
    # print(f"occurences_in_root['sub_assembly_parts']::{occurences_in_root['sub_assembly_parts']}")
    # print(f"occurences_in_root['parts']::len::{len(occurences_in_root['parts'])}")
    # print(f"relations_that_belong_to_assembly::len::{len(relations_that_belong_to_assembly)}")

    # for part in occurences_in_root['parts']:
    #   print(f"part::{part}")
    # for relation in relations:
    #   print("\n")
    #   print(f"relation::{relation}")
    #   print("\n")

    # for occ in assembly['rootAssembly']['occurrences']:
    #   print(f"occ::path::{occ['path']}")

    return occurences_in_root

counter = 0
def create_parts_tree(root_part, part_instance,
                      assemblyInstance,
                      occurences_in_root:dict,
                      assembly:dict,feature=None):
    global counter
    counter += 1
    print(f"create_parts_tree::{counter}")
    #children of base_part
    relations = get_part_relations(occurences_in_root['relations'],
                part_instance,assemblyInstance
                )

    print(f"relations::len::{len(relations)}")
    there_is_a_relation = len(relations)>0
    if there_is_a_relation:
        for relation in relations:

            feature = relation['feature']
            assemblyInfo = relation['assemblyInfo']
            assemblyInstanceId = relation['assemblyInstanceId']
            # print(f"create_parts_tree::assemblyInstanceId::{assemblyInstanceId}")
            child = relation['child'][0]
            path = []
            if assemblyInstanceId:
              path.append(assemblyInstanceId)
            path.append(child)
            # print(f"path::{path}")
            # this is where it is failing
            # it seems findInstance that is used inside find_occurence returns the first part that it finds
            # it does not return an instance of part specific to a specific assembly
            # occ = find_occurence(assembly["rootAssembly"]['occurrences'],
            #             child
            # )
            occ = find_occurrence(assembly["rootAssembly"]['occurrences'],path)
            # print("\n")
            # print(f"assemblyInfo::{assemblyInfo}")
            # print("\n")

            j = JointData2(
                name = feature['featureData']['name'],
                j_type = feature['featureData']['mateType'],
                z_axis = feature['featureData']['matedEntities'][0]['matedCS']['zAxis'],
                feature = feature,
                assemblyInfo = assemblyInfo
            )
            part = Part2(
                instance_id = child,
                transform = occ['transform'],
                joint = j,
                occurence = occ
            )

            create_parts_tree(part,child,assemblyInstanceId,occurences_in_root,assembly,relation['feature'])
            root_part.add_child(part)
    return

def create_mjcf_assembly_tree(assembly:dict):

    # print(f"config::{config}")
    state = OnshapeState()

    root = assembly["rootAssembly"]
    root = assembly["rootAssembly"]


    # print(f"assembly::keys::{assembly.keys()}")
    # print("\n")
    # print(f"root::{root.keys()}")
    # print("\n")
    # print(f"root::occurrences::0::{root['occurrences'][0]}")
    # print(f"root::occurrences::0::keys::{root['occurrences'][0].keys()}")
    # print(f"root::occurrences::0::keys::instance::{root['occurrences'][0]['instance']}")


    for idx,feature in enumerate(root['features']):
        print("\n")
        print(f"{idx}::feature::{feature}")
        print(f"{idx}::feature::featureData::matedEntities::{feature['featureData']['matedEntities']}")
        print("\n")


    # print(f"assembly::{assembly['rootAssembly']}")

    # print(f"root::assembly::elementID::{root['elementId']}")


    subassemblies = None
    ####### This code was  copy pasted from load_robot.py ########
    # Collecting occurrences, the path is the assembly / sub assembly chain
    occ_keys = []
    occ_dic = {
        "assembly":[],
        "part":[]
    }
    occurrences = {}
    for occurrence in root["occurrences"]:
        occurrence["assignation"] = None
        occurrence["instance"] = findInstance(occurrence["path"])
        occurrence["transform"] = np.matrix(np.reshape(occurrence["transform"], (4, 4)))
        occ_key = root['elementId']+"_"+"_".join(occurrence["path"])
        occurrences[occ_key] = occurrence
        occ_keys.append(occurrence["path"])

        print(f"mjcf::type::{occurrence['instance']['type']}::path::{occurrence['path']}")

    # print(f"occurrences::len::{len(occurrences)}")
    # for key,value in occurrences.items():
    #     print(f"oc::type::{value['instance']['type']}")
    #     print(f"oc::instance::{value['instance']}")
    #     print(f"oc::transform::{value['transform']}")


    if len(assembly["subAssemblies"])>0:
        subassemblies = assembly["subAssemblies"]
        print("\n")
        # print(f"subassemblies::{subassemblies}")
        print("\n")
        for subassembly in subassemblies:
            # print(f"subassembly::keys::{subassembly.keys()}")
            # getting occerance of subassembly
            instance = subassembly['instances'][0]
            # documentId = instance['documentId']
            # assemblyId = instance["elementId"]

            # print(f"instance::{instance}")
            # print(f"documentId::{documentId}")
            # print(f"assemblyId::{assemblyId}")

            document = client.get_document(subassembly["documentId"]).json()
            workspaceId = document["defaultWorkspace"]["id"]

            # # print(f"dic_to_assembly::document::{document}")

            assemblyId = subassembly["elementId"]

            print(f"subassembly::instance::id::{instance['id']}")
            assembly = client.get_assembly(
            subassembly["documentId"],
            workspaceId,
            assemblyId,
            )

            for idx,feature in enumerate(subassembly['features']):
                print("\n")
                print(f"{idx}::sub::feature::{feature}")
                print(f"{idx}::sub::feature::featureData::matedEntities::{feature['featureData']['matedEntities']}")
                print("\n")

            occurrences2 = {} # This is used for debugging
            for occurrence in assembly['rootAssembly']["occurrences"]:
                occurrence["assignation"] = None
                occurrence["instance"] = findInstance(occurrence["path"], subassembly['instances'])
                occurrence["transform"] = np.matrix(np.reshape(occurrence["transform"], (4, 4)))
                occ_key = assemblyId+"_"+"_".join(occurrence["path"])
                occurrences[occ_key] = occurrence
                occ_keys.append(occurrence["path"])
                print(f"mjcf::type::{occurrence['instance']['type']}::path::{occurrence['path']}")





            # print("\n\n")
            # print(f"occurrences::len::{len(occurrences2)}")
            # for key,value in occurrences2.items():
            #     print(f"oc::sub::type::{value['instance']['type']}")
            #     print(f"oc::sub::instance::{value['instance']}")
            #     print(f"oc::sub::transform::{value['transform']}")

    return
    ###### END ########
    # the e in e_ stand for Entity
    root_assembly = Assembly(
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
    state.add_assembly(root_assembly)
    dic_to_assembly(state,root,occurrences,subassemblies,root_assembly)

    # for part in root_assembly.parts:
    #     print(f"root_assembly::part::name::{part.name}")

    # for part in root_assembly.assemblies[0].parts:
        # print(f"root_assembly.assemblies[0]::part::name::{part.name}")

    # print(f"state::parts::keys::{state.parts.keys()}")
    # print(f"state::assemblies::keys::{state.assemblies.keys()}")

    return root_assembly,state,occurrences

def create_models_using_part_tree(assembly):
    occurences_in_root = get_part_transforms_and_fetuses(assembly)
    part_instance = occurences_in_root['robot_base']
    occ = find_occurence(assembly["rootAssembly"]['occurrences'],
                        part_instance
            )

    base_part = Part2(
        instance_id = part_instance,
        transform = occ['transform'],
        occurence = occ
    )
    create_parts_tree(base_part,part_instance,None,occurences_in_root,assembly)

    # print part tree
    pt = PrettyPrintTree(lambda x: x.children, lambda x: x.instance_id)
    pt(base_part)

    matrix = np.matrix(np.identity(4))
    # base pose
    body_pos = [0]*6
    mj_state = MujocoGraphState()
    root_node =  part_trees_to_node(base_part,matrix,body_pos,mj_state)

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


def create_models(assembly_tree:Assembly,onshape_state:OnshapeState,occurences):
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


    root_node = entity_node_to_node(entitiy_node,mj_state,matrix,b_pose,occurences)

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





















