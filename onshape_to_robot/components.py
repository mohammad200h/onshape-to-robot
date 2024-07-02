from typing import Dict, List, Tuple, Any,Optional
from dataclasses import dataclass,field
from uuid import uuid4,UUID

from enum import Enum

def all_ids_match(list1, list2):
    s1 = set([str(id) for id in list1])
    s2 = set([str(id) for id in list2])
    # print(f"all_ids_match::s1::{s1}")
    # print(f"all_ids_match::s2::{s2}")

    return s1==s2




class AngleType(Enum):
    RADIAN = "radian"
    DEGREE = "degree"

class TrackingMode(Enum):
    TrackCom = "trackcom"


def to_str(input:List[float])->str:
    return ' '.join(map(str, input))

@dataclass
class Node:
    prop:Any
    parent: Optional['Node'] = None
    children:List['Node'] = field(default_factory=list)

    def add_child(self, child_node:'Node'):
        """Adds a child node to the current node."""
        child_node.parent = self
        self.children.append(child_node)

@dataclass
class Tree:
    root:Node
    state:object



@dataclass
class Mesh:
    #TODO: maybe make it a path object
    file:str

@dataclass
class Material:
    # 'm' stands for material this is to not cofilict with keyword class
    m_class: str
    name:str
    rgba: List[float]


@dataclass
class Assets:
    materials: List[Material]
    meshes:List[Mesh]


@dataclass
class Joint:
    name:str
    j_type:str
    id:UUID
    j_range:List[float]
    axis:List[float]
    j_class:str = None



    def xml(self):
        name = f"name='{self.name}'" if self.name !=None else ""
        j_range = f"range='{to_str(self.j_range)}'" if self.j_range !=None else ""
        axis = f"axis='{to_str(self.axis)}'" if self.axis.any() else ""

        return f"<joint {name} {j_range} {axis}/>"
    def to_dict(self):
        return{
            self.id:{
                "name":self.name,
                "type":self.j_type,
                "range":self.j_range,
                "axis":self.axis,
                "class":self.j_class,
            }
        }

@dataclass
class Geom:
    name: str
    mesh: str
    material:Material = None
    g_type: str = "mesh"
    pos: List[float] = None
    euler: List[float] = None
    g_class: str = None
    id:UUID = uuid4()



    def xml(self):
        return f"<geom type='mesh' contype='0' conaffinity='0' pos='{to_str(self.pos)}' euler='{to_str(self.euler)}' mesh='{self.mesh}' />"
    def to_dict(self):
        return {
            self.id:{
                "name":self.name,
                "type":self.g_type,
                "material":self.material,
                "pos":self.pos,
                "euler":self.euler,
                "mesh":self.mesh,
                "class":self.g_class,
            }

        }
@dataclass
class Inertia:
    pos:List[float]
    euler:Optional[List[float]]
    mass:float
    diaginertia:List[float]


    def xml(self):
        return f"<inertial pos='{to_str(self.pos)}' mass='{self.mass}' euler='{to_str(self.euler)}' diaginertia='{to_str(self.diaginertia)}' />"

@dataclass
class Site:
    size:List[float]
    rgba:List[float]
    pos:List[float]
    name:str
    group:str


@dataclass
class BodyElements:
    inertia:Inertia
    geom:Geom
    joint:Optional[Joint] = None
    site:Optional[Site] = None


class Body(Node):
    def __init__(self,prop:BodyElements):
        super().__init__(prop)

    def xml(self):
        xml = ""
        joint_xml = self.prop.joint.xml() if self.prop.joint !=None else ""
        geom_xml =  self.prop.geom.xml()
        intertia_xml = self.prop.inertia.xml()
        children = ""
        for child in self.children:
            children += child.xml()

        # putting it all together
        xml += "<body>"
        xml += joint_xml
        xml += intertia_xml
        xml += geom_xml
        xml += children
        xml += "</body>"

        return xml



@dataclass
class Comiler:
    angle:AngleType
    meshdir:str = "./"
    autolimits:bool = True

@dataclass
class Option:
    integrator:str = "implicitfast"

# @dataclass
# class Default:

@dataclass
class Camera:
    name:str
    pos:List[float]
    xyaxes:List[float]
    fovy:List[float]
    mode:TrackingMode

class Light:
    name:str
    pos:List[float]
    mode:TrackingMode
    target:str
    diffuse:List[float]
    specular:List[float]
    cutoff:float

@dataclass
class WorldBody:
    Light :List[Light]
    camera:List[Camera]
    bodies:List[Body]
    sites:List[Site]

@dataclass
class ContExclude:
    body1:str
    body2:str

@dataclass
class Contacts:
    excludes:List[ContExclude]

@dataclass
class GeneralActuator:
    name:str
    joint:str
    # 'a' stands for material this is to not cofilict with keyword class
    a_class:str



@dataclass
class Actuator:
    general_acts:List[GeneralActuator]


@dataclass
class ElementState:
    defaults:dict=None
    ids:List[UUID] = field(default_factory=list)
    attirbute_groups:List[dict] = field(default_factory=list)

    def add(self,e):
        id = list(e.keys())[0]
        self.ids.append(id)
        self.attirbute_groups.append(e)


@dataclass
class MujocoGraphState:
    """
    stores state of the graph:
    This is going to be used for
    1.  default creation:
        If a thing happens more than once then it should be a default class
    2. assets
       - material should be created on first encounter and then reused
       - same is true for mesh

    """
    # used for defaults managment
    joint_state = ElementState()
    geom_state  = ElementState()
    # used for asset managment
    meshes = List[Mesh]
    materials = List[Material]



"""
  Thought : I yet dont how to set actuator,Light,Camera,Contact Exclude values from Onshape
  Possible solutions:
  https://forum.onshape.com/discussion/5027/ui-feature
  https://forum.onshape.com/discussion/514/how-to-create-plugins-for-onshape
  https://cad.onshape.com/FsDoc/uispec.html
  Onshape app stor: https://www.youtube.com/watch?v=Uvs6HAdb4eA
  https://forum.onshape.com/discussion/20554/information-on-creating-an-onshape-app
  How to create an app : https://forum.onshape.com/discussion/5785/question-tuto-how-to-create-an-app
  https://cad.onshape.com/help/Content/app_store_faqs_And.html
"""

def refactor_joint(tree:Body,graph_state:MujocoGraphState):
    """
    if an atterbute with same value
    for <tag attrbute=value/> is repeated in the tree a default is created
    when first instance of material or mesh is encountered an asset is created.
    """
    # all the attrbute present in all element type
    model_attrbutes = set()
    common_attributes_and_values = []
    ids = []

    # getting elements attributes
    for ag in graph_state.joint_state.attirbute_groups:
        id = list(ag.keys())[0]
        model_attrbutes.update(ag[id].keys())

    # remove id,class,name
    model_attrbutes.discard("id")
    model_attrbutes.discard("class")
    model_attrbutes.discard("name")

    # print(f"refactor::{model_attrbutes}")
    combinations:Dict[UUID,List[Tuple[str,Any]]] = {}

    unique_model_attribute_and_value = set()
    for ag in graph_state.joint_state.attirbute_groups:
        id = list(ag.keys())[0]
        ids.append(id)
        attrib_dic = ag[id]
        for key,value in attrib_dic.items():
            if key in model_attrbutes:
                pair = (key,value)
                # print(f"pair::{pair}")
                unique_model_attribute_and_value.add(pair)
    # print(f"unique_model_attribute_and_value::{unique_model_attribute_and_value}")
    # print(f"ids::{ids}")
    unique_model_attribute_and_value_dict_id:Dict[Tuple[str,Any],List[UUID]] ={}

    # add ids with the unique attribute_and_value to dictonary
    for attribute_and_value in unique_model_attribute_and_value:
        # print(f"attribute_and_value::{attribute_and_value}")
        unique_model_attribute_and_value_dict_id[attribute_and_value]=[]
        for ag in graph_state.joint_state.attirbute_groups:
            id = list(ag.keys())[0]
            # print(f"id::{id}")
            attrib_dic = ag[id]
            # print(f"attrib_dic::{attrib_dic}")
            for key,value in attrib_dic.items():
                pair = (key,value)
                # print(f"pair::{pair}")
                # print(f"pair == attribute_and_value::{pair == attribute_and_value}")

                if pair == attribute_and_value:
                    unique_model_attribute_and_value_dict_id[attribute_and_value].append(id)


    # print(f"unique_model_attribute_and_value_dict_id::{unique_model_attribute_and_value_dict_id}")

    unique_model_attribute_and_value_dict_count:Dict[Tuple[str,Any],int] = {}
    keys_with_frequency_of_one = []
    for key,value in unique_model_attribute_and_value_dict_id.items():
        unique_model_attribute_and_value_dict_count[key]=len(value)
        #remove (attribute,value) with count of one from unique_model_attribute_and_value_dict_id
        if len(value)==1:
            keys_with_frequency_of_one.append(key)
    # print(f"unique_model_attribute_and_value_dict_count::{unique_model_attribute_and_value_dict_count}")
    for key in keys_with_frequency_of_one:
        del unique_model_attribute_and_value_dict_id[key]

    # print("\n")
    # print(f"unique_model_attribute_and_value_dict_id::{unique_model_attribute_and_value_dict_id}")

    attribiutes_common_in_all_elements =[]


    # getting attrbuites shared among all elements
    num_ids = len(ids)
    for key,value in unique_model_attribute_and_value_dict_id.items():
        #there is a possible match
        # print(f"num_ids::{num_ids}")
        # print(f"value::len::{len(value)}")
        # print(f"key::{key}")
        # print(f"value::{value}")


        if unique_model_attribute_and_value_dict_count[key]==num_ids:
            #check all ids are in the
            if all_ids_match(ids,value):
                attribiutes_common_in_all_elements.append(key)

    # print(f"attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    # print(f"attribiutes_common_in_all_elements::len::{len(attribiutes_common_in_all_elements)}")

    attirbutes_shared_among_some_of_elemets:Dict[str,(str,Any)] = {}

    classes = {}
    counter =1
    for key,count in unique_model_attribute_and_value_dict_count.items():
        if count == num_ids:
            continue

        attribute,value = key
        # print(f"attribute,value ::{attribute},{value}")
        classes["joint_"+str(counter)] = key
        counter +=1

    return attribiutes_common_in_all_elements,classes

def refactor_geom(tree:Body,graph_state:MujocoGraphState):
    """
    if an atterbute with same value
    for <tag attrbute=value/> is repeated in the tree a default is created
    when first instance of material or mesh is encountered an asset is created.
    """
    # all the attrbute present in all element type
    model_attrbutes = set()
    common_attributes_and_values = []
    ids = []

    # getting elements attributes
    for ag in graph_state.geom_state.attirbute_groups:
        id = list(ag.keys())[0]
        model_attrbutes.update(ag[id].keys())

    # remove id,class,name
    model_attrbutes.discard("id")
    model_attrbutes.discard("class")
    model_attrbutes.discard("name")

    # print(f"refactor::{model_attrbutes}")
    combinations:Dict[UUID,List[Tuple[str,Any]]] = {}

    unique_model_attribute_and_value = set()
    for ag in graph_state.geom_state.attirbute_groups:
        id = list(ag.keys())[0]
        ids.append(id)
        attrib_dic = ag[id]
        for key,value in attrib_dic.items():
            if key in model_attrbutes:
                pair = (key,value)
                print(f"pair::{pair}")
                unique_model_attribute_and_value.add(pair)
    # print(f"unique_model_attribute_and_value::{unique_model_attribute_and_value}")
    # print(f"ids::{ids}")
    unique_model_attribute_and_value_dict_id:Dict[Tuple[str,Any],List[UUID]] ={}

    # add ids with the unique attribute_and_value to dictonary
    for attribute_and_value in unique_model_attribute_and_value:
        # print(f"attribute_and_value::{attribute_and_value}")
        unique_model_attribute_and_value_dict_id[attribute_and_value]=[]
        for ag in graph_state.geom_state.attirbute_groups:
            id = list(ag.keys())[0]
            # print(f"id::{id}")
            attrib_dic = ag[id]
            # print(f"attrib_dic::{attrib_dic}")
            for key,value in attrib_dic.items():
                pair = (key,value)
                # print(f"pair::{pair}")
                # print(f"pair == attribute_and_value::{pair == attribute_and_value}")

                if pair == attribute_and_value:
                    unique_model_attribute_and_value_dict_id[attribute_and_value].append(id)


    # print(f"unique_model_attribute_and_value_dict_id::{unique_model_attribute_and_value_dict_id}")

    unique_model_attribute_and_value_dict_count:Dict[Tuple[str,Any],int] = {}
    keys_with_frequency_of_one = []
    for key,value in unique_model_attribute_and_value_dict_id.items():
        unique_model_attribute_and_value_dict_count[key]=len(value)
        #remove (attribute,value) with count of one from unique_model_attribute_and_value_dict_id
        if len(value)==1:
            keys_with_frequency_of_one.append(key)
    # print(f"unique_model_attribute_and_value_dict_count::{unique_model_attribute_and_value_dict_count}")
    for key in keys_with_frequency_of_one:
        del unique_model_attribute_and_value_dict_id[key]

    # print("\n")
    # print(f"unique_model_attribute_and_value_dict_id::{unique_model_attribute_and_value_dict_id}")

    attribiutes_common_in_all_elements =[]


    # getting attrbuites shared among all elements
    num_ids = len(ids)
    for key,value in unique_model_attribute_and_value_dict_id.items():
        #there is a possible match
        # print(f"num_ids::{num_ids}")
        # print(f"value::len::{len(value)}")
        # print(f"key::{key}")
        # print(f"value::{value}")


        if unique_model_attribute_and_value_dict_count[key]==num_ids:
            #check all ids are in the
            if all_ids_match(ids,value):
                attribiutes_common_in_all_elements.append(key)

    # print(f"attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    # print(f"attribiutes_common_in_all_elements::len::{len(attribiutes_common_in_all_elements)}")

    attirbutes_shared_among_some_of_elemets:Dict[str,(str,Any)] = {}

    classes = {}
    counter =1
    for key,count in unique_model_attribute_and_value_dict_count.items():
        if count == num_ids:
            continue

        attribute,value = key
        # print(f"attribute,value ::{attribute},{value}")
        classes["geom_"+str(counter)] = key
        counter +=1


    #remove classes with a single memeber

    return attribiutes_common_in_all_elements,classes