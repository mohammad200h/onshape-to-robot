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
class Default:
    name:str
    # TODO: change the type of element from str to Enum: geom, joint
    element_type:str
    attrbutes:List[Tuple[str,Any]]
    elements:List[object]

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
    """
    <defult>
        <joint range="-0.1 0.1" type="hinge"/>
        <geom type="mesh"/>
    </defult>
    """
    super_defaults:List[Default]
    """
     <defult>
        <defult class="joint1">
            <joint range="-0.5 0.5"/>
        </defult>
        <defult class="joint2">
            <joint range="-0.7 0.2"/>
        </defult>
        <defult class="geom1">
            <geom type="box" size="1 1 1"/>
        </defult>
    </defult>
    """
    named_defaults:List[Default]
    root:Node
    state:object

    def refactor(self):
        # remove attrbutes covered by the supper default
        for s_default in self.super_defaults:
            if s_default.element_type =="joint":
                for t in s_default.attrbutes:
                    for elem in s_default.elements:
                        elem.set_attrib(
                            attrib=t[0],
                            value = None
                        )
            elif s_default.element_type =="geom":
                for t in s_default.attrbutes:
                    for elem in s_default.elements:
                        elem.set_attrib(
                            attrib=t[0],
                            value = None
                        )
        # remove attrbiutes convered by named default and
        # assign class to element
        for n_default in self.named_defaults:
            if n_default.element_type =="joint":
                    # print(f"refactor::n_default::joint::n_default.name::{n_default.name}")
                    for elem in n_default.elements:
                        # print(f"refactor::elem::before::{elem}")
                        elem.set_attrib(
                            attrib= n_default.attrbutes[0],
                            value = None
                        )
                        elem.set_attrib(
                                attrib="class",
                                value = n_default.name
                        )
                    # print(f"refactor::elem::after::{elem}")
            elif n_default.element_type =="geom":
                    for elem in n_default.elements:
                        elem.set_attrib(
                            attrib= n_default.attrbutes[0],
                            value = None
                        )
                        elem.set_attrib(
                                attrib="class",
                                value = n_default.name
                        )

    def xml(self):
        asset_xml = self.state.assets.xml()
        default_xml = self.default_xml()
        body_xml = self.root.xml()

        mj_xml = (
            "<mujoco model='robot'>"
            "<compiler angle='radian' autolimits='true'/>"
	        "<option integrator='implicitfast'/>"
            f"{asset_xml}"
            f"{default_xml}"
            "<worldbody>"
            f"{body_xml}"
            "</worldbody>"
            "</mujoco>"
        )

        return mj_xml

    def default_xml(self):
        # print(f"default_xml::called")
        # print("Tree::default_xml::called")
        #super default
        super_defaults = ""
        for s_default in self.super_defaults:
            if s_default.element_type =="joint":
                    xml = f"<joint {self.super_default_attributes_str(s_default.attrbutes)} />\n"
                    super_defaults +=xml
            elif s_default.element_type =="geom":
                    xml = f"<geom {self.super_default_attributes_str(s_default.attrbutes)} />\n"
                    super_defaults +=xml

        # namded defaults
        named_defaults=""
        for n_default in self.named_defaults:
            # print(f"default_xml::n_default::element_type::{n_default.element_type}")
            if n_default.element_type =="joint":
                    xml =(
                        f"<default class='{n_default.name}'>"
                        f"  <joint {self.named_defaults_attributes_str(n_default.attrbutes)} />\n"
                        "</default>"
                    )

                    named_defaults +=xml
            elif n_default.element_type =="geom":
                    xml =(
                        f"<default class='{n_default.name}'>"
                        f"  <geom  {self.named_defaults_attributes_str(n_default.attrbutes)} />\n"
                        "</default>"
                        )

                    named_defaults +=xml

        xml =(
            "<default>"
            f"  {super_defaults}"
            f"  {named_defaults}"

            "</default>"
        )
        return xml

    def super_default_attributes_str(self,attrbutes):
        #TODO : implemnet this
        attr = ""
        for t in attrbutes:
            value = to_str(t[1]) if type(t[1])==tuple else t[1]
            attr += f" {t[0]}='{value}'"
        return attr

    def named_defaults_attributes_str(self,attrbutes):
        attr = f""
        value = to_str(attrbutes[1]) if type(attrbutes[1])==tuple else attrbutes[1]
        attr += f"{attrbutes[0]}='{value}'"
        return attr


@dataclass
class Mesh:
    #TODO: maybe make it a path object
    file:str

@dataclass
class Material:
    name:str
    rgba: List[float]

@dataclass
class Assets:
    materials: List[Material] = field(default_factory=list)
    meshes:List[Mesh] = field(default_factory=list)

    def add_mesh(self,m):
        if  m not in self.meshes:
            self.meshes.append(m)

    def add_material(self,name,rgba):
        m = Material(name,rgba)
        if m not in self.materials:
            self.materials.append(m)

    def xml(self):
        meshes = ""
        material= ""
        for m in self.meshes:
            meshes += f" <mesh file='{m}'/>"
        for m in self.materials:
            material += f" <material name='{m.name}' rgba='{to_str(m.rgba)}'/>"

        asset = (
            "<asset>"
                f"{meshes}"
                f"{material}"
            "</asset>"
        )
        return asset

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
        axis = f"axis='{to_str(self.axis)}'" if self.axis !=None else ""
        j_class = f"class='{self.j_class}'" if self.j_class !=None else ""
        return f"<joint {j_class} {name} {j_range} {axis}/>"

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

    def set_attrib(self,attrib:str,value:Any):
        # print("Joint::set_attrib::called")
        # print(f"Joint::set_attrib::attrib::{attrib}")
        # print(f"Joint::set_attrib::value::{value}")

        if attrib == "name":
            self.name = value
        elif attrib == "type":
            self.j_type = value
        elif attrib == "range":
            self.j_range = value
        elif attrib == "axis":
            self.axis = value
        elif attrib == "class":
            self.j_class = value

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
        class_xml = f"class='{self.g_class}'" if self.g_class else ""
        material_xml = f"material='{self.material.name}'" if self.material else ""
        print(f"Geom::xml::self.g_class::{self.g_class}")

        return f"<geom {class_xml} type='mesh' pos='{to_str(self.pos)}' euler='{to_str(self.euler)}' mesh='{self.mesh}' {material_xml} />"

    def to_dict(self):
        return {
            self.id:{
                "name":self.name,
                "type":self.g_type,
                "material":self.material.name,
                "pos":self.pos,
                "euler":self.euler,
                "mesh":self.mesh,
                "class":self.g_class,
            }

        }

    def set_attrib(self,attrib:str,value:Any):
        if attrib == "name":
            self.name = value
        elif attrib == "type":
            self.g_type = value
        elif attrib == "material":
            print(f"Geom::set_attrib::material::value::{value}")
            self.material = value
        elif attrib == "pos":
            self.pos = value
        elif attrib == "euler":
            self.euler = value
        elif attrib == "mesh":
            self.mesh = value
        elif attrib == "class":
            print(f"Geom::set_attrib::class::value::{value}")
            self.g_class = value
            print(f"Geom::set_attrib::class::self.g_class::{self.g_class}")
            print(f"Geom::set_attrib::class::self.id::{self.id}")

@dataclass
class Inertia:
    pos:List[float]
    euler:Optional[List[float]]
    mass:float
    fullinertia:List[float]


    def xml(self):
        return f"<inertial pos='{to_str(self.pos)}' mass='{self.mass}' euler='{to_str(self.euler)}' fullinertia='{to_str(self.fullinertia)}' />"

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
        # TODO: Figure out what is wrong with axis of rotation
        joint_xml = self.prop.joint.xml() if self.prop.joint !=None else ""
        geom_xml =  self.prop.geom.xml()
        #TODO: figure out what is wrong with inertia
        # intertia_xml = self.prop.inertia.xml()
        children = ""
        for child in self.children:
            children += child.xml()

        # putting it all together
        xml += "<body>"
        xml += joint_xml
        # xml += intertia_xml
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
    defaults:dict = None
    elements:Dict[UUID,object] = field(default_factory=dict)
    ids:List[UUID] = field(default_factory=list)
    attirbute_groups:List[dict] = field(default_factory=list)

    def add(self,e,obj):
        id = list(e.keys())[0]
        self.ids.append(id)
        self.attirbute_groups.append(e)
        self.elements[id]=obj

    def get_element(self,id:UUID):
        """
        in : id
        out: pointer to node given i
        """
        return self.elements[id]

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
    assets = Assets()


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

    # print(f"unique_model_attribute_and_value_dict_count::after delete::{unique_model_attribute_and_value_dict_count}")

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

    attribiutes_common_in_all_elements_with_ids = (ids,attribiutes_common_in_all_elements)
    # print(f"attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    # print(f"attribiutes_common_in_all_elements::len::{len(attribiutes_common_in_all_elements)}")

    attirbutes_shared_among_some_of_elemets:Dict[str,(str,Any)] = {}

    classes = {}
    counter =1
    for key,count in unique_model_attribute_and_value_dict_count.items():
        if count == num_ids or count == 1:
            continue

        attribute,value = key
        # print(f"attribute,value ::{attribute},{value}")
        ids = unique_model_attribute_and_value_dict_id[key]
        classes["joint_"+str(counter)] = (ids,key)
        counter +=1

    # print(f"unique_model_attribute_and_value_dict_count::{unique_model_attribute_and_value_dict_count}")

    return attribiutes_common_in_all_elements_with_ids,classes

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
                # print(f"pair::{pair}")
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


    # print(f"unique_model_attribute_and_value_dict_count::after delete::{unique_model_attribute_and_value_dict_count}")

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

    attribiutes_common_in_all_elements_with_ids = (ids,attribiutes_common_in_all_elements)
    # print(f"attribiutes_common_in_all_elements::{attribiutes_common_in_all_elements}")
    # print(f"attribiutes_common_in_all_elements::len::{len(attribiutes_common_in_all_elements)}")

    attirbutes_shared_among_some_of_elemets:Dict[str,(str,Any)] = {}

    classes = {}
    counter =1
    for key,count in unique_model_attribute_and_value_dict_count.items():
        if count == num_ids or count == 1:
            continue

        attribute,value = key
        # print(f"attribute,value ::{attribute},{value}")
        ids = unique_model_attribute_and_value_dict_id[key]
        classes["geom_"+str(counter)] = (ids,key)
        counter +=1


    #remove classes with a single memeber
    # print(f"attribiutes_common_in_all_elements_with_ids::{attribiutes_common_in_all_elements_with_ids}")
    # print(f"classes::{classes}")

    return attribiutes_common_in_all_elements_with_ids,classes

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
  msybe builf an app on top of simulate enabling toggling between modeling and simulate
"""