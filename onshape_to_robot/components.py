from typing import List,Optional,Any
from dataclasses import dataclass,field
from uuid import uuid4

from enum import Enum


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
class Joint:
    name:str
    j_type:str
    j_range:List[float]
    axis:List[float]
    j_class:str = None


    def xml(self):
        name = f"name='{self.name}'" if self.name !=None else ""
        j_range = f"range='{to_str(self.j_range)}'" if self.j_range !=None else ""
        axis = f"axis='{to_str(self.axis)}'" if self.axis.any() else ""

        return f"<joint {name} {j_range} {axis}/>"

@dataclass
class Geom:
    name: str
    g_type: str = "mesh"
    color: Optional[List[float]] = None
    pos: List[float] = None
    euler: List[float] = None
    mesh: str = ""
    g_class: str = None

    def xml(self):
        return f"<geom type='mesh' contype='0' conaffinity='0' pos='{to_str(self.pos)}' euler='{to_str(self.euler)}' mesh='{self.mesh}' />"

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
    joints_defaults = List[dict]
    geoms_default = List[dict]
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

def refactor(self,tree:Body,graph_state:MujocoGraphState):
    """
    if an atterbute with same value
    for <tag attrbute=value/> is repeated in the tree a default is created
    when first instance of material or mesh is encountered an asset is created.
    """
    pass

