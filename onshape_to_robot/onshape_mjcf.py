from dataclasses import dataclass,field
from typing import Dict, List, Tuple, Any,Optional
from enum import Enum
from .components import Node




#######
# These are simply data containers.
# Their data might need to be proccesed
#######


class EntityType(Enum):
    PART = "part"
    Assembly = "assembly"

@dataclass
class EntityNode:
    entity:Any
    e_type:EntityType
    parent:Optional['EntityNode'] = None
    children:List['EntityNode'] = field(default_factory=list)



    def add_child(self, child_node:'EntityNode'):
        """Adds a child node to the current node."""
        child_node.parent = self
        self.children.append(child_node)


@dataclass
class Entity:
    e_id:str
    e_type:EntityType


@dataclass
class Part(Entity):
    name:str
    element_id:str
    document_id:str
    documentMicroversion:str
    configuration:str
    instance:dict
    # dervied values
    occerance:dict
    mesh_info:dict
    inertia_info:dict
    transform:List[float]
    color:List[float]

@dataclass
class Assembly(Entity):
    name:str
    element_id:str
    document_id:str
    instance:dict
    features:dict
    # dervied values
    occerance:dict
    root_entity:Entity = None
    e_type = EntityType.Assembly
    parts:List[Part] = field(default_factory=list)
    parts_id:List[str] = field(default_factory=list)

    assemblies:List['Assembly'] = field(default_factory=list)
    assemblies_id:List[str] = field(default_factory=list)
    relations:List[str] = field(default_factory=list)

    def add_part(self,part):
        self.parts.append(part)
        self.parts_id.append(part.e_id)

    def add_assembly(self,assembly):
        self.assemblies.append(assembly)
        self.assemblies_id.append(assembly.e_id)


@dataclass
class OnshapeState:
    unique_element_ids: List[str] = field(default_factory=list)
    # element_id is unique but partid is not

    # key -> element id
    # value -> list of assemblies
    assemblies_element_assembly_id:Dict[str,List[str]] =  field(default_factory=dict)
    assemblies_id:List[str] = field(default_factory=list)
    assemblies:List[Assembly] = field(default_factory=list)
    # key -> element id
    # value -> list part_ids
    parts_element_part_id: Dict[str,List[str]] =  field(default_factory=dict)
    parts_id:List[str] = field(default_factory=list)
    parts: List[Part] = field(default_factory=list)
    # key -> element id
    # value -> Node
    # model_trees:Dict[str,Node]

    def add_assembly(self,assembly:Assembly):
        if assembly.element_id not in self.unique_element_ids:
            self.unique_element_ids.append(assembly.element_id)
            self.assemblies_element_assembly_id[assembly.element_id] = [assembly.e_id]
        else:
            self.assemblies_element_assembly_id[assembly.element_id].append(assembly.e_id)
        self.assemblies_id.append(assembly.e_id)
        self.assemblies.append(assembly)


    def add_part(self,part:Part):
        if part.element_id not in self.unique_element_ids:
            self.unique_element_ids.append(part.element_id)
            self.parts_element_part_id[part.element_id] = [part.e_id]
        else:
            self.parts_element_part_id[part.element_id].append(part.e_id)
        self.parts_id.append(part.e_id)
        self.parts.append(part)

    def get_part(self,part_id):
        for part in self.parts:
            if part.e_id == part_id:
                return part
        return None

    def get_assembly(self,assembly_id):
        for assembly in self.assemblies:
            if assembly.e_id == assembly_id:
                return assembly
        return None

    def get_entity(self,id):
        p = self.get_part(id)
        a = self.get_assembly(id)
        if a != None:
            return {"entity":a,"type":"assembly","id":id,"name":a.name}
        elif p != None:
            return {"entity":p,"type":"part","id":id,"name":p.name}

    def get_assembly_using_the_part(self,part_id):
        """
        we want to see which assembly is using this part
        given part_id we get assembly
        """
        for assembly in self.assemblies:
            for assembly_part_id in assembly.parts_id:
                if assembly_part_id == part_id:
                    return assembly
        return None
