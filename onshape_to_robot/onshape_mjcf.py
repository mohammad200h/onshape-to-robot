from dataclasses import dataclass,field
from typing import Dict, List, Tuple, Any,Optional
from enum import Enum

e_type =["part","subAssembly"]

class Onshape:
    pass

class EntityType(Enum):
    PART = "part"
    Assembly = "assembly"

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

@dataclass
class Assembly(Entity):
    name:str
    element_id:str
    document_id:str
    parts:List[Part] = field(default_factory=list)
    assemblies:List['Assembly'] = field(default_factory=list)
