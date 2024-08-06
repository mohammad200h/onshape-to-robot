import numpy as np
from copy import copy
import commentjson as json
from colorama import Fore, Back, Style
import sys
from sys import exit
import os
import hashlib
from . import csg
from .robot_description import RobotURDF, RobotSDF

from .mjcf import create_mjcf,create_mjcf_assembly_tree,create_models


partNames = {}

def main():
    # Loading configuration, collecting occurrences and building robot tree
    from .load_robot import \
        config, client, tree, occurrences, getOccurrence, frames,assembly

    # print(f"assembly::{assembly}")

    assembly_tree,onshape_state = create_mjcf_assembly_tree(assembly)

    create_models(assembly_tree,onshape_state)
    # create_mjcf(tree)

if __name__ == "__main__":
    main()