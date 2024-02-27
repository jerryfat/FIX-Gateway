from __future__ import print_function

import sys, os

import subprocess # non-blocking
import requests # for pip dependencies info and msg text
# demo imports
import argparse # single package deps
import pyavtools  # twp packages deps

from platform import python_version

######################################################################################    

print("########## START ########## START ########## START ########## START ########## START ##########")
# Parse connection argument
parser = argparse.ArgumentParser()
filename = ""
package  = ""
#parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-f", "--filename", help="filename.py filename to find import(s) pip .whl and formatted [components] for pyqtdeploy3.3.0")
parser.add_argument("-p", "--package", help="package name to find import(s) pip .whl and formatted [components] for pyqtdeploy3.3.0")
args = parser.parse_args() # command line parameters

if args.filename: # input from sitl qgcs or usb serial port to autopilot
    filename = args.filename
    print("filename = ", filename )

if args.package: # input from sitl qgcs or usb serial port to autopilot
    package = args.package
    print("package = ", package )

print("python version:",python_version())

print("mkdir dist-site-wheels/ for module wheels dir.. && cd wheels/")

#  cwd =current working dir from import os 
cwd = os.getcwd() 
# use thuis apps imports

# create wheels dir to store .whl file downloaded using pip download
command = "mkdir "+cwd+"/wheels" #  && cd dist-site-wheels
subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)



print("$$$$$$$$ begin downloading wheels .whl for imported dist-site module(s): $$$$$$$$" ) 

import ast
allmodules = set()
if(len(package) >= 1):
    print("using commandline -p package: ",package ," module")
    allmodules.add(package)
    for module in allmodules:
        print(module)
elif(len(filename) >=3):
    print("using commandline -f create dict {module:module-wheel.whl}  using command line filename.py -f imported modules and find filename.whl from pip download")
    # get all import statements from filename.py
    def visit_Import(node):
        for name in node.names:
            allmodules.add(name.name.split(".")[0])

    def visit_ImportFrom(node):
        # if node.module is missing it's a "from . import ..." statement
        # if level > 0 it's a "from .submodule import ..." statement
        if node.module is not None and node.level == 0:
            allmodules.add(node.module.split(".")[0])

    node_iter = ast.NodeVisitor()
    node_iter.visit_Import = visit_Import
    node_iter.visit_ImportFrom = visit_ImportFrom
    #
    with open(filename) as f:
        node_iter.visit(ast.parse(f.read()))
        print(allmodules)
        print("------end-------------")
    for module in allmodules:
        print(module)
else:
    print("no commandline args, using this list-modules-imported.py imported modules")
    modulenames = set(sys.modules) & set(globals())
    ALLmodules = [sys.modules[name] for name in modulenames]
    for i in ALLmodules: 
        # if not a ''built in'' pkg  in python, then download pkg module from pip each wheel as .whl name for each name in modulenames
        name = str(i).split("'")
        allmodules.add(name[1])
    for module in allmodules:
        print(module)

print("allmodules:",allmodules)
#exit()

for i in allmodules: 
    # if not a ''built in'' pkg  in python, then download pkg module from pip each wheel as .whl name for each name in modulenames
    name = "\'"+ i + "\'"
    #print("i:",name) 
    #component = "[" + i + "]"  #str(i).split("'")
    #
    ModuleWheels = {} # dict key:val  module:wheel.whl
    VerifyAllWheelsList = []
    #
    if ( " (built-in)>" not in name ) or  ("ERROR" not in name ): # skip downloading and adding module,  if  it is ''built-in'' package
        component = "[" + i + "]" 
        # get wheel  file(s)from pypi
        command = "python -m pip download --only-binary :all: --dest "+cwd+"/wheels"+" --no-cache "+i
        # proc = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        proc = subprocess.Popen([ 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        output = str(proc.stdout.read()).replace("\\n" , "\n\b") #.split("\n")
        output = str(output).replace("b'" , "") 
        output = str(output).replace("\'" , "") 
        if("ERROR" in str(output) ): # no module wheel names to get or get module
            print("###### ", component, " module IGNORE, no wheel needed from pip download ######" ) 
        else:
            print("###### ", component, " module ###### \n##### STARTING - getting module .whl(s) for pyqtdeploy" )
        print("pip download stdout:\n",str(output) ) #("\n")) # print("stdout: ",output.splitlines() ) #("\n"))           
        wheels = output.split("\n")  
        #
        for whl in wheels: # check each line in wheels returned back from pip dowload command in subproccess
            if ("Collecting" in whl):  # if string found in line from pip download stdout
                if("ERROR" not in str(output) ): # if no ERROR in returned stdout from pip download
                    #component = "[" + i + "]"  #str(i).split("'")
                    pypi_url = 'https://pypi.python.org/pypi/' + component + '/json'
                    dependencies = requests.get(pypi_url).json()
            if (("Downloading" in whl) or ("File was already downloaded" in whl)): 
                if("ERROR" not in str(output) ): # if no ERROR in returned stdout from pip download
                    if ("Downloading" in whl) and (".metadata" not in whl): 
                        wheel = whl.replace( ("Downloading "+cwd+"/wheels/") , "")
                        wheel = wheel.replace("\x08", "")
                        print('#### wheel = ', wheel) 
                        wheel = wheel.lower()
                        moduleStr = wheel.split("-")
                        #print ("moduleStr: ", moduleStr)
                        #print ("moduleStr[0].split(" ")", moduleStr[0].split(" ") )
                        module = moduleStr[0].split(" ")[1] 
                        #print(moduleStr, module )
                        ModuleWheels[ module ] = wheel.replace("\x08  ","")
                    if ("File was already downloaded " in whl):
                        wheel = whl.replace( ("File was already downloaded "+cwd+"/wheels/"), "")
                        wheel = wheel.replace("\x08", "")
                        print('#### wheel = ', wheel) 
                        wheel = wheel.lower()
                        moduleStr = wheel.split("-")
                        #print ("moduleStr: ", moduleStr)
                        #print ("moduleStr[0].split(" ")", moduleStr[0].split(" ") )
                        module = moduleStr[0].split(" ")[2] 
                        #print(moduleStr, module )
                        ModuleWheels[ module ] = wheel.replace("\x08  ","").lower()

            if ("Saved" in whl):                   
                if("ERROR" not in str(output) ): # if no ERROR in returned stdout from pip download
                    print("#### 'Saved' txt in stdout ",component)
                    if ("Saved " in whl): 
                        wheel = whl.replace( ("\x08Saved ./wheels/") , "")
                        wheel = wheel.replace("\x08","")
                        print('wheel = ', wheel) 
                        wheel = wheel.lower()
                        moduleStr = wheel.split("-")
                        #print ("moduleStr: ", moduleStr)
                        #print ("moduleStr[0].split(" ")", moduleStr[0].split(" ") )
                        module = moduleStr[0].split(" ")[0] 
                        #print(moduleStr, module )
                        ModuleWheels[ module ] = wheel.replace("\x08  ","")
                    print('wheel = '        + whl.replace("Saved ./wheels/" , "")  )

            if ("Successfully downloaded" in whl):                 
                if("ERROR" not in str(output) ): # if no ERROR in returned stdout from pip download
                    print("#### 'Successfully downloaded' txt in stdout ",component)  # printout sysroot.toml b.decode('utf-8')
                    print("###### FINISHED getting wheels for: ", whl.replace("\x08Successfully downloaded " , "")  )
                    VerifyAllWheelsList = whl.replace("\x08Successfully downloaded " , "").split(" ")
                    print("VerifyAllWheels()", VerifyAllWheelsList, "len(ModuleWheels):", len(ModuleWheels) ," len(VerifyAllWheelsList)", len(VerifyAllWheelsList))
                    print("ModuleWheels()", ModuleWheels, "len(ModuleWheels):", len(ModuleWheels) ," len(VerifyAllWheelsList)", len(VerifyAllWheelsList))
                    for module in VerifyAllWheelsList:
                        for component in ModuleWheels:
                            if (component in module.replace("-","_")) or (component.replace("_","-") in module):
                                print("VERIFY component:", component , " ==  module:", module, "  pip wheel:", ModuleWheels[ component ])
                            #else:
                                #print("ERROR VERIFY component:", component , " ==  module:", module, "  pip wheel:", ModuleWheels[ component ])
                    if (len(ModuleWheels) == len(VerifyAllWheelsList)): 
                        #print("VERIFY ERROR , mismatch in Downloaded components vs Saved wheels")
                        #else:
                        print("VERIFY SUCCESS len(ModuleWheels):", len(ModuleWheels) ," len(VerifyAllWheelsList)", len(VerifyAllWheelsList), "\n", ModuleWheels,"\n", VerifyAllWheelsList)
    else:
        print("###### skipped ", i, " ###### --> IGNORING module "+component + " is a (built-in) ...")


plugin = "\"wheel\""
dependencies = "[]"  #"[" + "Python:importlib.resources, Python:os" + "]"
exclusions = "[" + "__main__.py" + "]" 

print("\n#### append to sysroot.toml: ####\n")
for component in ModuleWheels:
    whl = ModuleWheels[component] # print dict ke:value seems to add a space when printing, this does not
    print("["+component.lower()+"]")
    print('plugin = '       + plugin)
    print('wheel =', whl) #"\"", str(ModuleWheels[component]).replace(" ","\"") , "\"") #space at front of wheel ?
    print('dependencies = ' + dependencies )
    print('exclusions = '   + exclusions  )
    print("")

print("########## END ########## END ########## END ########## END ########## END ##########")

'''
import os, time, rlcompleter, readline, lxml, yaml

import logging
import logging.config
import argparse
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import yaml
import importlib
#----------------
##from __future__ import print_function
import sys
sys.path.append("/usr/local/python/cv2/python-3.6/")
sys.path.append("../")

#from PyQt5 import QtWidgets as qtw
#from PyQt5 import QtGui as qtg
#from PyQt5 import QtCore as qtc
#
from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QMenu, QMdiArea, QMdiSubWindow, QMenuBar, QAction, qApp,  QStatusBar, QMessageBox, QAbstractItemView
from PyQt5.QtGui import QPalette, QColor
#
from PyQt5 import QtWidgets # for mdi demo

# DRONEKIT
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command   
# EOF on TCP socket forever loop
# exit button pressed


from pymavlink import mavutil # Needed for command message definitions

# Import ardupilotmega module for MAVLink 1
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

# Import common module for MAVLink 2
from pymavlink.dialects.v20 import common as mavlink2

import time
from datetime import datetime, timedelta
#
#import tkinter as tk # python3 gui widgets# Import mavutil
#from tkinter import ttk
#from tkinter import scrolledtext
# xml
import xml.dom.minidom
import xml.etree.ElementTree as ET

# gui
#from tkinter import *
#import tkMessageBox
#import tkinter
#from tkinter import messagebox
#from tkinter import filedialog  # for file open close dialog box
#from tkinter import Menu


# start and use SITL internally
import dronekit_sitl
#
import json # to convert strings to python dictionaries

import socket  #for tcp/udp
#
import os
#
import struct  # forpacking dict into binary to send dict over socket
import pickle  # for sending over tcp

import cherrypy
from jinja2 import Environment, FileSystemLoader
# from __future__ import print_function
import os
import simplejson
import time



# Set up option parsing to get connection string --connect "connection_string" on cli
import argparse
'''


'''
std output from pip download
 ######  argparse  ######
stdout:  b'Collecting argparse\n  Downloading argparse-1.4.0-py2.py3-none-any.whl (23 kB)\nSaved ./wheels/argparse-1.4.0-py2.py3-none-any.whl\nSuccessfully downloaded argparse\n'
wheels: ["b'Collecting argparse", '  Downloading argparse-1.4.0-py2.py3-none-any.whl (23 kB)', 'Saved ./wheels/argparse-1.4.0-py2.py3-none-any.whl', 'Successfully downloaded argparse', "'"]
#### append to sysroot.toml: ####
[argparse]
wheel = b'Collecting argparse
  Downloading argparse-1.4.0-py2.py3-none-any.whl (23 kB)
Saved ./wheels/argparse-1.4.0-py2.py3-none-any.whl
Successfully downloaded argparse
'
plugin = wheel
dependencies = [Python:importlib.resources, Python:os]
exclusions = [__main__.py]
'''

'''

String literals are described by the following lexical definitions:

https://docs.python.org/3.3/reference/lexical_analysis.html#string-and-bytes-literals

stringliteral   ::=  [stringprefix](shortstring | longstring)
stringprefix    ::=  "r" | "u" | "R" | "U"
shortstring     ::=  "'" shortstringitem* "'" | '"' shortstringitem* '"'
longstring      ::=  "'''" longstringitem* "'''" | '"""' longstringitem* '"""'
shortstringitem ::=  shortstringchar | stringescapeseq
longstringitem  ::=  longstringchar | stringescapeseq
shortstringchar ::=  <any source character except "\" or newline or the quote>
longstringchar  ::=  <any source character except "\">
stringescapeseq ::=  "\" <any source character>

bytesliteral   ::=  bytesprefix(shortbytes | longbytes)
bytesprefix    ::=  "b" | "B" | "br" | "Br" | "bR" | "BR" | "rb" | "rB" | "Rb" | "RB"
shortbytes     ::=  "'" shortbytesitem* "'" | '"' shortbytesitem* '"'
longbytes      ::=  "'''" longbytesitem* "'''" | '"""' longbytesitem* '"""'
shortbytesitem ::=  shortbyteschar | bytesescapeseq
longbytesitem  ::=  longbyteschar | bytesescapeseq
shortbyteschar ::=  <any ASCII character except "\" or newline or the quote>
longbyteschar  ::=  <any ASCII character except "\">
bytesescapeseq ::=  "\" <any ASCII character>



download wheel for package 

example add lxml package
(pyqt-crom-venv) jf@jf:~/Documents/pyqt-crom/utils$ cd ~/Documents/pyqt-crom/examples/mavgcs-proj

(pyqt-crom-venv) jf@jf:~/Documents/pyqt-crom/examples/mavgcs-proj$ python -m pip download --only-binary :all: --dest . --no-cache lxml
Collecting lxml
  Downloading lxml-5.1.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl.metadata (3.5 kB)
Downloading lxml-5.1.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (8.0 MB)
   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 8.0/8.0 MB 3.0 MB/s eta 0:00:00
Saved ./lxml-5.1.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
Successfully downloaded lxml

in sysroot.toml
[lxml]
plugin = "wheel"
wheel = "lxml-5.1.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
dependencies = ["Python:importlib.resources", "Python:os"]
exclusions = ["__main__.py"]


https://www.riverbankcomputing.com/static/Docs/pyqtdeploy/sysroot.html#ref-component-spec-file
from example:
[certifi]
plugin = "wheel"
wheel = "certifi-2021.10.8-py2.py3-none-any.whl"
dependencies = ["Python:importlib.resources", "Python:os"]
exclusions = ["__main__.py"]




try:
    # 👇️ using Python 3.10+
    from collections.abc import MutableMapping
except ImportError:
    # 👇️ using Python 3.10-
    from collections     import MutableMapping

# 👇️ <class 'collections.abc.MutableMapping'>
#print(MutableMapping)
'''
