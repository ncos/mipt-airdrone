#!/usr/bin/python
# // Author: Anton Mitrokhin

import os


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LOCAL_BASH_FILE = ROOT_DIR + "/devel/setup.bash"
LAUNCHER_DIR = ROOT_DIR + "/contrib/launchers"
HOME_DIR = os.path.expanduser("~")


print ROOT_DIR, LAUNCHER_DIR

LAUNCHER_NAMES = ["airdrone_launch", 
                  "airdrone_real_launch",
                  "airdrone_test", 
                  "rebuild",
                  "rebuild_we"]


def add_to_file(f, contents, string):
    if string not in contents:
        f.write(string + '\n')
        print "ADDED: " + string
    else:
        print "OK: " + string

with open(HOME_DIR + "/.bashrc", 'r+') as fbashrc:
    contents = fbashrc.read()
    add_to_file(fbashrc, contents, "source /opt/ros/hydro/setup.bash")
    add_to_file(fbashrc, contents, "source " + LOCAL_BASH_FILE)
    add_to_file(fbashrc, contents, "export PATH=" + LAUNCHER_DIR + ":$PATH")



