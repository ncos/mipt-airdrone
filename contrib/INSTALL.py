#!/usr/bin/python
# Author: Anton Mitrokhin, MIPT 2014

import sys, os, subprocess

ROS_INSTALL_DIR = "/opt/ros/hydro"
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LOCAL_BASH_FILE = ROOT_DIR + "/devel/setup.bash"
LAUNCHER_DIR = ROOT_DIR + "/contrib/launchers_gen"
HOME_DIR = os.path.expanduser("~")

print "\nINSTALL SCRIPT FOR MIPT-AIRDRONE PROJECT"
print "author: Anton Mitrokhin, 2014"
print "Warning! This script assumes you had installed ROS Hydro to the", ROS_INSTALL_DIR
print ""
print "Workspace root directory:", ROOT_DIR
print "Launcher bash scripts directory:", LAUNCHER_DIR
print ""


def ensure_dir(f):
    if not os.path.exists(f):
        print "Created directory", f
        os.makedirs(f)

def add_to_file(f, contents, string):
    if string not in contents:
        f.write(string + ' # Generated by AIRDRONE-INSTALL-PY\n')
        print "ADDED: " + string
    else:
        print "OK: " + string

def cleanup_bashrc():
    output = []
    cnt = 0
    with open(HOME_DIR + "/.bashrc", 'r') as fbashrc:
        for line in fbashrc:
            cnt = cnt + 1
            if not 'Generated by AIRDRONE-INSTALL-PY' in line:
                cnt = cnt - 1
                output.append(line)

    with open(HOME_DIR + "/.bashrc", 'w') as fbashrc:
        fbashrc.writelines(output)

    print "Removed", cnt, "lines"


def gen_ad_rebuild_bash(file_path):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'cd ' + ROOT_DIR + '\n\n'\
    'catkin_make\n\n'\
    'source ' + LOCAL_BASH_FILE + '\n'\
    'echo "Press any key to continue..."\nread'

    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)


def gen_ad_rebuild_eclipse_bash(file_path):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'cd ' + ROOT_DIR + '\n\n'\
    'catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug'\
    ' -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8\n\n'\
    'source ' + LOCAL_BASH_FILE + '\n'\
    'echo "Press any key to continue..."\nread'

    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)


def gen_ad_airdrone_test_bash(file_path):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'source ' + LOCAL_BASH_FILE + '\n\n'\
    'roslaunch airdrone_launch airdrone_simulator.launch\n\n'\
    'sleep 1'

    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)


def gen_ad_airdrone_launch_bash(file_path):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'source ' + LOCAL_BASH_FILE + '\n\n'\
    'roslaunch airdrone_launch airdrone.launch\n\n'\
    'sleep 0.5'

    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)


def gen_ad_airdrone_real_launch_bash(file_path):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'source ' + LOCAL_BASH_FILE + '\n\n'\
    'roslaunch airdrone_launch airdrone_real.launch\n\n'\
    'sleep 0.5'

    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)




def add_launcher(name, icon, command):
    # The generation is done analogicaly to alacarte utilite
    file_path = HOME_DIR + '/.local/share/applications/' + name + '_ad_gen.desktop'
    format_str = '#!/usr/bin/env xdg-open\n\n'\
                 '[Desktop Entry]\n'\
                 'Version=1.0\n'\
                 'Type=Application\n'\
                 'Terminal=true\n'\
                 'Name=' + name + '\n'\
                 'Icon=' + icon + '\n'\
                 'Exec=' + command
    try:
        desktop_file = open(file_path, 'w+')
        desktop_file.write(format_str)
        os.chmod(file_path, 0775)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print sys.exc_info()
        exit(1)

try:
    os.remove(ROOT_DIR + "/src/CMakeLists.txt")
except:
    pass

print "\nInitializing ROS workspace at " + ROOT_DIR + "/src";
if not subprocess.call("cd " + ROOT_DIR + "/src\n catkin_init_workspace", shell=True) == 0:
    print "Unable to execute catkin_init_workspace. Have you installed ROS?!"
    exit(1)


cleanup_bashrc() #Remove obsolete config paths
with open(HOME_DIR + "/.bashrc", 'r+') as fbashrc:
    contents = fbashrc.read()
    add_to_file(fbashrc, contents, "source /opt/ros/hydro/setup.bash")
    add_to_file(fbashrc, contents, "source " + LOCAL_BASH_FILE)
    add_to_file(fbashrc, contents, "export PATH=" + LAUNCHER_DIR + ":$PATH")




print ""
ensure_dir(LAUNCHER_DIR)
gen_ad_rebuild_bash             (LAUNCHER_DIR + "/ad_rebuild")
gen_ad_rebuild_eclipse_bash     (LAUNCHER_DIR + "/ad_rebuild_eclipse")
gen_ad_airdrone_test_bash       (LAUNCHER_DIR + "/ad_airdrone_test")
gen_ad_airdrone_launch_bash     (LAUNCHER_DIR + "/ad_airdrone_launch")
gen_ad_airdrone_real_launch_bash(LAUNCHER_DIR + "/ad_airdrone_real_launch")

print ""
add_launcher('AdRebuild',         ROOT_DIR + '/contrib/icons/AdRebuild.png',            LAUNCHER_DIR + "/ad_rebuild")
add_launcher('AdRebuild4Eclipse', ROOT_DIR + '/contrib/icons/AdRebuildEclipse.png',     LAUNCHER_DIR + "/ad_rebuild_eclipse")
add_launcher('AdSimulator',       ROOT_DIR + '/contrib/icons/AdAirdroneTest.png',       LAUNCHER_DIR + "/ad_airdrone_test")
add_launcher('AdRunInSimulator',  ROOT_DIR + '/contrib/icons/AdAirdroneLaunch.png',     LAUNCHER_DIR + "/ad_airdrone_launch")
add_launcher('AdRun4Real',        ROOT_DIR + '/contrib/icons/AdAirdroneRealLaunch.png', LAUNCHER_DIR + "/ad_airdrone_real_launch")







