#!/usr/bin/python
# Author: Anton Mitrokhin, MIPT 2014

import sys, os, subprocess, shutil

HOME_DIR = os.path.expanduser("~")
INSTALL_DIR = '/opt'


def add_eclipse_launcher():
    file_path = HOME_DIR + '/.local/share/applications/eclipse.desktop'
    format_str = '[Desktop Entry]\n'\
                 'Name=Eclipse\n'\
                 'Type=Application\n'\
                 'Exec='+INSTALL_DIR+'/eclipse/eclipse\n'\
                 'Terminal=false\n'\
                 'Icon=/opt/eclipse/icon.xpm\n'\
                 'Comment=Integrated Development Environment\n'\
                 'NoDisplay=false\n'\
                 'Categories=Development;IDE\n'\
                 'Name[en]=Eclipse\n'\
                 'Exec=env UBUNTU_MENUPROXY=0 '+INSTALL_DIR+'/eclipse/eclipse'
    try:
        desktop_file = open(file_path, 'w+')
        desktop_file.write(format_str)
        os.chmod(file_path, 0775)
        print "Successfully written to", file_path
    except:
        print "Error creating ", file_path
        print sys.exc_info()
        exit(1)


def ensure_dir(f):
    if not os.path.exists(f):
        print "Created directory", f
        os.makedirs(f)
        os.chmod(f, 0777)


if os.getuid() != 0:
    print "This program is intended to run as sudo"
    exit(1)

print "\nINSTALL CUSTOM ECLIPSE"
print "author: Anton Mitrokhin, 2014"
print "Warning! The downloaded eclipse folder should be in current directory"
print ""


ensure_dir('/randomstuff')

try:
    shutil.move('eclipse', INSTALL_DIR)
except:
    print "The 'eclipse' folder is expected to be at the same directory as this script"
    exit(1)

add_eclipse_launcher()

