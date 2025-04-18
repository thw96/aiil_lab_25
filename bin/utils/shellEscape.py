#! /usr/bin/env python3

'''
Util methods for capturing or running shell commands
'''
from utils.echo import (
    print_error,
    print_status,
    print_warning
)
import subprocess

'''
Capture the output of an exec command
'''
def capture(command, emptyFail=False):
    retVal = None
    
    try:
        data = subprocess.check_output(command, shell=True)
        retVal = data.decode('utf-8').rstrip()
    except subprocess.CalledProcessError as err:
        if not emptyFail:
            print_error("Subprocess cal error: {0}".format(err))
            raise
        else:
            retVal = ""
    
    return retVal

'''
Check the status of a command using subprocess check
'''
def check_output(command):
    try:
        stderr = subprocess.DEVNULL
        subprocess.check_output(command, stderr=stderr, shell=True)
        return True
    except subprocess.CalledProcessError:
        return False

''' 
Execute the command
Return True/False if the command was successful
'''
def exec(command, hideOutput=True):
    stdout = None
    stderr = None
    if hideOutput:
        stdout = subprocess.DEVNULL
        stderr = subprocess.DEVNULL
    returncode = subprocess.call(command, stdout=stdout, stderr=stderr, shell=True)
    return returncode == 0