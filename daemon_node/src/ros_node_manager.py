#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import signal
import subprocess, threading
import sys, time
import traceback
import rospy


def singleton(cls):
    instances = {}

    def getinstance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return getinstance


class ProcessWrapper(object):

    def __init__(self, command_line, name):
        self.time_of_death = None
        self.started = False
        self.popen = None
        self.exit_code = None
        self.args = []
        self.pid = -1
        self.command_line = command_line
        self.name = name

    def wait(self):
        if self.started:
            self.popen.wait()

    def start(self):
        """
        Start a manager in process name
        """
        try:
            # self.popen = subprocess.Popen(args=[self.command_line], shell=True, close_fds=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            cmd_list = self.command_line.split(" ")
            FNULL = open(os.devnull, 'w')
            # never use subprocess.PIPE as stdout, set stdout to log file or devnull
            self.popen = subprocess.Popen(args=cmd_list, close_fds=True, preexec_fn=os.setsid, stdout=FNULL, stderr=subprocess.STDOUT)
        except Exception as err:
            rospy.logfatal('Subprocess Popen exception: {}'.format(err))
            return 1
        else:
            if self.popen.pid == 0 or self.popen.returncode is not None:
                rospy.logfatal('Start process {} failed.'.format(self.name))
                return 2

        self.started = True
        self.pid = self.popen.pid
        rospy.logwarn('Start process {} successfully. pid: {}'.format(self.name, self.popen.pid))
        return 0

    def is_alive(self):
        """
        Check the process if is still running
        @return: True if process is still running
        @rtype: bool
        """
        if not self.started:
            return False

        if self.popen is None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False

        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False
        return True

    def get_exit_state(self):
        """
        @return: description of exit state
        @rtype: str
        """
        if self.popen.returncode is None:
            pass
        elif self.popen.returncode != 0:
            output = 'Process {} has died [pid: {}, exitcode: {}, cmd: {}]'.format(self.name, self.pid, self.exit_code, self.command_line)
            rospy.logerr(output)
        else:
            output = 'Process {} has finished. [pid: {}, cmd: {}]'.format(self.name, self.pid, self.command_line)
            rospy.logerr(output)

@singleton
class ProcessMonitor(object):

    def __init__(self):
        self.procs = []
        self.done = False
        self.is_shutdown = False

    def register(self, p):
        """
        Register process with L{ProcessMonitor}
        @param p: Process
        @type  p: L{Process}
        """
        if self.has_process(p.name):
            rospy.logerr('cannot add process due to duplicate name "{}".'.format(p.name))
            return False
        elif self.is_shutdown:
            rospy.logerr('cannot add process {} due to monitor has been stopped.'.format(p.name))
            return False
        else:
            self.procs.append(p)
            return True

    def has_process(self, name):
        """
        @return: True if process is still be monitored. If False, process
        has died or was never registered with process
        @rtype: bool
        """
        return len([p for p in self.procs if p.name == name]) > 0

    def stop(self, kill_name, signal=signal.SIGTERM):
        """
        Stop processes in monitor
        """
        for p in self.procs:
            if p.name == kill_name and p.is_alive():
                p.popen.send_signal(signal)
                rospy.logwarn("send signal to {}".format(kill_name))
                break

        for p in self.procs:
            if p.name == kill_name:
                if p.is_alive():
                    rospy.logwarn('Waiting for [{}][{}] exit'.format(p.name, p.pid))
                    p.wait()
                rospy.logwarn('process [{}] has been stopped'.format(p.name))
        rospy.logwarn("skip, process {} not alive".format(kill_name))

    def check_cleanup(self, signal=signal.SIGTERM):
        """
        Stop all processes in monitor
        """
        for p in self.procs:
            if p.is_alive():
                p.popen.send_signal(signal)
                rospy.logwarn("send signal to {}".format(p.name))

        for p in self.procs:
            if p.is_alive():
                rospy.logwarn('Waiting for [{}][{}] exit'.format(p.name, p.pid))
                p.wait()
            rospy.logwarn('process [{}] has been stopped'.format(p.name))
        self.procs = []
        rospy.logwarn("cleanup done, all process killed")
