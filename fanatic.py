#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import re
import os
import sys

from twisted.internet import protocol, reactor, defer, utils
from twisted.internet.defer import inlineCallbacks, returnValue
from twisted.cred import portal, checkers
from twisted.conch import manhole, manhole_ssh

from adl3 import *

LOOP_DELAY = 3.

class ADLError(Exception):
    pass

def fmt_float(v):
    return '{0:8.3f}'.format(v)

def fmt_floats(*v):
    return ' '.join(map(fmt_float, v))

def clone_env(**kw):
    e = dict(os.environ)
    e.update(kw)

    return e

def check_cmd_code(code):
    if code != os.EX_OK:
        raise ValueError('Could not get temperature')

class TempSensors(object):

    @classmethod
    def read(cls):
        temps = []

        temp = ADLTemperature()
        temp.iSize = sizeof(temp)

        for x in (0, 5, 6, 11):
            if ADL_Overdrive5_Temperature_Get(x, 0, byref(temp)) != ADL_OK:
                raise ADLError("ADL_Overdrive5_Temperature_Get failed.")

            temps.append(temp.iTemperature/1000.0)

        return temps

class PidControl(object):
    INTEGRAL_MAX = 6000.
    INTEGRAL_MIN = 3000.
    INTEGRAL_INIT = 5000.

    def __init__(self, Kp, Ki, Kd):
        self._integral = self.INTEGRAL_INIT / Ki
        self._error = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def loop(self, dt, sp, pv):
        self._previous_error = self._error

        self._error = sp - pv
        self._integral = self._clamp_integral(self._integral + self._error*dt)
        self._derivative = (self._error - self._previous_error)/dt

        return sum(self.state())

    def state(self):
        return self.Kp*self._error, self.Ki*self._integral, self.Kd*self._derivative

    def _clamp_integral(self, i):
        iv = i * self.Ki

        if iv > self.INTEGRAL_MAX:
            return self.INTEGRAL_MAX / self.Ki
        if iv < self.INTEGRAL_MIN:
            return self.INTEGRAL_MIN / self.Ki

        return i


class AdlFan(object):
    FAN_MIN_RPM = 2500.
    FAN_MAX_RPM = 6000.

    FAN_MIN_PCT = 0.
    FAN_MAX_PCT = 100.

    def __init__(self, adapterno):
        self.adapterno = adapterno

    @classmethod
    def _clamp_speed_rpm(cls, v):
        return int(max(min(v, cls.FAN_MAX_RPM), cls.FAN_MIN_RPM))

    @classmethod
    def _clamp_speed_pct(cls, v):
        return int(max(min(v, cls.FAN_MAX_PCT), cls.FAN_MIN_PCT))

    def fan_set_rpm(self, speed):
        fsv = ADLFanSpeedValue()
        fsv.iSize = sizeof(fsv)
        fsv.iSpeedType = ADL_DL_FANCTRL_SPEED_TYPE_RPM
        fsv.iFanSpeed = self._clamp_speed_rpm(speed)
        fsv.iFlags = ADL_DL_FANCTRL_FLAG_USER_DEFINED_SPEED

        if ADL_Overdrive5_FanSpeed_Set(self.adapterno, 0, byref(fsv)) != ADL_OK:
            raise ADLError("ADL_Overdrive5_FanSpeed_Set failed.")

        return fsv.iFanSpeed

    def fan_set_pct(self, speed):
        fsv = ADLFanSpeedValue()
        fsv.iSize = sizeof(fsv)
        fsv.iSpeedType = ADL_DL_FANCTRL_SPEED_TYPE_PERCENT
        fsv.iFanSpeed = self._clamp_speed_pct(speed)
        fsv.iFlags = ADL_DL_FANCTRL_FLAG_USER_DEFINED_SPEED

        if ADL_Overdrive5_FanSpeed_Set(self.adapterno, 0, byref(fsv)) != ADL_OK:
            raise ADLError("ADL_Overdrive5_FanSpeed_Set failed.")

        return fsv.iFanSpeed
    

class FanControl(AdlFan):
    # Setpoints a.k.a. target temperatures
    SP_OFFLINE = 60.
    SP_ONLINE = 81.

    # Startup conditions
    STARTUP_TIME = 120.
    STARTUP_RPM = 5200

    # GPUs considered online once they hit TEMP_ONLINE + TEMP_HYST,
    # they are considered offline once they hit TEMP_ONLINE -
    # TEMP_HYST.
    TEMP_ONLINE = 60.
    TEMP_HYST = 2.

    STATE_MAP = {
        0: 'offline',
        1: 'startup',
        2: 'online',
    }

    def __init__(self, *a):
        super(FanControl, self).__init__(*a)
        self._pid = PidControl(-250., -9., -50.)
        self._state = 0
        self._startup_time = None

    def loop(self, dt, pv):
        if self._state == 0:
            mv_raw = self._st_offline(dt, pv)
        elif self._state == 1:
            mv_raw = self._st_startup(dt, pv)
        elif self._state == 2:
            mv_raw = self._st_online(dt, pv)
        
        if mv_raw is not None:
            mv = self.fan_set_rpm(mv_raw)
        else:
            mv = -1

        print self.STATE_MAP[self._state], self._startup_time
        print '[{0}] {1:5d} RPM\t\t\tPID: {2}'.format(self.adapterno, mv, fmt_floats(*self._pid.state()))

    def _st_offline(self, dt, pv):
        mv = self._pid.loop(dt, self.SP_OFFLINE, pv)

        if pv >= self.TEMP_ONLINE + self.TEMP_HYST:
            self.fan_set_rpm(self.STARTUP_RPM)
            self._state = 1
            self._startup_time = 0.

        return mv

    def _st_startup(self, dt, pv):
        self._pid.loop(dt, self.SP_ONLINE - self.TEMP_HYST, pv)
        self._startup_time += dt

        if self._startup_time > self.STARTUP_TIME:
            self._startup_time = None
            self._state = 2

        return self.STARTUP_RPM

    def _st_online(self, dt, pv):
        mv = self._pid.loop(dt, self.SP_ONLINE, pv)

        if pv < self.TEMP_ONLINE - self.TEMP_HYST:
            self._state = 0

        return mv


fan0 = FanControl(0)
fan1 = FanControl(6)

def sensor_loop(tm=None, iter=0):

    # read temps
    temps = TempSensors.read()

    new_tm = time.time()
    if tm is None:
        dt = LOOP_DELAY
    else:
        dt = new_tm - tm


    # Process variable
    pv0 = max(temps[0:2])
    pv1 = max(temps[2:4])

    print 'iter: {0:05d}, Δt: {1:.4f}'.format(iter, dt)
    print
    print 'Temperatures:\t  {0} {1}\t  {2}'.format(pv0, pv1, repr(temps))
    fan0.loop(dt, pv0)
    fan1.loop(dt, pv1)

    print

    reactor.callLater(LOOP_DELAY, sensor_loop, new_tm, iter+1)

def getManholeFactory(namespace, **passwords):
    realm = manhole_ssh.TerminalRealm()
    realm.chainedProtocolFactory.protocolFactory = lambda x: manhole.Manhole(namespace)

    p = portal.Portal(realm)
    p.registerChecker(checkers.InMemoryUsernamePasswordDatabaseDontUse(**passwords))
    f = manhole_ssh.ConchFactory(p)
    return f

#reactor.listenTCP(2222, getManholeFactory(globals(), joe='aaa'))

if __name__ == '__main__':
    #os.environ['DISPLAY'] = ':0'
    if ADL_Main_Control_Create(ADL_Main_Memory_Alloc, 1) != ADL_OK:
        raise ADLError('Cannot initialize ADL.')

    sensor_loop()
    reactor.run()
