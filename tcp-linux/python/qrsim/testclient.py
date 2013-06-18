#!/usr/bin/env python

from qrsim.tcpclient import TCPClient
import argparse
import os
import sys
import traceback


def test(name):
    def make_test(fn):
        def test_func(*args, **kwargs):
            sys.stdout.write("QRSIM %s test " % name)
            try:
                val = fn(*args, **kwargs)
                sys.stdout.write("[PASSED]" + os.linesep)
                return val
            except Exception as err:
                sys.stdout.write("[FAILED]" + os.linesep)
                sys.stderr.write(str(err) + os.linesep)
                sys.stderr.write(traceback.format_exc() + os.linesep)
        return test_func
    return make_test


@test('init')
def test_init(client):
    client.init('TaskKeepSpot', False)


@test('disconnect')
def test_disconnect(client):
    client.disconnect()


@test('quit')
def test_quit(client):
    client.quit()


@test('reset')
def test_reset(client):
    client.reset()


@test('stepWP')
def test_step_wp(client):
    wps = [[0, 0, -10, 0] for i in xrange(client.numUAVs)]
    client.step_wp(0.1, wps)


@test('stepCtrl')
def test_step_ctrl(client):
    ctrls = [[0, 0, 0.53, 0, 10] for i in xrange(client.numUAVs)]
    client.step_ctrl(0.1, ctrls)


@test('stepVel')
def test_step_vel(client):
    velocities = [[0.1, 0, 0] for i in xrange(client.numUAVs)]
    client.step_vel(0.1, velocities)


@test('rpc')
def test_rpc(client):
    client.init('TaskPlumeSingleSourceGaussian', False)
    client.reset()
    assert [1.0] == client.rpc('TASK', 'getSamplesPerLocation')


@test('platforms rpc')
def test_platforms_rpc(client):
    client.init('TaskPlumeSingleSourceGaussianDefaultControls', False)
    client.reset()
    wps = [[0, 0, -10, 0] for i in xrange(client.numUAVs)]
    client.step_wp(0.1, wps)
    assert len(client.rpc('PLATFORMS', 'getPlumeSensorOutput')) > 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Test the QRSim TCP client Python implementation.')
    parser.add_argument('ip', nargs=1, type=str)
    parser.add_argument('port', nargs=1, type=int)
    args = parser.parse_args()

    client = TCPClient()
    client.connect_to(args.ip[0], args.port[0])
    test_init(client)
    test_reset(client)
    test_step_wp(client)
    test_step_ctrl(client)
    test_step_vel(client)
    test_rpc(client)
    test_platforms_rpc(client)
    test_disconnect(client)

    client.connect_to(args.ip[0], args.port[0])
    test_quit(client)
