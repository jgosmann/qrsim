#!/usr/bin/env python

from setuptools import setup, find_packages
from distutils.core import Command, Distribution
from distutils.command.clean import clean as _clean
from distutils.command.build_py import build_py as _build_py
from distutils.errors import DistutilsOptionError, DistutilsExecError
from distutils.spawn import find_executable
import os.path
import subprocess


class build_proto(Command):
    description = "Generate Python source code from proto files."
    user_options = [('with-protoc', None, "Path to protoc compiler.")]

    def initialize_options(self):
        self.protoc = find_executable('protoc')

    def finalize_options(self):
        if self.protoc is None:
            raise DistutilsOptionError('Could not locate protoc.')

    def run(self):
        if self.distribution.protos is None:
            return

        for filename, outdir in self.distribution.protos:
            if subprocess.call([
                    self.protoc, '--python_out=' + outdir,
                    '-I' + os.path.dirname(filename), filename]) != 0:
                DistutilsExecError('Non-zero return value of protoc.')


class clean(_clean):
    def run(self):
        for dirpath, dirnames, filenames in os.walk('.'):
            for filename in filenames:
                if filename.endswith('_pb2.py'):
                    os.remove(os.path.join(dirpath, filename))
        _clean.run(self)


class build_py(_build_py):
    def run(self):
        self.run_command('build_proto')
        _build_py.run(self)


class DistWithProto(Distribution):
    def __init__(self, attrs=None):
        self.protos = None
        Distribution.__init__(self, attrs)


setup(
    name='py-qrsim-tcpclient',
    version='1.0',
    description='Python implementation and API of QRSim TCP client.',
    author='Jan Gosmann',
    author_email='jan.gosmann@bccn-berlin.de',
    packages=find_packages(),
    setup_requires=['protobuf'],
    install_requires=['protobuf'],
    distclass=DistWithProto,
    cmdclass={
        'build_proto': build_proto, 'clean': clean, 'build_py': build_py},
    protos=[('../proto/qrs_srv_cli_msg.proto', 'qrsim')]
)
