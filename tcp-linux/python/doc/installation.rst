Getting Started
===============

In the following will be discussed what is needed to get Python QRSim TCP client
installed.


Requirements
------------

* `Python <http://python.org>`_ >= 2.7 (earlier versions might work, but are untested)
* `Google Protocol Buffers <https://developers.google.com/protocol-buffers/>`_
  >= 2.5.0 (earlier versions might work, but are untested)


Installation
------------

All the magic should be done by::

$ python setup.py install


Test the Installation
---------------------

To test the installation start the QRSim TCP server from Matlab®
with::

>> QRSimTCPServer(10000)

Then run::

$ python -m qrsim.testclient 127.0.0.1 10000
