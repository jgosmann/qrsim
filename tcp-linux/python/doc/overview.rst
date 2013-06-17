Overview
========

First you have to import all classes you want to use from the package::

    from qrsim.tcpclient import TCPClient

Then you can instantiate a client, connect to a QRSim TCP server and start
issuing commands::

    client = TCPClient()
    client.connect_to('127.0.0.1', 10000)
    client.init('TaskKeepSpot', False)

    waypoints = [[0, 0, -10, 0] for i in xrange(client.numUAVs)]
    client.step_wp(0.1, waypoints)

You can also access the UAV state vectors::

    print('The true position:')
    print(client.state.position)
    print('The position estimated by GPS:')
    print(client.noisy_state.position)

Do not forget to disconnect from the server in the end::

    client.disconnect()

It is also possible completely shutdown the server::

    client.quit()
