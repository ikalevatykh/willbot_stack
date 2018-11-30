from collections import OrderedDict
import cPickle as pickle
import socket

import click
import numpy as np


class AgentClient(object):
    def __init__(self, host, port=18000):
        self._sock = None
        self._stream = None
        self._sock = socket.socket()
        self._sock.connect((host, port))
        self._stream = self._sock.makefile('rwb')

    def send_settings(self, settings):
        pickle.dump(settings, self._stream, protocol=2)
        self._stream.flush()
        response = pickle.load(self._stream)
        if response is None:
            return None
        return response['setup']

    def get_action(self, obs):
        pickle.dump(obs, self._stream, protocol=2)
        self._stream.flush()
        act_raw = pickle.load(self._stream)
        if act_raw is None:
            return None
        
        act = OrderedDict([
            ('grip_velocity', act_raw[3]), 
            ('linear_velocity', np.array(act_raw[:3]))
        ])
        return act

    def __del__(self):
        if self._stream is not None:
            self._stream.close()
        if self._sock is not None:
            self._sock.close()


@click.command(help='script_client host [options]')
@click.argument('host', type=str)
@click.option('-p', '--port', type=int, default=18000)
def main(host, port):
    agent = AgentClient(host, port)

    while True:
        obs = {'depth': np.array((224, 224), dtype=np.uint8)}
        act = agent.get_action(obs)
        print(act)
        if act is None:
            return

if __name__ == "__main__":
    main()
