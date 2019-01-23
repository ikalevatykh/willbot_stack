import socket
import click
import numpy as np
import os
import pickle
import socket
from pathlib import Path



class StreamIO(object):
    def __init__(self, conn):
        self._conn = conn
        self._stream = conn.makefile('rwb')

    def read(self):
        return pickle.load(self._stream, encoding='latin1')

    def write(self, data):
        pickle.dump(data, self._stream, protocol=2, fix_imports=True)
        self._stream.flush()

    def close(self):
        self._stream.close()


@click.command(help='agent_server')
@click.option('-p', '--port', type=int, default=18000)
def main(port):
    sock = socket.socket()
    sock.bind(('', port))
    sock.listen(1)

    velocity_scale = 1
    previous_settings = None

    while True:
        print('Waiting for client [{}] ...'.format(port))
        conn, addr = sock.accept()
        stream = StreamIO(conn)
        print('Connected')

        try:
            settings = stream.read()
            print(settings)
            if previous_settings == settings:
                pass
            elif settings['agent_type'] == 'network':
                print('Setup network...')
                # you can use settings to setup network
                agent = YourNetworkAgentClass inherits MIME Agent

            previous_settings = settings
            print('ok')
            stream.write({'setup': True})

            while True:
                obs = stream.read()
                print('obs', obs.keys())
                act = agent.get_action(obs)
                print('act', act)
                act_mod = None
                if act is not None:
                    linear_velocity = act['linear_velocity'] * velocity_scale
                    grip_velocity = act['grip_velocity']

                    act_mod = linear_velocity.tolist() + [float(grip_velocity), ]

                stream.write(act_mod)
                if act is None:
                    break
        except Exception as e:
            print(e)
        finally:
            stream.close()
            conn.close()

    sock.close()


if __name__ == "__main__":
    main()
