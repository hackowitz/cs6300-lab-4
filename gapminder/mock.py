import logging
import re

class Node:

    def __init__(self, name: str = None):
        self._name = name

    def get_logger(self):
        return logging.getLogger(f'{self._name}')

    def create_publisher(self, *_, **__):
        return Publisher()

    def create_subscription(self, *_, **__):
        return Subscriber()

class dotdict(dict):

    def __getattr__(self, key):
        return self[key]



class LaserScan(dotdict):

    def __init__(self, other: dict = None):
        if other is not None:
            for key, value in other.items():
                if isinstance(key, str):
                    *head, tail = key.split('.')
                    node = self
                    for key in head:
                        node = node.setdefault(key, dotdict())
                    if match := re.match(r'(?P<array>\w+)\[(?P<index>\d+)\]', tail):
                        array = node.setdefault(match['array'], [])
                        for i in range(len(array) - 1, index := int(match['index'])):
                            array.append(None)
                        array[index] = value
                    else:
                        node[tail] = value


class AckermannDriveStamped(dotdict):

    def __init__(self):
        self.drive = dotdict()


class Publisher:

    def publish(msg):
        ...

class Subscriber:

    ...
