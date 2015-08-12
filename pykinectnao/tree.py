__author__ = 'Angeall'


class Node(object):
    def __init__(self, value=None, children=[]):
        self.value = value
        self.children = children

    def add_child(self, value=None):
        self.children.append(Node(value))


class Tree(object):
    def __init__(self, value=None, children=[]):
        self.root = Node(value, children)

