#!/usr/bin/env python

import sys, time


class Value():
    '''a value for the status bar'''
    def __init__(self, name, text, fg='black'):
        self.name = name
        self.text = text
        self.fg = fg

    def write(self, name, text, fg='black'):
        self.name = name
        self.text = text
        self.fg = fg