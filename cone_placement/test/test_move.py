#! /usr/bin/env python

#TODO make some tests (ask chatGPT for this?) 

PKG = 'cone_placement'

import sys
import unittest

from giraffe_interface import GiraffeMoveBaseClient

class TestMove(unittest.TestCase):
    # testing 
    def test_one_equals_one(self):
        self.assertEqual(1,1, "1!=1")
    
    def test_move_base(self):

        pass

if __name__=='__main__':
    import rostest
    rostest.rosrun(PKG, 'test_move', TestMove)