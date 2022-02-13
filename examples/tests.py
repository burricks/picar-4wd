import unittest
import a_star2
from helps import Node

class TestToArrayIndicesMethod(unittest.TestCase):

    def test_one(self):
        angle = -80
        distance = 50
        currentPosition = Node(60,0)
        res_x, res_y = a_star2.getArrayIndices(angle, distance, currentPosition)
        self.assertEqual(res_x, 109)
        self.assertEqual(res_y, 8)


if __name__ == '__main__':
    unittest.main()