import sys 
import unittest


class TestPythonVersion(unittest.TestCase):
    def test_python_version(self): 
        version_info = sys.version_info 
        self.assertEqual(version_info[0], 3)
        self.assertEqual(version_info[1], 10)
        self.assertEqual(version_info[2], 2)

if __name__ == '__main__':
    unittest.main()
