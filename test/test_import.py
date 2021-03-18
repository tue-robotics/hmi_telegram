import unittest


class TestImport(unittest.TestCase):
    def test_import(self):
        """
        If no exception is raised, this test will succeed
        """
        import hmi_telegram


if __name__ == '__main__':
    unittest.main()
