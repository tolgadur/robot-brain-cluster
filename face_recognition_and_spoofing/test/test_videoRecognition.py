import unittest
import src.VideoRecognition


class TestVideoRecognition(unittest.TestCase):
    def setUp(self):
        self.videoRecognition = src.VideoRecognition.VideoRecognition()

    def test_run(self):
        try:
            self.videoRecognition.run()
        except Exception:
            self.fail()
        else:
            pass


if __name__ == '__main__':
    unittest.main()