import unittest
import src.ModelTrainer
import os
import sys
import pickle
import numpy as np


class TestModelTrainer(unittest.TestCase):

    sys.path.append('../src')
    def setUp(self):
        self.modelTrainer = src.ModelTrainer.ModelTrainer()

    def test_train_with_camera(self):
        self.assertEqual(1, 1)
        path, dirs, files = next(os.walk("./dataset/Tolga/"))
        file_count_before = len(files)

        self.modelTrainer.train_with_camera("Tolga")

        path, dirs, files = next(os.walk("./dataset/Tolga/"))
        file_count_after = len(files)
        self.assertEqual(file_count_before, file_count_after)

    def test_extract_embeddings(self):
        embeddings_before = pickle.loads(open(self.modelTrainer.embeddings, "rb").read())

        self.modelTrainer.extract_embeddings()

        embeddings_after = pickle.loads(open(self.modelTrainer.embeddings, "rb").read())

        self.assertTrue(np.array_equal(embeddings_before["embeddings"], embeddings_after["embeddings"]))

    def test_train_model(self):
        try:
            self.modelTrainer.train_model()
        except Exception:
            self.fail()
        else:
            pass


if __name__ == '__main__':
    unittest.main()
