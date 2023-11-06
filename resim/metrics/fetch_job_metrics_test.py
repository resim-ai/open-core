
import unittest
import resim.metrics.fetch_job_metrics as fjm
import uuid

class FetchJobMetricsTest(unittest.TestCase):
    def test_fetch(self):
        fjm.fetch_job_metrics(
            batch_ids=[
                uuid.UUID("b6ff9ce1-a481-4793-a9ea-d607f6efe628"),
                uuid.UUID("0a74e080-7782-4a6c-ac26-248caa575405")],
            job_ids=[
                uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"),
                uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010")])


if __name__ == '__main__':
    unittest.main()
