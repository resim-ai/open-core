import unittest
from typing import Any
from unittest.mock import MagicMock, patch, mock_open

from resim.sdk.test import Test

PROJECT_ID = "project-123"
BATCH_ID = "batch-789"
JOB_ID = "job-456"
TEST_NAME = "my-test"
CONFIG_PATH = "/fake/path/config.resim.yml"
UPLOAD_URL = "https://upload.example.com/emissions"


class TestTest(unittest.TestCase):
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    @patch("resim.sdk.test.Emitter")
    def test_happy_path(
        self,
        mock_emitter_cls: Any,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_batch = MagicMock()
        mock_batch.project_id = PROJECT_ID
        mock_batch.id = BATCH_ID
        mock_batch.metrics_config_path = CONFIG_PATH

        mock_create_response = MagicMock()
        mock_create_response.status_code = 201
        mock_create_response.parsed.job_id = JOB_ID
        mock_create_job.sync_detailed.return_value = mock_create_response

        mock_log_response = MagicMock()
        mock_log_response.status_code = 201
        mock_log_response.parsed.upload_url = UPLOAD_URL
        mock_create_log.sync_detailed.return_value = mock_log_response

        mock_httpx.put.return_value = MagicMock(status_code=200)

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_job.sync_detailed.return_value = mock_close_response

        emissions_content = b"fake emissions data"
        m = mock_open(read_data=emissions_content)
        with (
            patch("builtins.open", m),
            patch("os.path.getsize", return_value=len(emissions_content)),
        ):
            with Test(mock_client, mock_batch, TEST_NAME) as test:
                self.assertEqual(test.name, TEST_NAME)

        mock_emitter_cls.assert_called_once_with(
            config_path=CONFIG_PATH,
            output_path=test._emissions_output_path,
        )
        mock_create_job.sync_detailed.assert_called_once()
        mock_create_log.sync_detailed.assert_called_once()
        mock_httpx.put.assert_called_once()
        mock_close_job.sync_detailed.assert_called_once()


if __name__ == "__main__":
    unittest.main()
