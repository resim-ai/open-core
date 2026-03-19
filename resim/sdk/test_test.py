import unittest
from typing import Any
from unittest.mock import MagicMock, patch, mock_open

from resim.sdk.test import Test, LogType

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
    def test_happy_path(
        self,
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
        # mock_open returns the same value on every read(), so iter(f.read, b"") would
        # loop forever. Give read() a side_effect that returns data once then b"" (EOF).
        m.return_value.__enter__.return_value.read.side_effect = [
            emissions_content,
            b"",  # SHA256 chunked read in attach_log
            emissions_content,  # httpx.put content read in attach_log
        ]
        with (
            patch("builtins.open", m),
            patch("os.path.getsize", return_value=len(emissions_content)),
        ):
            with Test(mock_client, mock_batch, TEST_NAME) as test:
                self.assertEqual(test.name, TEST_NAME)

        self.assertEqual(str(test.config_path), CONFIG_PATH)
        mock_create_job.sync_detailed.assert_called_once()
        mock_create_log.sync_detailed.assert_called_once()
        mock_httpx.put.assert_called_once()
        mock_close_job.sync_detailed.assert_called_once()

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_attach_log(
        self,
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
        extra_log_content = b"fake image data"
        m = mock_open(read_data=emissions_content)
        # Each attach_log call reads the file twice: once for SHA256 (chunked, needs EOF
        # sentinel b"") and once for the httpx.put upload. Two attach_log calls total:
        # first for the explicit attach, second for the emissions file on close.
        m.return_value.__enter__.return_value.read.side_effect = [
            extra_log_content,
            b"",  # SHA256 for extra log
            extra_log_content,  # httpx.put upload for extra log
            emissions_content,
            b"",  # SHA256 for emissions
            emissions_content,  # httpx.put upload for emissions
        ]
        with (
            patch("builtins.open", m),
            patch("os.path.getsize", return_value=len(emissions_content)),
        ):
            with Test(mock_client, mock_batch, TEST_NAME) as test:
                test.attach_log("some_image.jpeg", LogType.MP4_LOG)

        # Two create_job_log calls: one for the extra log, one for emissions on exit
        self.assertEqual(mock_create_log.sync_detailed.call_count, 2)
        self.assertEqual(mock_httpx.put.call_count, 2)

        first_call_body = mock_create_log.sync_detailed.call_args_list[0].kwargs["body"]
        self.assertEqual(first_call_body.log_type, LogType.MP4_LOG)


if __name__ == "__main__":
    unittest.main()
