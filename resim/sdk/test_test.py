import unittest
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch, mock_open

import httpx

from resim.sdk.test import Test, LogType
from resim.sdk.client.models.light_job_status import LightJobStatus
from resim.sdk.client.types import Unset

PROJECT_ID = "project-123"
BATCH_ID = "batch-789"
JOB_ID = "job-456"
TEST_NAME = "my-test"
CONFIG_PATH = "/fake/path/config.resim.yml"
UPLOAD_URL = "https://upload.example.com/emissions"


class TestTest(unittest.TestCase):
    def _make_test(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
    ) -> Test:
        """A Test whose API calls are stubbed, ready to upload or close."""
        mock_batch = MagicMock()
        mock_batch.project_id = PROJECT_ID
        mock_batch.id = BATCH_ID
        mock_batch.metrics_config_path = CONFIG_PATH

        create = MagicMock(status_code=201)
        create.parsed.job_id = JOB_ID
        mock_create_job.sync_detailed.return_value = create

        log = MagicMock(status_code=201)
        log.parsed.upload_url = UPLOAD_URL
        mock_create_log.sync_detailed.return_value = log

        mock_httpx.put.return_value = MagicMock(status_code=200)
        mock_close_job.sync_detailed.return_value = MagicMock(status_code=204)

        return Test(MagicMock(), mock_batch, TEST_NAME)

    def _patched_open(self) -> Any:
        content = b"fake emissions data"
        m = mock_open(read_data=content)
        m.return_value.__enter__.return_value.read.side_effect = [
            content,
            b"",
            content,
        ]
        return patch("builtins.open", m)

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_test_creation(
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

        self.assertEqual(test.config_paths, [Path(CONFIG_PATH)])
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

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_attach_log_without_log_type(
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
                test.attach_log("some_image.jpeg")

        # Omitting log_type leaves it off the request body so the server can
        # infer it from the file name.
        first_call_body = mock_create_log.sync_detailed.call_args_list[0].kwargs["body"]
        self.assertIsInstance(first_call_body.log_type, Unset)
        self.assertNotIn("logType", first_call_body.to_dict())

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_attach_system_log(
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
        system_log_content = b"fake system log data"
        m = mock_open(read_data=emissions_content)
        m.return_value.__enter__.return_value.read.side_effect = [
            system_log_content,
            b"",  # SHA256 for system log
            system_log_content,  # httpx.put upload for system log
            emissions_content,
            b"",  # SHA256 for emissions
            emissions_content,  # httpx.put upload for emissions
        ]
        with (
            patch("builtins.open", m),
            patch("os.path.getsize", return_value=len(system_log_content)),
        ):
            with Test(mock_client, mock_batch, TEST_NAME) as test:
                test.attach_system_log("logs/run.log", file_name="robot.log")

        # System logs are always uploaded as SYSTEM_LOG, and the file_name
        # override is passed through.
        self.assertEqual(mock_create_log.sync_detailed.call_count, 2)
        first_call_body = mock_create_log.sync_detailed.call_args_list[0].kwargs["body"]
        self.assertEqual(first_call_body.log_type, LogType.SYSTEM_LOG)
        self.assertEqual(first_call_body.file_name, "robot.log")

    @patch("resim.sdk.test.os.unlink")
    @patch("resim.sdk.test.tempfile.NamedTemporaryFile")
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_exception_uploads_stacktrace_and_closes_with_error(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
        mock_named_tmp_file: Any,
        mock_unlink: Any,
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

        STACKTRACE_PATH = "/tmp/execution_log_test.txt"
        mock_tmp_file = MagicMock()
        mock_tmp_file.__enter__ = MagicMock(return_value=mock_tmp_file)
        mock_tmp_file.__exit__ = MagicMock(return_value=None)
        mock_tmp_file.name = STACKTRACE_PATH
        mock_named_tmp_file.return_value = mock_tmp_file

        stacktrace_content = b"fake stacktrace"
        emissions_content = b"fake emissions data"
        m = mock_open()
        # Two attach_log calls: stacktrace (CONTAINER_LOG) then emissions (EMISSIONS_LOG).
        # Each reads the file twice: once for SHA256 (chunked, needs b"" EOF sentinel)
        # and once for the httpx.put upload.
        m.return_value.__enter__.return_value.read.side_effect = [
            stacktrace_content,
            b"",  # SHA256 EOF for stacktrace
            stacktrace_content,  # httpx.put upload for stacktrace
            emissions_content,
            b"",  # SHA256 EOF for emissions
            emissions_content,  # httpx.put upload for emissions
        ]

        test_exception = RuntimeError("test error")
        with (
            patch("builtins.open", m),
            patch("os.path.getsize", return_value=len(stacktrace_content)),
        ):
            with self.assertRaises(RuntimeError):
                with Test(mock_client, mock_batch, TEST_NAME):
                    raise test_exception

        # First log call should be the stacktrace (CONTAINER_LOG)
        self.assertEqual(mock_create_log.sync_detailed.call_count, 2)
        first_call_body = mock_create_log.sync_detailed.call_args_list[0].kwargs["body"]
        self.assertEqual(first_call_body.log_type, LogType.CONTAINER_LOG)

        # close_job should be called with ERROR status and the error message
        self.assertEqual(mock_httpx.put.call_count, 2)
        close_call_body = mock_close_job.sync_detailed.call_args_list[0].kwargs["body"]
        self.assertEqual(close_call_body.status, LightJobStatus.ERROR)
        self.assertEqual(close_call_body.error_message, repr(test_exception))

        # Temp file should be cleaned up
        mock_unlink.assert_called_once_with(STACKTRACE_PATH)

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_upload_emissions_leaves_the_job_open(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
    ) -> None:
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )

        with self._patched_open():
            test.upload_emissions()

        mock_httpx.put.assert_called_once()
        mock_close_job.sync_detailed.assert_not_called()

        # A second upload is a no-op, and close still closes the job once.
        with self._patched_open():
            test.upload_emissions()
            test.close()
            test.close()

        mock_httpx.put.assert_called_once()
        mock_close_job.sync_detailed.assert_called_once()

    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_close_uploads_when_upload_emissions_was_not_called(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
    ) -> None:
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )

        with self._patched_open():
            test.close()

        mock_httpx.put.assert_called_once()
        mock_close_job.sync_detailed.assert_called_once()

    @patch("resim.sdk.test.time.sleep")
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_upload_retries_a_dropped_connection(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
        mock_sleep: Any,
    ) -> None:
        # A presigned PUT is idempotent, so a reset connection is retried
        # rather than losing the run's work.
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )
        mock_httpx.TransportError = httpx.TransportError
        mock_httpx.put.side_effect = [
            httpx.ConnectError("reset by peer"),
            MagicMock(status_code=200),
        ]

        with self._patched_open():
            test.upload_emissions()

        self.assertEqual(mock_httpx.put.call_count, 2)
        mock_sleep.assert_called_once()

    @patch("resim.sdk.test.time.sleep")
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_upload_retries_a_server_error(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
        mock_sleep: Any,
    ) -> None:
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )
        mock_httpx.TransportError = httpx.TransportError
        mock_httpx.put.side_effect = [
            MagicMock(status_code=503, content=b"slow down"),
            MagicMock(status_code=200),
        ]

        with self._patched_open():
            test.upload_emissions()

        self.assertEqual(mock_httpx.put.call_count, 2)

    @patch("resim.sdk.test.time.sleep")
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_upload_does_not_retry_a_client_error(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
        mock_sleep: Any,
    ) -> None:
        # An expired or malformed presigned URL will not fix itself.
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )
        mock_httpx.TransportError = httpx.TransportError
        mock_httpx.put.return_value = MagicMock(status_code=403, content=b"expired")

        with self._patched_open():
            with self.assertRaises(Exception) as ctx:
                test.upload_emissions()

        self.assertEqual(mock_httpx.put.call_count, 1)
        self.assertIn("403", str(ctx.exception))

    @patch("resim.sdk.test.time.sleep")
    @patch("resim.sdk.test.httpx")
    @patch("resim.sdk.test.close_job")
    @patch("resim.sdk.test.create_job_log")
    @patch("resim.sdk.test.create_job_for_batch")
    def test_upload_gives_up_after_the_last_attempt(
        self,
        mock_create_job: Any,
        mock_create_log: Any,
        mock_close_job: Any,
        mock_httpx: Any,
        mock_sleep: Any,
    ) -> None:
        test = self._make_test(
            mock_create_job, mock_create_log, mock_close_job, mock_httpx
        )
        mock_httpx.TransportError = httpx.TransportError
        mock_httpx.put.side_effect = httpx.ConnectError("down")

        with self._patched_open():
            with self.assertRaises(Exception) as ctx:
                test.upload_emissions()

        self.assertEqual(mock_httpx.put.call_count, 4)
        self.assertIn("ConnectError", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
