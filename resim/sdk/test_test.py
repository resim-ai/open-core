import os
import tempfile
import threading
import unittest
from pathlib import Path
from typing import Any, Union
from unittest.mock import MagicMock, patch

import httpx

from resim.sdk.test import Test, LogType, LogUploadError, UPLOAD_TIMEOUT_S
from resim.sdk.client.models.light_job_status import LightJobStatus
from resim.sdk.client.types import Unset

PROJECT_ID = "project-123"
BATCH_ID = "batch-789"
JOB_ID = "job-456"
TEST_NAME = "my-test"
CONFIG_PATH = "/fake/path/config.resim.yml"
UPLOAD_URL_PREFIX = "https://upload.example.com"


def _response(status_code: int) -> MagicMock:
    return MagicMock(status_code=status_code, content=b"")


def _create_log_response(*_args: Any, **kwargs: Any) -> MagicMock:
    """Answer create_job_log with an upload URL that identifies the file."""
    response = MagicMock()
    response.status_code = 201
    response.parsed.upload_url = f"{UPLOAD_URL_PREFIX}/{kwargs['body'].file_name}"
    response.parsed.required_headers.to_dict.return_value = {}
    return response


class _UploadResponder:
    """Thread-safe stand-in for httpx.put that answers per destination file.

    Uploads run in parallel, so responses are keyed by the file name in the
    upload URL rather than by call order. Files with no queued outcome get the
    default status.
    """

    def __init__(self, default_status: int = 200):
        self._queued: dict[str, list[Union[int, BaseException]]] = {}
        self._default_status = default_status
        self._lock = threading.Lock()

    def queue(self, file_name: str, outcomes: list[Union[int, BaseException]]) -> None:
        """Queue the outcomes for successive upload attempts of one file."""
        self._queued[file_name] = list(outcomes)

    def __call__(self, url: str, **_kwargs: Any) -> MagicMock:
        file_name = url.rsplit("/", 1)[-1]
        with self._lock:
            queue = self._queued.get(file_name)
            outcome: Union[int, BaseException] = (
                queue.pop(0) if queue else self._default_status
            )
        if isinstance(outcome, BaseException):
            raise outcome
        return _response(outcome)


class TestTest(unittest.TestCase):
    """Tests for the Test lifecycle and its log uploads.

    Each test runs in its own temporary working directory, because the emitter
    writes its emissions file relative to the current directory. Log files on
    disk are real, so checksums and uploads run the same code as production.
    """

    def setUp(self) -> None:
        self._tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp_dir.cleanup)
        previous_dir = os.getcwd()
        os.chdir(self._tmp_dir.name)
        self.addCleanup(os.chdir, previous_dir)

        self.client = MagicMock()
        self.batch = MagicMock()
        self.batch.project_id = PROJECT_ID
        self.batch.id = BATCH_ID
        self.batch.metrics_config_path = CONFIG_PATH

        self.mock_create_job = self._patch("resim.sdk.test.create_job_for_batch")
        self.mock_create_log = self._patch("resim.sdk.test.create_job_log")
        self.mock_close_job = self._patch("resim.sdk.test.close_job")
        self.mock_httpx = self._patch("resim.sdk.test.httpx")
        # Retry backoff would otherwise add seconds to the suite.
        self.mock_sleep = self._patch("resim.sdk.test.time.sleep")

        create_job_response = MagicMock()
        create_job_response.status_code = 201
        create_job_response.parsed.job_id = JOB_ID
        self.mock_create_job.sync_detailed.return_value = create_job_response
        self.mock_create_log.sync_detailed.side_effect = _create_log_response
        self.mock_close_job.sync_detailed.return_value = _response(204)

        # Uploads go through a per-test httpx.Client, not the module function.
        self.mock_client_instance = self.mock_httpx.Client.return_value
        self.put = self.mock_client_instance.put
        self.uploads = _UploadResponder()
        self.put.side_effect = self.uploads

    def _patch(self, target: str) -> MagicMock:
        patcher = patch(target)
        mock = patcher.start()
        self.addCleanup(patcher.stop)
        return mock

    def _write_file(self, name: str, content: bytes = b"log data") -> str:
        path = Path(self._tmp_dir.name) / name
        path.write_bytes(content)
        return str(path)

    def _create_log_bodies(self) -> list[Any]:
        return [
            call.kwargs["body"]
            for call in self.mock_create_log.sync_detailed.call_args_list
        ]

    def _create_log_body(self, file_name: str) -> Any:
        """Find the create_job_log body for one file, whatever order it ran in."""
        for body in self._create_log_bodies():
            if body.file_name == file_name:
                return body
        raise AssertionError(f"no log was created for {file_name}")

    def _close_job_body(self) -> Any:
        return self.mock_close_job.sync_detailed.call_args_list[0].kwargs["body"]

    def test_test_creation(self) -> None:
        with Test(self.client, self.batch, TEST_NAME) as test:
            self.assertEqual(test.name, TEST_NAME)

        self.assertEqual(test.config_paths, [Path(CONFIG_PATH)])
        self.mock_create_job.sync_detailed.assert_called_once()
        # Only the emissions file is uploaded on close.
        self.mock_create_log.sync_detailed.assert_called_once()
        self.put.assert_called_once()
        self.mock_close_job.sync_detailed.assert_called_once()
        self.assertEqual(self._close_job_body().status, LightJobStatus.SUCCEEDED)

    def test_attach_log(self) -> None:
        content = b"fake image data"
        log_path = self._write_file("some_image.jpeg", content)

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path, LogType.MP4_LOG, wait=True)

        # Two create_job_log calls: one for the extra log, one for emissions on exit
        self.assertEqual(self.mock_create_log.sync_detailed.call_count, 2)
        self.assertEqual(self.put.call_count, 2)

        body = self._create_log_bodies()[0]
        self.assertEqual(body.log_type, LogType.MP4_LOG)
        self.assertEqual(body.file_name, "some_image.jpeg")
        self.assertEqual(body.file_size, len(content))

    def test_upload_declares_content_length(self) -> None:
        content = b"fake image data"
        log_path = self._write_file("some_image.jpeg", content)

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path, wait=True)

        # Object stores reject chunked transfer encoding, so the streamed upload
        # must declare its length.
        headers = self.put.call_args_list[0].kwargs["headers"]
        self.assertEqual(headers["Content-Length"], str(len(content)))

    def test_attach_log_without_log_type(self) -> None:
        log_path = self._write_file("some_image.jpeg")

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path, wait=True)

        # Omitting log_type leaves it off the request body so the server can
        # infer it from the file name.
        body = self._create_log_bodies()[0]
        self.assertIsInstance(body.log_type, Unset)
        self.assertNotIn("logType", body.to_dict())

    def test_attach_system_log(self) -> None:
        log_path = self._write_file("run.log")

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_system_log(log_path, file_name="robot.log")

        # System logs are always uploaded as SYSTEM_LOG, and the file_name
        # override is passed through.
        self.assertEqual(self.mock_create_log.sync_detailed.call_count, 2)
        body = self._create_log_body("robot.log")
        self.assertEqual(body.log_type, LogType.SYSTEM_LOG)

    def test_background_uploads_are_awaited_on_close(self) -> None:
        paths = [self._write_file(f"log_{i}.log") for i in range(5)]

        with Test(self.client, self.batch, TEST_NAME) as test:
            for path in paths:
                test.attach_log(path, LogType.CONTAINER_LOG)

        # Every background upload finished before the job was closed.
        self.assertEqual(self.put.call_count, 6)
        uploaded = {body.file_name for body in self._create_log_bodies()}
        self.assertEqual(
            uploaded,
            {f"log_{i}.log" for i in range(5)} | {"emissions.resim.jsonl"},
        )
        self.assertEqual(self._close_job_body().status, LightJobStatus.SUCCEEDED)

    def test_background_upload_does_not_block_the_caller(self) -> None:
        # attach_log(wait=False) hands the work to the pool and returns, so the
        # test body keeps running while the upload is still in flight.
        release = threading.Event()
        started = threading.Event()

        def block_until_released(*_args: Any, **_kwargs: Any) -> MagicMock:
            started.set()
            release.wait(timeout=5)
            return _response(200)

        self.put.side_effect = block_until_released
        log_path = self._write_file("background.log")

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path)
            self.assertTrue(started.wait(timeout=5))
            release.set()

    def test_upload_retries_transient_failures(self) -> None:
        log_path = self._write_file("flaky.log")
        self.uploads.queue(
            "flaky.log", [httpx.ConnectError("connection reset"), 503, 200]
        )

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path)

        # Three tries for the flaky log, plus the emissions upload.
        self.assertEqual(self.put.call_count, 4)
        self.mock_sleep.assert_called()
        self.assertEqual(self._close_job_body().status, LightJobStatus.SUCCEEDED)

    def test_create_log_retries_transient_failures(self) -> None:
        log_path = self._write_file("flaky.log")
        failed_once: set[str] = set()

        def fail_first_time(*args: Any, **kwargs: Any) -> MagicMock:
            name = kwargs["body"].file_name
            if name not in failed_once:
                failed_once.add(name)
                return _response(502)
            return _create_log_response(*args, **kwargs)

        self.mock_create_log.sync_detailed.side_effect = fail_first_time

        with Test(self.client, self.batch, TEST_NAME) as test:
            test.attach_log(log_path, wait=True)

        # One retry each for the log and for the emissions file.
        self.assertEqual(self.mock_create_log.sync_detailed.call_count, 4)

    def test_upload_does_not_retry_client_errors(self) -> None:
        log_path = self._write_file("rejected.log")
        self.uploads.queue("rejected.log", [403])

        with self.assertRaises(Exception) as caught:
            with Test(self.client, self.batch, TEST_NAME) as test:
                test.attach_log(log_path, wait=True)

        self.assertIn("403", str(caught.exception))
        # One try for the rejected log, then the stacktrace and emissions uploads.
        self.assertEqual(self.put.call_count, 3)

    def test_blocking_upload_raises_at_the_call_site(self) -> None:
        log_path = self._write_file("rejected.log")
        self.uploads.queue("rejected.log", [403])
        raised = None

        with self.assertRaises(Exception):
            with Test(self.client, self.batch, TEST_NAME) as test:
                try:
                    test.attach_log(log_path, wait=True)
                except Exception as error:  # noqa: BLE001 - recorded, then re-raised
                    raised = error
                    raise

        # wait=True reports the failure where the caller can catch it, rather
        # than deferring it to close().
        self.assertIn("403", str(raised))

    def test_failed_background_upload_closes_job_with_error(self) -> None:
        log_path = self._write_file("doomed.log")
        self.uploads.queue("doomed.log", [500, 500, 500])

        with self.assertRaises(LogUploadError) as caught:
            with Test(self.client, self.batch, TEST_NAME) as test:
                test.attach_log(log_path)

        self.assertEqual(
            [name for name, _ in caught.exception.failures], ["doomed.log"]
        )
        # Three tries for the failing log, plus the emissions upload. The job is
        # still closed so it does not hang open, but with an ERROR status.
        self.assertEqual(self.put.call_count, 4)
        body = self._close_job_body()
        self.assertEqual(body.status, LightJobStatus.ERROR)
        self.assertIn("doomed.log", body.error_message)

    def test_every_failed_upload_is_reported(self) -> None:
        paths = [self._write_file(f"log_{i}.log") for i in range(2)]
        self.uploads.queue("log_0.log", [400])
        self.uploads.queue("log_1.log", [400])

        with self.assertRaises(LogUploadError) as caught:
            with Test(self.client, self.batch, TEST_NAME) as test:
                for path in paths:
                    test.attach_log(path)

        # Every file is attempted even after the first one fails.
        self.assertEqual(
            sorted(name for name, _ in caught.exception.failures),
            ["log_0.log", "log_1.log"],
        )

    def test_one_http_client_is_shared_and_closed(self) -> None:
        paths = [self._write_file(f"log_{i}.log") for i in range(4)]

        with Test(self.client, self.batch, TEST_NAME) as test:
            for path in paths:
                test.attach_log(path)

        # One client for the whole test, so the connection to the object store
        # is reused instead of re-handshaked per log.
        self.mock_httpx.Client.assert_called_once()
        self.assertEqual(self.put.call_count, 5)
        self.mock_client_instance.close.assert_called_once()

    def test_http_client_carries_the_upload_timeout(self) -> None:
        with Test(self.client, self.batch, TEST_NAME):
            pass

        # The timeout lives on the client, so every upload inherits it.
        self.assertEqual(
            self.mock_httpx.Client.call_args.kwargs["timeout"], UPLOAD_TIMEOUT_S
        )

    @patch("resim.sdk.test.os.unlink")
    @patch("resim.sdk.test.tempfile.NamedTemporaryFile")
    def test_exception_uploads_stacktrace_and_closes_with_error(
        self,
        mock_named_tmp_file: Any,
        mock_unlink: Any,
    ) -> None:
        stacktrace_path = self._write_file("execution_log_test.txt", b"")
        mock_tmp_file = MagicMock()
        mock_tmp_file.__enter__ = MagicMock(return_value=mock_tmp_file)
        mock_tmp_file.__exit__ = MagicMock(return_value=None)
        mock_tmp_file.name = stacktrace_path
        mock_named_tmp_file.return_value = mock_tmp_file

        test_exception = RuntimeError("test error")
        with self.assertRaises(RuntimeError):
            with Test(self.client, self.batch, TEST_NAME):
                raise test_exception

        # First log call should be the stacktrace (CONTAINER_LOG)
        self.assertEqual(self.mock_create_log.sync_detailed.call_count, 2)
        self.assertEqual(self._create_log_bodies()[0].log_type, LogType.CONTAINER_LOG)

        # close_job should be called with ERROR status and the error message
        self.assertEqual(self.put.call_count, 2)
        body = self._close_job_body()
        self.assertEqual(body.status, LightJobStatus.ERROR)
        self.assertEqual(body.error_message, repr(test_exception))

        # Temp file should be cleaned up
        mock_unlink.assert_called_once_with(stacktrace_path)

    def test_stacktrace_upload_failure_still_closes_job(self) -> None:
        # The test already failed; a broken stacktrace upload must not stop the
        # job from closing or hide the original exception.
        self.uploads.queue("stacktrace.log", [400])

        with self.assertRaises(RuntimeError):
            with Test(self.client, self.batch, TEST_NAME):
                raise RuntimeError("test error")

        self.mock_close_job.sync_detailed.assert_called_once()
        self.assertEqual(self._close_job_body().status, LightJobStatus.ERROR)


if __name__ == "__main__":
    unittest.main()
