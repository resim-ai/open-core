import concurrent.futures
import hashlib
import logging
import os
import random
import tempfile
import threading
import time
import traceback
import httpx
from httpx import TransportError
from types import TracebackType
from typing import Callable, Optional, Sequence, TypeVar

from pathlib import Path
from resim.sdk.batch import Batch
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.types import UNSET, Unset
from resim.sdk.metrics.emissions import Emitter
from resim.sdk.client.api.light_batches import (
    create_job_for_batch,
    create_job_log,
    close_job,
)
from resim.sdk.client.models.close_job_input import CloseJobInput
from resim.sdk.client.models.light_job_status import LightJobStatus
from resim.sdk.client.models.create_job_for_batch_input import CreateJobForBatchInput
from resim.sdk.client.models.create_job_log_input import CreateJobLogInput
from resim.sdk.client.models.log_type import LogType

__all__ = ["Test", "LogType", "LogUploadError"]

logger = logging.getLogger(__name__)

# Number of threads used for background log uploads within a single test.
UPLOAD_WORKERS = 4
# Total tries per upload step, including the first one.
UPLOAD_ATTEMPTS = 3
# Seconds allowed for a single socket operation, not for the upload as a whole.
# The httpx default of 5 seconds is far too short for a large log.
UPLOAD_TIMEOUT_S = 300.0

# Responses worth another try: the server is busy or briefly unavailable.
_RETRYABLE_STATUS_CODES = frozenset({408, 425, 429, 500, 502, 503, 504})
_RETRY_BASE_DELAY_S = 0.5
_CHECKSUM_CHUNK_SIZE = 1024 * 1024

T = TypeVar("T")


class LogUploadError(Exception):
    """One or more log uploads failed after all tries.

    Attributes:
        failures: (file_name, exception) pairs, one per failed upload.
    """

    def __init__(self, failures: Sequence[tuple[str, BaseException]]):
        self.failures = list(failures)
        detail = "; ".join(f"{name}: {error!r}" for name, error in self.failures)
        super().__init__(f"{len(self.failures)} log upload(s) failed: {detail}")


class _RetryableUploadError(Exception):
    """Internal marker for a response that deserves another try."""


def _retry(
    operation: Callable[[], T],
    *,
    description: str,
    attempts: int,
) -> T:
    """Run an operation, retrying transient failures with exponential backoff.

    Args:
        operation: Callable to run. Must be safe to call more than once.
        description: Short label used in retry log messages.
        attempts: Total number of tries, including the first one.

    Returns:
        Whatever the operation returns.

    Raises:
        Exception: The last failure, if every try fails.
    """
    delay = _RETRY_BASE_DELAY_S
    for attempt in range(1, attempts + 1):
        try:
            return operation()
        except (_RetryableUploadError, TransportError) as error:
            if attempt >= attempts:
                raise
            # Jitter keeps a batch of parallel uploads from retrying in lockstep.
            sleep_for = delay + random.uniform(0.0, delay / 2)
            logger.warning(
                "%s failed (try %d of %d), retrying in %.1fs: %r",
                description,
                attempt,
                attempts,
                sleep_for,
                error,
            )
            time.sleep(sleep_for)
            delay *= 2
    raise AssertionError("unreachable")


class Test(Emitter):
    def __init__(self, client: AuthenticatedClient, batch: Batch, name: str):
        """Create a test (job) inside a batch.

        Args:
            client: Authenticated API client.
            batch: The batch this test belongs to.
            name: Display name for the test.
        """
        self._client = client
        self._batch = batch
        self.name = name
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._http_client: Optional[httpx.Client] = None
        self._pending: list[tuple[str, "concurrent.futures.Future[None]"]] = []
        self._pending_lock = threading.Lock()

        body = CreateJobForBatchInput(name=self.name)
        response = create_job_for_batch.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201 or not response.parsed:
            raise Exception(
                f"failed to create job {response.status_code}: {response.content}"
            )

        self._test = response.parsed
        emissions_file_path = Path(f"emissions_{self._test.job_id}.resim.jsonl")
        super().__init__(
            config_path=self._batch.metrics_config_path,
            output_path=emissions_file_path,
        )

    def attach_log(
        self,
        file_path: str,
        log_type: Optional[LogType] = None,
        file_name: Optional[str] = None,
        *,
        wait: bool = False,
    ) -> None:
        """Upload a local file as a log attachment for this test.

        The upload runs on a background pool, so several uploads overlap with
        each other and with the rest of the test. Closing the test waits for
        every background upload and reports the ones that failed. Transient
        failures are retried either way.

        Args:
            file_path: Path to the local file to upload. Unless wait is True,
                the file must still exist, complete and unchanged, when the
                upload runs.
            log_type: The log type classification. Defaults to None, which lets
                ReSim infer the log type from the file name.
            file_name: Override the filename used when uploading. Defaults to
                the basename of file_path.
            wait: Block until this upload finishes instead of backgrounding it.
                Use it when the file is about to be deleted or rewritten, or
                when the failure has to surface at the call site.

        Raises:
            Exception: If the upload failed and wait is True. Background
                failures are raised by close() as a LogUploadError instead.
        """
        if wait:
            self._upload(file_path, log_type, file_name)
            return

        name = file_name if file_name is not None else Path(file_path).name
        future = self._pool().submit(self._upload, file_path, log_type, file_name)
        with self._pending_lock:
            self._pending.append((name, future))

    def attach_system_log(
        self,
        file_path: str,
        file_name: Optional[str] = None,
        *,
        wait: bool = False,
    ) -> None:
        """Upload a local file as a system log for this test.

        Args:
            file_path: Path to the local file to upload.
            file_name: Override the filename used when uploading. Defaults to
                the basename of file_path.
            wait: Whether to block until the upload finishes. See attach_log.
        """
        self.attach_log(
            file_path, log_type=LogType.SYSTEM_LOG, file_name=file_name, wait=wait
        )

    def _pool(self) -> concurrent.futures.ThreadPoolExecutor:
        """Return this test's upload pool, creating it on first use."""
        with self._pending_lock:
            if self._executor is None:
                self._executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=UPLOAD_WORKERS,
                    thread_name_prefix="resim-log-upload",
                )
            return self._executor

    def _http(self) -> httpx.Client:
        """Return this test's HTTP client, creating it on first use.

        One client per test keeps the connection to the object store alive
        between uploads. Without it every log pays a fresh TLS handshake, which
        dominates the time spent on a small log.
        """
        with self._pending_lock:
            if self._http_client is None:
                self._http_client = httpx.Client(timeout=UPLOAD_TIMEOUT_S)
            return self._http_client

    def _upload(
        self,
        file_path: str,
        log_type: Optional[LogType],
        file_name: Optional[str],
    ) -> None:
        """Register a log with the API and upload its contents."""
        if file_name is None:
            file_name = Path(file_path).name

        file_size = os.path.getsize(file_path)
        checksum = _checksum(file_path)

        upload_url, upload_headers = _retry(
            lambda: self._create_log(file_name, file_size, checksum, log_type),
            description=f"creating job log {file_name}",
            attempts=UPLOAD_ATTEMPTS,
        )

        _retry(
            lambda: self._put_file(
                file_path, file_name, file_size, upload_url, upload_headers
            ),
            description=f"uploading log {file_name}",
            attempts=UPLOAD_ATTEMPTS,
        )

    def _create_log(
        self,
        file_name: str,
        file_size: int,
        checksum: str,
        log_type: Optional[LogType],
    ) -> tuple[str, dict[str, str]]:
        """Register a log with the API and return its upload URL and headers."""
        body = CreateJobLogInput(
            file_name=file_name,
            file_size=file_size,
            checksum=checksum,
            log_type=log_type if log_type is not None else UNSET,
        )
        response = create_job_log.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            self._test.job_id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201:
            message = (
                f"failed to create job log {response.status_code}: {response.content}"
            )
            if response.status_code in _RETRYABLE_STATUS_CODES:
                raise _RetryableUploadError(message)
            raise Exception(message)

        log_output = response.parsed
        assert log_output is not None, "Failed to parse job log response"
        upload_headers: dict[str, str] = {}
        if not isinstance(log_output.required_headers, Unset):
            upload_headers = log_output.required_headers.to_dict()
        return log_output.upload_url, upload_headers

    def _put_file(
        self,
        file_path: str,
        file_name: str,
        file_size: int,
        upload_url: str,
        upload_headers: dict[str, str],
    ) -> None:
        # Stream from the file handle instead of reading it into memory. The
        # explicit Content-Length keeps httpx from using chunked encoding, which
        # object stores reject. The handle is reopened on every try so a retry
        # starts from the beginning of the file.
        headers = dict(upload_headers)
        headers["Content-Length"] = str(file_size)
        with open(file_path, "rb") as f:
            response = self._http().put(upload_url, headers=headers, content=f)

        if response.status_code != 200:
            message = (
                f"failed to upload log {file_name}. "
                f"Got response {response.status_code}: {response.content!r}"
            )
            if response.status_code in _RETRYABLE_STATUS_CODES:
                raise _RetryableUploadError(message)
            raise Exception(message)

    def __enter__(self) -> "Test":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        """
        Close the test, and trigger the metrics phase of the pipeline.
        If an exception occurred, upload its traceback as an EXECUTION_LOG.
        """
        status = LightJobStatus.SUCCEEDED
        if exc_type is not None:
            status = LightJobStatus.ERROR
            with tempfile.NamedTemporaryFile(
                mode="w",
                suffix=".txt",
                prefix="execution_log_",
                delete=False,
            ) as f:
                traceback.print_exception(exc_type, exc_value, tb, file=f)
                tmp_path = f.name
            try:
                # wait=True: the temp file is deleted as soon as this returns.
                self.attach_log(
                    tmp_path,
                    LogType.CONTAINER_LOG,
                    file_name="stacktrace.log",
                    wait=True,
                )
            except Exception as upload_error:
                # The test already failed. Closing the job and reporting the
                # original exception matters more than the stacktrace upload.
                logger.warning("failed to upload stacktrace: %r", upload_error)
            finally:
                os.unlink(tmp_path)
        self.close(
            status=status,
            error=repr(exc_value) if exc_value is not None else None,
        )

    def close(
        self,
        status: LightJobStatus = LightJobStatus.SUCCEEDED,
        error: str | None = None,
    ) -> None:
        """Finish the test: flush emissions, drain uploads, and close the job.

        Waits for every background upload. If any of them failed, the job is
        closed with an ERROR status and the failures are raised once the job
        is closed.

        Args:
            status: Status to close the job with.
            error: Optional error message to record on the job.

        Raises:
            LogUploadError: If any log upload failed after all tries.
        """
        if self.file is None:
            return
        super().close()

        self.attach_log(
            str(self.output_path),
            LogType.EMISSIONS_LOG,
            file_name="emissions.resim.jsonl",
        )
        failures = self._drain_uploads()

        upload_error: Optional[LogUploadError] = None
        if failures:
            upload_error = LogUploadError(failures)
            if status == LightJobStatus.SUCCEEDED:
                status = LightJobStatus.ERROR
                error = str(upload_error)

        body = CloseJobInput(status=status)
        if error:
            body.error_message = error
        response = close_job.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            self._test.job_id,
            client=self._client,
            body=body,
        )
        if response.status_code != 204:
            raise Exception(
                f"failed to close test. Expected 204 response, got {response.status_code} instead"
            )
        if upload_error is not None:
            raise upload_error

    def _drain_uploads(self) -> list[tuple[str, BaseException]]:
        """Wait for every background upload and return the ones that failed."""
        with self._pending_lock:
            pending = self._pending
            self._pending = []
            executor = self._executor
            self._executor = None

        failures = _collect_failures(pending)
        if executor is not None:
            executor.shutdown(wait=True)

        # Only now can the client be closed: a worker that had not started yet
        # would otherwise create a replacement after this ran.
        with self._pending_lock:
            http_client = self._http_client
            self._http_client = None
        if http_client is not None:
            http_client.close()
        return failures


def _collect_failures(
    pending: Sequence[tuple[str, "concurrent.futures.Future[None]"]],
) -> list[tuple[str, BaseException]]:
    """Wait for each future and return (file_name, exception) for the failures."""
    failures: list[tuple[str, BaseException]] = []
    for name, future in pending:
        try:
            future.result()
        except Exception as error:
            failures.append((name, error))
    return failures


def _checksum(file_path: str) -> str:
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(_CHECKSUM_CHUNK_SIZE), b""):
            h.update(chunk)
    return h.hexdigest()
