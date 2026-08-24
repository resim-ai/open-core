import hashlib
import os
import tempfile
import time
import traceback
import httpx
from types import TracebackType
from typing import Any, Callable, Optional

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

__all__ = ["Test", "LogType"]

# Uploads go to a presigned URL over the open internet, so a transient failure
# is expected rather than exceptional.
_UPLOAD_TIMEOUT_SECONDS = 120.0

# Every call retries, but what counts as retryable depends on the call. A
# failure during the connect phase proves the request never reached the server,
# so any call can be replayed. Once the request is on the wire — a 5xx, a read
# timeout — it is unknown whether the server acted, so only calls that are safe
# to repeat are retried on those.
_API_ATTEMPTS = 4
_API_BACKOFF_SECONDS = 0.5
_CONNECT_ERRORS = (httpx.ConnectError, httpx.ConnectTimeout)
_TRANSPORT_ERROR = httpx.TransportError
_RETRYABLE_STATUS = (429, 500, 502, 503, 504)


def _call_api(
    call: Callable[[], Any],
    *,
    expected: int,
    describe: str,
    replayable: bool = False,
    succeeded_if: Optional[Callable[[Any], bool]] = None,
) -> Any:
    """Make an API call, retrying the failures that are safe to retry.

    Args:
        call: A zero-argument callable returning a ``sync_detailed`` response.
        expected: The status code that means success.
        describe: Used in the error message when every attempt fails.
        replayable: True when repeating the call cannot create anything twice.
            Non-replayable calls are retried only on connect-phase failures.
        succeeded_if: Given a failed response, returns True when it shows an
            earlier attempt already landed. Lets a replayed call recognise its
            own prior success instead of reporting a spurious failure.

    Returns:
        The successful response.

    Raises:
        Exception: If no attempt succeeded.
    """
    last_error: Optional[str] = None
    for attempt in range(_API_ATTEMPTS):
        try:
            response = call()
        except _CONNECT_ERRORS as e:
            # The connection never opened, so nothing was sent.
            last_error = repr(e)
        except _TRANSPORT_ERROR as e:
            last_error = repr(e)
            if not replayable:
                break
        else:
            if response.status_code == expected:
                return response
            if attempt > 0 and succeeded_if is not None and succeeded_if(response):
                return response
            last_error = f"{response.status_code}: {response.content!r}"
            if response.status_code not in _RETRYABLE_STATUS:
                break
            if not replayable and response.status_code != 503:
                # A 503 is a load balancer shedding load, so the request never
                # reached the application. Other 5xx may have been acted on.
                break
        if attempt + 1 < _API_ATTEMPTS:
            time.sleep(_API_BACKOFF_SECONDS * 2**attempt)

    raise Exception(f"{describe}. Last error {last_error}")


def _already_closed(response: Any) -> bool:
    """True when CloseJob reports the job was closed by an earlier attempt."""
    if response.status_code != 400:
        return False
    content = response.content
    if isinstance(content, bytes):
        return b"already closed" in content
    return "already closed" in str(content)


class Test(Emitter):
    def __init__(self, client: AuthenticatedClient, batch: Batch, name: str):
        self._client = client
        self._batch = batch
        self.name = name
        self._closed = False

        body = CreateJobForBatchInput(name=self.name)
        # Not replayable: the endpoint creates a new job every call, so a blind
        # retry would add a duplicate test to the batch.
        response = _call_api(
            lambda: create_job_for_batch.sync_detailed(
                self._batch.project_id,
                self._batch.id,
                client=self._client,
                body=body,
            ),
            expected=201,
            describe=f"failed to create job {self.name!r}",
        )
        if not response.parsed:
            raise Exception(f"failed to parse job creation response {response.content}")

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
    ) -> None:
        """Upload a local file as a log attachment for this test.

        Args:
            file_path: Path to the local file to upload.
            log_type: The log type classification. Defaults to None, which lets
                ReSim infer the log type from the file name.
            file_name: Override the filename used when uploading. Defaults to
                the basename of file_path.
        """
        if file_name is None:
            file_name = Path(file_path).name

        h = hashlib.sha256()
        with open(file_path, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                h.update(chunk)

        body = CreateJobLogInput(
            file_name=file_name,
            file_size=os.path.getsize(file_path),
            checksum=h.hexdigest(),
            log_type=log_type if log_type is not None else UNSET,
        )
        response = _call_api(
            lambda: create_job_log.sync_detailed(
                self._batch.project_id,
                self._batch.id,
                self._test.job_id,
                client=self._client,
                body=body,
            ),
            expected=201,
            describe=f"failed to create job log {file_name!r}",
        )

        log_output = response.parsed
        assert log_output is not None, "Failed to parse job log response"
        upload_url = log_output.upload_url
        upload_headers = {}
        if not isinstance(log_output.required_headers, Unset):
            upload_headers = log_output.required_headers.to_dict()

        with open(file_path, "rb") as f:
            body = f.read()

        # Replayable: a presigned PUT writes the same object every time, and a
        # dropped connection partway through a run would otherwise lose all of
        # the run's work.
        _call_api(
            lambda: httpx.put(
                upload_url,
                headers=upload_headers,
                content=body,
                timeout=_UPLOAD_TIMEOUT_SECONDS,
            ),
            expected=200,
            describe=f"failed to upload log {file_name}",
            replayable=True,
        )

    def attach_system_log(
        self,
        file_path: str,
        file_name: Optional[str] = None,
    ) -> None:
        """Upload a local file as a system log for this test.

        Args:
            file_path: Path to the local file to upload.
            file_name: Override the filename used when uploading. Defaults to
                the basename of file_path.
        """
        self.attach_log(file_path, log_type=LogType.SYSTEM_LOG, file_name=file_name)

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
                self.attach_log(
                    tmp_path, LogType.CONTAINER_LOG, file_name="stacktrace.log"
                )
            finally:
                os.unlink(tmp_path)
        self.close(
            status=status,
            error=repr(exc_value) if exc_value is not None else None,
        )

    def upload_emissions(self) -> None:
        """Finish the emissions file and upload it, leaving the job open.

        ``close`` calls this for you. Call it directly to upload a test's data
        before ending the job. Safe to call more than once.
        """
        if self.file is None:
            return
        Emitter.close(self)
        self.attach_log(
            str(self.output_path),
            LogType.EMISSIONS_LOG,
            file_name="emissions.resim.jsonl",
        )

    def close(
        self,
        status: LightJobStatus = LightJobStatus.SUCCEEDED,
        error: str | None = None,
    ) -> None:
        """Upload any remaining emissions and close the job, starting metrics.

        Safe to call more than once.
        """
        if self._closed:
            return
        self.upload_emissions()
        self._closed = True
        body = CloseJobInput(status=status)
        if error:
            body.error_message = error
        # Replayable: closing an already-closed job is refused with a 400 that
        # names the condition, so a retry can tell its own earlier success from
        # a real failure.
        _call_api(
            lambda: close_job.sync_detailed(
                self._batch.project_id,
                self._batch.id,
                self._test.job_id,
                client=self._client,
                body=body,
            ),
            expected=204,
            describe=f"failed to close test {self.name!r}",
            replayable=True,
            succeeded_if=_already_closed,
        )
