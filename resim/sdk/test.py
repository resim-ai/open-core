import hashlib
import os
import tempfile
import traceback
import httpx
from types import TracebackType
from typing import Any, Optional, Union

from pathlib import Path
from resim.sdk.batch import Batch
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.types import Unset
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


class Test(Emitter):
    def __init__(self, client: AuthenticatedClient, batch: Batch, name: str):
        self._client = client
        self._batch = batch
        self.name = name

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
        log_type: LogType = LogType.OTHER_LOG,
        file_name: Optional[str] = None,
    ) -> None:
        """Upload a local file as a log attachment for this test.

        Args:
            file_path: Path to the local file to upload.
            log_type: The log type classification. Defaults to LogType.OTHER_LOG.
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
            log_type=log_type,
        )
        response = create_job_log.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            self._test.job_id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201:
            raise Exception(
                f"failed to create job log {response.status_code}: {response.content}"
            )

        log_output = response.parsed
        assert log_output is not None, "Failed to parse job log response"
        upload_url = log_output.upload_url
        upload_headers = {}
        if not isinstance(log_output.required_headers, Unset):
            upload_headers = log_output.required_headers.to_dict()

        with open(file_path, "rb") as f:
            r = httpx.put(upload_url, headers=upload_headers, content=f.read())

        if r.status_code != 200:
            raise Exception(
                f"failed to upload log {file_name}. Got response {r.status_code}: {r.content}"
            )

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
            error="Test threw an exception during run. See stacktrace under logs.",
        )

    def close(self, status: LightJobStatus = LightJobStatus.SUCCEEDED, error: str | Unset = Unset()) -> None:  # type: ignore[override]
        if self.file is None:
            return
        super().close()
        self.attach_log(
            str(self.output_path),
            LogType.EMISSIONS_LOG,
            file_name="emissions.resim.jsonl",
        )
        body = CloseJobInput(status=status, error_message=error)
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
