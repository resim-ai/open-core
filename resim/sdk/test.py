import hashlib
import os
import httpx
import uuid
from types import TracebackType
from typing import Any, Optional
from contextlib import AbstractContextManager
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
from resim.sdk.client.models.create_job_for_batch_input import CreateJobForBatchInput
from resim.sdk.client.models.create_job_log_input import CreateJobLogInput
from resim.sdk.client.models.log_type import LogType


class Test(AbstractContextManager):
    def __init__(self, client: AuthenticatedClient, batch: Batch, name: str):
        self._client = client
        # internal id used to prevent clobbering data from other tests possibly
        # running concurrently
        self._test_id = uuid.uuid4()
        self._batch = batch
        self.name = name

        self._emissions_output_path = Path(f"emissions_{self._test_id}.resim.jsonl")
        self._emitter = Emitter(
            config_path=batch.metrics_config_path,
            output_path=self._emissions_output_path,
        )

    def __enter__(self) -> "Test":
        body = CreateJobForBatchInput(name=self.name)
        response = create_job_for_batch.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201:
            raise Exception(
                f"failed to create job {response.status_code}: {response.content}"
            )

        self._test = response.parsed

        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.__upload_emissions_log()
        self.__close_test()

    def __upload_emissions_log(self) -> None:
        emissions_file_name = str(self._emissions_output_path)

        h = hashlib.sha256()
        with open(emissions_file_name, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                h.update(chunk)

        body = CreateJobLogInput(
            file_name="emissions.resim.jsonl",
            file_size=os.path.getsize(emissions_file_name),
            checksum=h.hexdigest(),
            log_type=LogType.EMISSIONS_LOG,
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

        with open(emissions_file_name, "rb") as f:
            r = httpx.put(upload_url, headers=upload_headers, content=f.read())
        if r.status_code != 200:
            raise Exception(f"failed to upload emissions log for test {self.name}")

    def __close_test(self) -> None:
        response = close_job.sync_detailed(
            self._batch.project_id,
            self._batch.id,
            self._test.job_id,
            client=self._client,
        )
        if response.status_code != 204:
            raise Exception(
                f"failed to close test. Expected 204 response, got {response.status_code} instead"
            )

    def emit(
        self, topic_name: str, data: dict[str, Any], timestamp: Optional[int] = None
    ) -> None:
        self._emitter.emit(topic_name, data, timestamp)
