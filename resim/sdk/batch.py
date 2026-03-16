from contextlib import AbstractContextManager
from types import TracebackType
from resim.sdk.client.api.projects import list_branches_for_project
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.api.light_batches import create_light_batch, close_batch
from resim.sdk.client.models import Batch as ApiBatch
from resim.sdk.client.models.light_batch_input import LightBatchInput
from resim.sdk.bff_client import metrics


class Batch(AbstractContextManager):
    def __init__(
        self,
        client: AuthenticatedClient,
        project_id: str,
        branch: str,
        name: str | None = None,
        version: str | None = None,
        metrics_set_name: str | None = None,
        metrics_config_path: str | None = None,
    ):
        self._client = client
        self.project_id = project_id
        self.batch: ApiBatch | None = None
        self._name = name
        self._version = version
        self._branch = branch
        self._metrics_set_name = metrics_set_name
        self.metrics_config_path = metrics_config_path

    def __enter__(self) -> "Batch":
        if self.metrics_config_path:
            metrics.sync_config(
                self._client,
                self.project_id,
                self._branch,
                config_path=self.metrics_config_path,
            )

        self._branch_id = self.__get_branch_id(self._branch)

        body = LightBatchInput(branch_id=self._branch_id)
        if self._name:
            body.batch_name = self._name
        if self._metrics_set_name:
            body.metrics_set_name = self._metrics_set_name
        if self._version:
            body.version = self._version
        response = create_light_batch.sync_detailed(
            self.project_id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201:
            raise Exception(
                f"Failed to create batch: {response.status_code} {response.content}"
            )
        self.batch = response.parsed

        return self

    @property
    def id(self) -> str:
        assert self.batch is not None and self.batch.batch_id, "Batch not created"
        return str(self.batch.batch_id)

    @property
    def friendly_name(self) -> str:
        assert self.batch is not None and self.batch.friendly_name, "Batch not created"
        return str(self.batch.friendly_name)

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        response = close_batch.sync_detailed(
            self.project_id,
            self.id,
            client=self._client,
        )
        if response.status_code != 204:
            raise Exception(
                f"Failed to close batch. Expected 204 response, got {response.status_code} instead"
            )

    def __get_branch_id(self, name: str) -> str:
        branches = list_branches_for_project.sync(
            self.project_id, client=self._client, name=name
        )
        assert branches is not None, "failed to fetch branches"

        # Since we're filtering branches by name, we're not bothering paging thru
        # all results.
        for branch in branches.branches:
            if branch.name == name:
                return str(branch.branch_id)

        raise Exception(
            f"branch {name} does not exist. Did you sync your metrics config?"
        )
