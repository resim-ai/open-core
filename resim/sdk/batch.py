from collections.abc import Sequence
from contextlib import AbstractContextManager
from types import TracebackType
from typing import Union
from resim.sdk.client.api.projects import list_branches_for_project, list_projects
from resim.sdk.client.api.systems import list_systems
from resim.sdk.client.api.test_suites import list_test_suites
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.api.light_batches import create_light_batch, close_batch
from resim.sdk.client.models import Batch as ApiBatch
from resim.sdk.client.models.light_batch_input import LightBatchInput
from resim.sdk.bff_client import metrics


class Batch(AbstractContextManager):
    """Light batch lifecycle; optionally syncs metrics config before creation.

    ``metrics_config_path`` may be a single file path or a sequence of paths. Multiple
    files are merged (each topic must appear in only one file) before upload.
    """

    def __init__(
        self,
        client: AuthenticatedClient,
        branch: str,
        project_id: str | None = None,
        project_name: str | None = None,
        name: str | None = None,
        version: str | None = None,
        metrics_set_name: str | None = None,
        metrics_config_path: Union[str, Sequence[str], None] = None,
        templates_path: str | None = None,
        system: str | None = None,
        test_suite: str | None = None,
        test_suite_revision: int | None = None,
    ):
        """Create a Batch context manager.

        Exactly one of project_id or project_name must be provided. If project_name
        is given, the project ID is resolved for you.

        Args:
            client: Authenticated API client.
            branch: Name of the branch to create the batch on.
            project_id: UUID of the project. Mutually exclusive with project_name.
            project_name: Human-readable name of the project. Mutually exclusive with project_id.
            name: Optional display name for the batch.
            version: Optional version string to associate with the batch, such as a commit SHA or semver.
            metrics_set_name: Optional name of the metrics set to use, from your config file.
            metrics_config_path: Optional path to a metrics config file to sync before creating the batch.
            templates_path: Optional path to a directory of custom metric ``.liquid`` template files to sync alongside
                the metrics config. Ignored if ``metrics_config_path`` is not set.
            system: Optional name of the system to use for the build.
            test_suite: Optional name of the test suite to attach the batch to.
            test_suite_revision: Optional revision of the test suite to pin to. Requires test_suite.
        """
        if project_id is None and project_name is None:
            raise ValueError("Either project_id or project_name must be provided")
        if project_id is not None and project_name is not None:
            raise ValueError("Only one of project_id or project_name may be provided")
        if test_suite_revision is not None and test_suite is None:
            raise ValueError("test_suite_revision requires test_suite to be provided")
        self._client = client
        self.project_id: str | None = project_id
        self._project_name = project_name
        self.batch: ApiBatch | None = None
        self._name = name
        self._version = version
        self._branch = branch
        self._metrics_set_name = metrics_set_name
        self.metrics_config_path = metrics_config_path
        self._templates_path = templates_path
        self._system = system
        self._test_suite = test_suite
        self._test_suite_revision = test_suite_revision

    def __enter__(self) -> "Batch":
        if self.project_id is None and self._project_name is not None:
            self.project_id = self.__resolve_project_id(self._project_name)

        if self.metrics_config_path:
            metrics.sync_config(
                self._client,
                self.project_id,
                self._branch,
                config_path=self.metrics_config_path,
                templates_path=self._templates_path,
            )

        self._branch_id = self.__get_branch_id(self._branch)

        body = LightBatchInput(branch_id=self._branch_id)
        if self._name:
            body.batch_name = self._name
        if self._metrics_set_name:
            body.metrics_set_name = self._metrics_set_name
        if self._version:
            body.version = self._version
        if self._system:
            body.system_id = self.__get_system_id(self._system)
        if self._test_suite:
            body.test_suite_id = self.__get_test_suite_id(self._test_suite)
            if self._test_suite_revision is not None:
                body.test_suite_revision = self._test_suite_revision

        response = create_light_batch.sync_detailed(
            self.project_id,
            client=self._client,
            body=body,
        )
        if response.status_code != 201:
            raise Exception(
                f"Failed to create batch. Received a {response.status_code} response from API: {response.content}"
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
                f"Failed to close batch. Expected 204 response, got {response.status_code} instead: {response.content}"
            )

    def __resolve_project_id(self, name: str) -> str:
        page_token: str | None = None
        while True:
            kwargs = {"client": self._client}
            if page_token:
                kwargs["page_token"] = page_token
            response = list_projects.sync(**kwargs)
            assert response is not None, "failed to fetch projects"
            for project in response.projects:
                if project.name == name:
                    return str(project.project_id)
            page_token = response.next_page_token
            if not page_token:
                break
        raise Exception(f"project {name!r} not found")

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

    def __get_system_id(self, name: str) -> str:
        response = list_systems.sync(self.project_id, client=self._client, name=name)
        assert response is not None, "failed to fetch systems"
        for system in response.systems:
            if system.name == name:
                return str(system.system_id)
        raise Exception(f"system {name!r} not found")

    def __get_test_suite_id(self, name: str) -> str:
        response = list_test_suites.sync(
            self.project_id, client=self._client, name=name
        )
        assert response is not None, "failed to fetch test suites"
        for test_suite in response.test_suites:
            if test_suite.name == name:
                return str(test_suite.test_suite_id)
        raise Exception(f"test suite {name!r} not found")
