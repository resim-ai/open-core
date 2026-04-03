import unittest
from typing import Any
from unittest.mock import MagicMock, patch
from resim.sdk.client.models.light_batch_input import LightBatchInput
from resim.sdk.batch import Batch

PROJECT_ID = "project-123"
PROJECT_NAME = "my-project"
BRANCH_NAME = "main"
BRANCH_ID = "branch-456"
BATCH_ID = "batch-789"
BATCH_FRIENDLY_NAME = "friendly-batch"
SYSTEM_NAME = "my-system"
SYSTEM_ID = "system-abc"
TEST_SUITE_NAME = "my-test-suite"
TEST_SUITE_ID = "suite-def"
TEST_SUITE_REVISION = 3


class BatchTest(unittest.TestCase):
    def _make_standard_mocks(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
    ) -> None:
        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        with Batch(
            mock_client,
            BRANCH_NAME,
            project_id=PROJECT_ID,
            name="hello-world",
            metrics_set_name="metrics",
            version="1.0.2",
        ) as batch:
            self.assertEqual(batch.project_id, PROJECT_ID)
            self.assertEqual(batch.id, BATCH_ID)
            self.assertEqual(batch.friendly_name, BATCH_FRIENDLY_NAME)

        mock_list_branches.sync.assert_called_once_with(
            PROJECT_ID, client=mock_client, name=BRANCH_NAME
        )
        mock_create_batch.sync_detailed.assert_called_once_with(
            PROJECT_ID,
            client=mock_client,
            body=LightBatchInput(
                branch_id=BRANCH_ID,
                batch_name="hello-world",
                metrics_set_name="metrics",
                version="1.0.2",
            ),
        )
        mock_close_batch.sync_detailed.assert_called_once_with(
            PROJECT_ID, BATCH_ID, client=mock_client
        )

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    @patch("resim.sdk.batch.list_projects")
    def test_batch_resolves_project_by_name(
        self,
        mock_list_projects: Any,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_project = MagicMock()
        mock_project.name = PROJECT_NAME
        mock_project.project_id = PROJECT_ID
        mock_list_projects.sync.return_value = MagicMock(
            projects=[mock_project], next_page_token=None
        )

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        with Batch(
            mock_client,
            BRANCH_NAME,
            project_name=PROJECT_NAME,
        ) as batch:
            self.assertEqual(batch.project_id, PROJECT_ID)
            self.assertEqual(batch.id, BATCH_ID)

        mock_list_projects.sync.assert_called_once_with(client=mock_client)

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    @patch("resim.sdk.batch.list_projects")
    def test_batch_resolves_project_by_name_across_pages(
        self,
        mock_list_projects: Any,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_project = MagicMock()
        mock_project.name = PROJECT_NAME
        mock_project.project_id = PROJECT_ID
        page_token = "next-page-token"
        mock_list_projects.sync.side_effect = [
            MagicMock(projects=[], next_page_token=page_token),
            MagicMock(projects=[mock_project], next_page_token=None),
        ]

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        with Batch(mock_client, BRANCH_NAME, project_name=PROJECT_NAME) as batch:
            self.assertEqual(batch.project_id, PROJECT_ID)

        self.assertEqual(mock_list_projects.sync.call_count, 2)
        mock_list_projects.sync.assert_any_call(client=mock_client)
        mock_list_projects.sync.assert_any_call(
            client=mock_client, page_token=page_token
        )

    @patch("resim.sdk.batch.list_projects")
    def test_batch_raises_if_project_name_not_found(
        self, mock_list_projects: Any
    ) -> None:
        mock_client = MagicMock()
        other_project = MagicMock()
        other_project.name = "other-project"
        mock_list_projects.sync.return_value = MagicMock(
            projects=[other_project], next_page_token=None
        )

        with self.assertRaises(Exception, msg=f"project {PROJECT_NAME!r} not found"):
            with Batch(mock_client, BRANCH_NAME, project_name=PROJECT_NAME):
                pass

    def test_batch_raises_if_neither_project_id_nor_name(self) -> None:
        mock_client = MagicMock()
        with self.assertRaises(ValueError):
            Batch(mock_client, BRANCH_NAME)

    def test_batch_raises_if_both_project_id_and_name(self) -> None:
        mock_client = MagicMock()
        with self.assertRaises(ValueError):
            Batch(
                mock_client,
                BRANCH_NAME,
                project_id=PROJECT_ID,
                project_name=PROJECT_NAME,
            )

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_syncs_metrics_config(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        config_path = "/fake/path/config.resim.yml"
        with Batch(
            mock_client,
            BRANCH_NAME,
            project_id=PROJECT_ID,
            metrics_config_path=config_path,
        ) as batch:
            self.assertEqual(batch.id, BATCH_ID)

        mock_metrics.sync_config.assert_called_once_with(
            mock_client,
            PROJECT_ID,
            BRANCH_NAME,
            config_path=config_path,
            templates_path=".resim/metrics/templates",
        )

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_syncs_multiple_metrics_config_paths(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        config_paths = ("/fake/a.resim.yml", "/fake/b.resim.yml")
        with Batch(
            mock_client,
            BRANCH_NAME,
            project_id=PROJECT_ID,
            metrics_config_path=config_paths,
        ) as batch:
            self.assertEqual(batch.id, BATCH_ID)

        mock_metrics.sync_config.assert_called_once_with(
            mock_client,
            PROJECT_ID,
            BRANCH_NAME,
            config_path=config_paths,
            templates_path=".resim/metrics/templates",
        )

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_skips_metrics_sync_when_no_config(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()

        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 204
        mock_close_batch.sync_detailed.return_value = mock_close_response

        with Batch(
            mock_client, BRANCH_NAME, project_id=PROJECT_ID, metrics_config_path=None
        ):
            pass

        mock_metrics.sync_config.assert_not_called()

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_raises_if_branch_is_missing(
        self, mock_list_branches: Any, _mock_metrics: Any
    ) -> None:
        mock_client = MagicMock()
        mock_list_branches.sync.return_value = MagicMock(branches=[])

        with self.assertRaises(Exception, msg=f"branch {BRANCH_NAME} does not exist"):
            with Batch(mock_client, BRANCH_NAME, project_id=PROJECT_ID):
                pass

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_systems")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_with_system(
        self,
        mock_list_branches: Any,
        mock_list_systems: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()
        self._make_standard_mocks(
            mock_list_branches, mock_create_batch, mock_close_batch
        )

        mock_system = MagicMock()
        mock_system.name = SYSTEM_NAME
        mock_system.system_id = SYSTEM_ID
        mock_list_systems.sync.return_value = MagicMock(systems=[mock_system])

        with Batch(mock_client, BRANCH_NAME, project_id=PROJECT_ID, system=SYSTEM_NAME):
            pass

        mock_list_systems.sync.assert_called_once_with(
            PROJECT_ID, client=mock_client, name=SYSTEM_NAME
        )
        body = mock_create_batch.sync_detailed.call_args.kwargs["body"]
        self.assertEqual(body.system_id, SYSTEM_ID)

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_test_suites")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_with_test_suite(
        self,
        mock_list_branches: Any,
        mock_list_test_suites: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()
        self._make_standard_mocks(
            mock_list_branches, mock_create_batch, mock_close_batch
        )

        mock_suite = MagicMock()
        mock_suite.name = TEST_SUITE_NAME
        mock_suite.test_suite_id = TEST_SUITE_ID
        mock_list_test_suites.sync.return_value = MagicMock(test_suites=[mock_suite])

        with Batch(
            mock_client, BRANCH_NAME, project_id=PROJECT_ID, test_suite=TEST_SUITE_NAME
        ):
            pass

        mock_list_test_suites.sync.assert_called_once_with(
            PROJECT_ID, client=mock_client, name=TEST_SUITE_NAME
        )
        body = mock_create_batch.sync_detailed.call_args.kwargs["body"]
        self.assertEqual(body.test_suite_id, TEST_SUITE_ID)

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_test_suites")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_with_test_suite_and_revision(
        self,
        mock_list_branches: Any,
        mock_list_test_suites: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()
        self._make_standard_mocks(
            mock_list_branches, mock_create_batch, mock_close_batch
        )

        mock_suite = MagicMock()
        mock_suite.name = TEST_SUITE_NAME
        mock_suite.test_suite_id = TEST_SUITE_ID
        mock_list_test_suites.sync.return_value = MagicMock(test_suites=[mock_suite])

        with Batch(
            mock_client,
            BRANCH_NAME,
            project_id=PROJECT_ID,
            test_suite=TEST_SUITE_NAME,
            test_suite_revision=TEST_SUITE_REVISION,
        ):
            pass

        body = mock_create_batch.sync_detailed.call_args.kwargs["body"]
        self.assertEqual(body.test_suite_id, TEST_SUITE_ID)
        self.assertEqual(body.test_suite_revision, TEST_SUITE_REVISION)

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_systems")
    @patch("resim.sdk.batch.list_test_suites")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_with_system_and_test_suite(
        self,
        mock_list_branches: Any,
        mock_list_test_suites: Any,
        mock_list_systems: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()
        self._make_standard_mocks(
            mock_list_branches, mock_create_batch, mock_close_batch
        )

        mock_system = MagicMock()
        mock_system.name = SYSTEM_NAME
        mock_system.system_id = SYSTEM_ID
        mock_list_systems.sync.return_value = MagicMock(systems=[mock_system])

        mock_suite = MagicMock()
        mock_suite.name = TEST_SUITE_NAME
        mock_suite.test_suite_id = TEST_SUITE_ID
        mock_list_test_suites.sync.return_value = MagicMock(test_suites=[mock_suite])

        with Batch(
            mock_client,
            BRANCH_NAME,
            project_id=PROJECT_ID,
            system=SYSTEM_NAME,
            test_suite=TEST_SUITE_NAME,
        ):
            pass

        body = mock_create_batch.sync_detailed.call_args.kwargs["body"]
        self.assertEqual(body.system_id, SYSTEM_ID)
        self.assertEqual(body.test_suite_id, TEST_SUITE_ID)

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.list_systems")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_raises_if_system_not_found(
        self, mock_list_branches: Any, mock_list_systems: Any, _mock_metrics: Any
    ) -> None:
        mock_client = MagicMock()
        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        other_system = MagicMock()
        other_system.name = "other-system"
        mock_list_systems.sync.return_value = MagicMock(systems=[other_system])

        with self.assertRaises(Exception, msg=f"system {SYSTEM_NAME!r} not found"):
            with Batch(
                mock_client, BRANCH_NAME, project_id=PROJECT_ID, system=SYSTEM_NAME
            ):
                pass

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.list_test_suites")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_raises_if_test_suite_not_found(
        self, mock_list_branches: Any, mock_list_test_suites: Any, _mock_metrics: Any
    ) -> None:
        mock_client = MagicMock()
        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        other_suite = MagicMock()
        other_suite.name = "other-suite"
        mock_list_test_suites.sync.return_value = MagicMock(test_suites=[other_suite])

        with self.assertRaises(
            Exception, msg=f"test suite {TEST_SUITE_NAME!r} not found"
        ):
            with Batch(
                mock_client,
                BRANCH_NAME,
                project_id=PROJECT_ID,
                test_suite=TEST_SUITE_NAME,
            ):
                pass

    def test_batch_raises_if_revision_without_test_suite(self) -> None:
        mock_client = MagicMock()
        with self.assertRaises(ValueError):
            Batch(
                mock_client,
                BRANCH_NAME,
                project_id=PROJECT_ID,
                test_suite_revision=TEST_SUITE_REVISION,
            )

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_raises_if_create_fails(
        self, mock_list_branches: Any, mock_create_batch: Any, _mock_metrics: Any
    ) -> None:
        mock_client = MagicMock()
        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 500
        mock_create_batch.sync_detailed.return_value = mock_response

        with self.assertRaises(Exception, msg="Failed to create batch"):
            with Batch(mock_client, BRANCH_NAME, project_id=PROJECT_ID):
                pass

    @patch("resim.sdk.batch.metrics")
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch_raises_if_close_fails(
        self,
        mock_list_branches: Any,
        mock_create_batch: Any,
        mock_close_batch: Any,
        _mock_metrics: Any,
    ) -> None:
        mock_client = MagicMock()
        mock_branch = MagicMock()
        mock_branch.name = BRANCH_NAME
        mock_branch.branch_id = BRANCH_ID
        mock_list_branches.sync.return_value = MagicMock(branches=[mock_branch])

        mock_response = MagicMock()
        mock_response.status_code = 201
        mock_response.parsed.batch_id = BATCH_ID
        mock_response.parsed.friendly_name = BATCH_FRIENDLY_NAME
        mock_create_batch.sync_detailed.return_value = mock_response

        mock_close_response = MagicMock()
        mock_close_response.status_code = 500
        mock_close_batch.sync_detailed.return_value = mock_close_response

        with self.assertRaises(Exception, msg="Failed to close batch"):
            with Batch(mock_client, BRANCH_NAME, project_id=PROJECT_ID):
                pass


if __name__ == "__main__":
    unittest.main()
