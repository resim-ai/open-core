import unittest
from typing import Any
from unittest.mock import MagicMock, patch

from resim.sdk.batch import Batch

PROJECT_ID = "project-123"
BRANCH_NAME = "main"
BRANCH_ID = "branch-456"
BATCH_ID = "batch-789"
BATCH_FRIENDLY_NAME = "friendly-batch"


class BatchTest(unittest.TestCase):
    @patch("resim.sdk.batch.close_batch")
    @patch("resim.sdk.batch.create_light_batch")
    @patch("resim.sdk.batch.list_branches_for_project")
    def test_batch(
        self, mock_list_branches: Any, mock_create_batch: Any, mock_close_batch: Any
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

        with Batch(mock_client, PROJECT_ID, BRANCH_NAME) as batch:
            self.assertEqual(batch.project_id, PROJECT_ID)
            self.assertEqual(batch.id, BATCH_ID)
            self.assertEqual(batch.friendly_name, BATCH_FRIENDLY_NAME)

        mock_list_branches.sync.assert_called_once_with(
            PROJECT_ID, client=mock_client, name=BRANCH_NAME
        )
        mock_create_batch.sync_detailed.assert_called_once()
        mock_close_batch.sync_detailed.assert_called_once_with(
            PROJECT_ID, BATCH_ID, client=mock_client
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
            PROJECT_ID,
            BRANCH_NAME,
            metrics_config_path=config_path,
        ) as batch:
            self.assertEqual(batch.id, BATCH_ID)

        mock_metrics.sync_config.assert_called_once_with(
            mock_client,
            PROJECT_ID,
            BRANCH_NAME,
            config_path=config_path,
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

        with Batch(mock_client, PROJECT_ID, BRANCH_NAME):
            pass

        mock_metrics.sync_config.assert_not_called()


if __name__ == "__main__":
    unittest.main()
