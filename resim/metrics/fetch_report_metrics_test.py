import unittest
import uuid
import asyncio
from resim_python_client.client import AuthenticatedClient
import datetime
import resim.metrics.fetch_report_metrics as frp


from resim_python_client.models.report import Report
from resim_python_client.models.list_batches_output import ListBatchesOutput
from resim_python_client.models.metric_status import MetricStatus
from resim_python_client.models.report_status import ReportStatus
from resim_python_client.models.list_jobs_output import ListJobsOutput




class AsyncFetchAllPagesTest(unittest.IsolatedAsyncioTestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.report = Report(
            associated_account= "",
            branch_id= "",
            creation_timestamp= datetime.datetime.max,
            end_timestamp= datetime.datetime.max,
            last_updated_timestamp= datetime.datetime.max,
            metrics_build_id= "",
            metrics_status= MetricStatus.PASSED,
            name= "",
            org_id= "",
            output_location= "",
            project_id= "",
            report_id= "",
            respect_revision_boundary=False,
            start_timestamp= datetime.datetime.min,
            status= ReportStatus.SUCCEEDED,
            status_history=[],
            test_suite_id= "",
            test_suite_revision=0,
            user_id= "",
        )
            

        
    async def test_fetch_batches_for_report(self):
        self.assertTrue(True);

if __name__ == "__main__":
    unittest.main()
