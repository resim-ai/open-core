import uuid
import asyncio
from resim_python_client.client import AuthenticatedClient
import resim.metrics.fetch_report_metrics as frp

token=""
api_url= "https://api.resim.ai/v1/"
project_id=uuid.UUID("72d84a89-e36d-463d-b875-dc2776f36a5f")
report_id=uuid.UUID("64015d7b-caee-4ff4-a9a9-604cf879e178")



async def main():
    client = AuthenticatedClient(
        base_url=api_url,
        token=token)
    batches = await frp.fetch_batches_for_report(client=client,
                                       report_id=report_id,
                                       project_id=project_id)
    print([b.batch_id for b in batches])

    jobs = await frp.fetch_jobs_for_batches(client=client,
                                            batch_ids=[b.batch_id for b in batches],
                                            project_id=project_id)

    print({k: len(v) for (k,v) in jobs.items()})
                                            

if __name__ == '__main__':
    asyncio.run(main())
