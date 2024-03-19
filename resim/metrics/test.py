
import uuid
import resim.metrics.fetch_job_metrics as fjm




with open("/workspaces/token.txt", "r") as tokenfile:
    TOKEN = tokenfile.read()

# Remove newline
TOKEN = TOKEN[:-1]

assert len(TOKEN) != 0
print(f"Read in bearer token of length {len(TOKEN)}")


API_URL = "https://api.resim.io/v1/"
BATCH_ID = uuid.UUID("87ac5dbb-fa66-4ef4-8d03-b472ced03593")
PROJECT_ID = uuid.UUID("035168b0-22f9-407f-b7e4-cd8d4dd30da1")

metrics = fjm.fetch_job_metrics_by_batch(
    token=TOKEN,
    api_url=API_URL,
    project_id=PROJECT_ID,
    batch_id=BATCH_ID)

print(metrics)
