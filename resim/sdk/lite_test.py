import time
import random
import os
from resim.sdk.batch import Batch
from resim.sdk.test import Test
from resim.sdk.auth.username_password_client import UsernamePasswordClient

# === localhost ===
# RERUN_API_BASE_URL = "http://localhost:8080"
# PROJECT_ID = "2160d246-52e8-4c0f-a0fd-75c0345afc57"  # from local

# === staging ===
# RERUN_API_BASE_URL = "https://api.resim.io"
# BFF_BASE_URL = "https://bff.resim.io/graphql"
# PROJECT_ID = "f6bd6358-5c44-430a-9476-12e7eb169fe3"  # from staging

# === dev app 2869 ===
RERUN_API_BASE_URL = "https://dev-env-pr-2869.api.dev.resim.io/v1"
PROJECT_ID = "f4019641-2c17-4f24-9e98-fef809b23519"  # 2869 dev app


def main() -> None:
    client = UsernamePasswordClient(
        client_id="LLNl3xsbNLSd16gQyYsiEn3tbLDZo1gj",
        base_url=RERUN_API_BASE_URL,
        domain="https://resim-dev.us.auth0.com",
        username="priv.resim.ai",
        password=os.environ["RESIM_PASSWORD"],
    )

    branch_name = "mgrijalva-testing"
    with Batch(
        client=client,
        project_id=PROJECT_ID,
        branch=branch_name,
        metrics_set_name="cool metrics",
        metrics_config_path="resim/sdk/.resim/metrics/config.resim.yml",
    ) as batch:
        print(f"Created batch {batch.friendly_name}. id {batch.id}")
        with Test(client, batch, "hello world") as test:
            for i in range(0, 100):
                test.emit(
                    "position",
                    {"x": i, "y": i * random.randint(1, 5)},
                    time.time_ns(),
                )

        with Test(client, batch, "test 2") as test:
            for i in range(0, 100):
                test.emit(
                    "position",
                    {"x": i, "y": i * random.randint(1, 2)},
                    time.time_ns(),
                )

        with Test(client, batch, "test 3") as test:
            for i in range(0, 100):
                test.emit(
                    "position",
                    {"x": i, "y": i * random.randint(1, 8)},
                    time.time_ns(),
                )
    print("Batch done!")


if __name__ == "__main__":
    main()
