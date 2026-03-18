# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import time
import random
from resim.sdk.batch import Batch
from resim.sdk.test import Test
from resim.sdk.auth.username_password_client import UsernamePasswordClient

PROJECT_ID = "<project_id>"
BRANCH_NAME = "my-test-branch"
USERNAME = "<username>"
PASSWORD = "<password>"

def main() -> None:
    client = UsernamePasswordClient(username=USERNAME, password=PASSWORD)

    with Batch(client=client, project_id=PROJECT_ID, branch=BRANCH_NAME, metrics_set_name="my metrics", metrics_config_path="config.resim.yml") as batch:
        print(f"Created batch {batch.friendly_name}. id {batch.id}")
        
        with Test(client, batch, "hello world") as test:
            # Emit sensor data
            for i in range(0, 100):
                test.emit("position", {"x": i, "y": i * random.randint(1, 5)}, time.time_ns())

            # Emit an image
            test.attach_log("robot.png")
            test.emit("images", {"img": "arm.gif"}, time.time_ns())
    
    print("Batch done!")


if __name__ == "__main__":
    main()
