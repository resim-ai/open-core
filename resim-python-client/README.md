# resim-python-client
A client library for accessing the ReSim Customer API

## Usage
First, create a client:

```python
from resim_python_client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient, UsernamePasswordClient

# If you're using your local machine, you can use the device code flow to get a token
device_code_client = DeviceCodeClient()
client = AuthenticatedClient(base_url="https://api.resim.ai/v1/", token=device_code_client.get_jwt())

# OR

# If you're running in CI, ReSim will provide you with a username & password to auth with
client = UsernamePasswordClient(
    username="your_username", # or set the `RESIM_USERNAME` environment variable
    password="your_password"  # or set the `RESIM_PASSWORD` environment variable
)
```

Now call your endpoint and use the generated models:

```python
from resim_python_client.models import Batch
from resim_python_client.api.batches import get_batch
from resim_python_client.types import Response

with client as client:
    my_data = get_batch.sync(client=client)
    assert my_data != None, "Failed to get batch"
    # or if you need more info (e.g. status_code)
    response: Response[Batch] = get_batch.sync_detailed(client=client)
    assert response.status_code == 200, "Failed to get batch"
```

Or do the same thing with an async version:

```python
from resim_python_client.models import Batch
from resim_python_client.api.batches import get_batch
from resim_python_client.types import Response

async with client as client:
    my_data: Batch = await get_batch.asyncio(client=client)
    assert my_data != None, "Failed to get batch"
    # or if you need more info (e.g. status_code)
    response: Response[Batch] = await get_batch.asyncio_detailed(client=client)
    assert response.status_code == 200, "Failed to get batch"
```

To discover more endpoints, check out our [Swagger Docs](https://redocly.github.io/redoc/?url=https://api.resim.ai). 

Things to know:
1. Every path/method combo becomes a Python module with four functions:
    1. `sync`: Blocking request that returns parsed data (if successful) or `None`. This may be omitted if there is no data to parse.
    1. `sync_detailed`: Blocking request that always returns a `Request`, optionally with `parsed` set if the request was successful.
    1. `asyncio`: Like `sync` but async instead of blocking. 
    1. `asyncio_detailed`: Like `sync_detailed` but async instead of blocking.

1. All path/query params, and bodies become method arguments.
1. If your endpoint had any tags on it, the first tag will be used as a module name for the function (`batches` above)
1. Any endpoint which did not have a tag is in `resim_python_client.api.default`

### Example: updating an experience

If you are looking to update an experience, browse to the [updateExperience endpoint](https://redocly.github.io/redoc/?url=https://api.resim.ai#tag/experiences/operation/updateExperience) and you'll see that it has a tag of `experiences`. This means that the generated module will be `resim_python_client.api.experiences.update_experience`. A quick usage example:

```python
from resim_python_client.models import UpdateExperienceInput, Experience
from resim_python_client.api.experiences import update_experience

with client as client:
    updated_experience: Experience = update_experience.sync(
        project_id="ca3b7ce3-0242-4da7-bf51-1eb1945c7de3", experience_id="bf6806c7-aa15-464d-8b2c-387d12c732da", 
        client=client, body=UpdateExperienceInput(name="Better Experience")
    )
    assert updated_experience != None, "Failed to update experience"
    assert updated_experience.name == "Better Experience", "Experience name was not updated"
```

## Advanced customizations

There are more settings on the generated `Client` class which let you control more runtime behavior, check out the docstring on that class for more info. You can also customize the underlying `httpx.Client` or `httpx.AsyncClient` (depending on your use-case):

```python
from resim_python_client import Client

def log_request(request):
    print(f"Request event hook: {request.method} {request.url} - Waiting for response")

def log_response(response):
    request = response.request
    print(f"Response event hook: {request.method} {request.url} - Status {response.status_code}")

client = Client(
    base_url="https://api.example.com",
    httpx_args={"event_hooks": {"request": [log_request], "response": [log_response]}},
)

# Or get the underlying httpx client to modify directly with client.get_httpx_client() or client.get_async_httpx_client()
```

You can even set the httpx client directly, but beware that this will override any existing settings (e.g., base_url):

```python
import httpx
from resim_python_client import Client

client = Client(
    base_url="https://api.example.com",
)
# Note that base_url needs to be re-set, as would any shared cookies, headers, etc.
client.set_httpx_client(httpx.Client(base_url="https://api.example.com", proxies="http://localhost:8030"))
```
