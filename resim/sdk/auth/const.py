import pathlib

DEFAULT_BASE_URL = "https://api.resim.ai/v1"
DEFAULT_DOMAIN = "https://resim.us.auth0.com"
DEFAULT_SCOPE = "offline_access"
DEFAULT_AUDIENCE = "https://api.resim.ai"
DEFAULT_CACHE_LOCATION = pathlib.Path.home() / ".resim" / "token.json"
