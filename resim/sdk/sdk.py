from contextlib import contextmanager
from dataclasses import dataclass, field
import tempfile
from typing import Optional
from resim.metrics.python.emissions import Emitter
from pathlib import Path


@dataclass
class Test(Emitter):
    name: str

    def __init__(self, name: str, filename: Path):
        self.name = name
        super().__init__(output_path=filename)


@dataclass
class Batch:
    name: str
    project: str
    system: str
    branch: Optional[str] = None
    version: Optional[str] = None
    tests: list[Test] = field(default_factory=list)

    def __init__(
        self,
        *,
        name: str,
        project: str,
        system: str,
        branch: Optional[str] = None,
        version: Optional[str] = None,
    ):
        self.name = name
        self.project = project
        self.system = system
        self.branch = branch
        self.version = version
        self.tests = []

    @contextmanager
    def run_test(self, name: str):
        # TODO(mattc): ensure we delete the file after it's uploaded
        with tempfile.NamedTemporaryFile(
            suffix=".resim.jsonl", delete=False
        ) as temp_file:
            test = Test(name, Path(temp_file.name))
            self.tests.append(test)
            try:
                yield test
            finally:
                self.tests.remove(test)


@contextmanager
def init(
    batch: str,
    project: str,
    system: str,
    branch: Optional[str] = None,
    version: Optional[str] = None,
):
    # do nothing
    try:
        yield Batch(
            name=batch, project=project, system=system, branch=branch, version=version
        )
    finally:
        # check if the batch exists
        # If it does, update the test suite with the new tests, call a rerun
        # if it doesn't, create a new test suite, run it and then upload the results
        pass
