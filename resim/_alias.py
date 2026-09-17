# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Import-time aliasing so legacy resim.* module paths resolve to their new homes.

The SDK and demo moved from resim.sdk / resim.demo to signalflag.sdk /
signalflag.demo. :func:`alias_package` keeps the old dotted paths importable and,
importantly, makes them resolve to the *same* module objects as the new paths. That
preserves isinstance checks across old and new imports and lets mock.patch
on an old path affect the new module.
"""

import importlib
import importlib.abc
import importlib.util
import sys
from types import ModuleType
from typing import Optional, Sequence


class _AliasLoader(importlib.abc.Loader):
    """Loader that hands back an already-imported module instead of executing anything."""

    def __init__(self, module: ModuleType):
        self._module = module
        self._spec = module.__spec__

    def create_module(self, spec):  # type: ignore[override]
        return self._module

    def exec_module(self, module: ModuleType) -> None:
        # The import machinery overwrites __spec__ with the alias spec; put the real
        # one back so the module keeps reporting its canonical name and location.
        module.__spec__ = self._spec


class _AliasFinder(importlib.abc.MetaPathFinder):
    """Resolve <old>.x.y to the already-importable <new>.x.y."""

    def __init__(self) -> None:
        self._aliases: dict[str, str] = {}

    def add(self, old: str, new: str) -> None:
        self._aliases[old] = new

    def find_spec(
        self,
        fullname: str,
        path: Optional[Sequence[str]] = None,
        target: Optional[ModuleType] = None,
    ):
        for old, new in self._aliases.items():
            if not fullname.startswith(old + "."):
                continue
            new_name = new + fullname[len(old) :]
            try:
                module = importlib.import_module(new_name)
            except ModuleNotFoundError as e:
                if e.name == new_name:
                    # Nothing by that name on the new side either: let the normal
                    # import raise ModuleNotFoundError for the name the caller used.
                    return None
                raise
            return importlib.util.spec_from_loader(
                fullname, _AliasLoader(module), is_package=hasattr(module, "__path__")
            )
        return None


_finder = _AliasFinder()


def alias_package(old: str, new: str) -> ModuleType:
    """Make old (and every submodule under it) an alias of new.

    Call this from the __init__ of the legacy package, passing __name__ as
    old. The legacy package replaces itself in sys.modules with the new
    package, so import old yields the new module object, and a finder is
    registered so import old.sub.module yields new.sub.module.
    """
    if _finder not in sys.meta_path:
        sys.meta_path.insert(0, _finder)
    _finder.add(old, new)
    module = importlib.import_module(new)
    sys.modules[old] = module
    return module
