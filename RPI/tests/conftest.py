"""Pytest bootstrap: stub Raspberry Pi-only camera modules on Windows/dev machines."""

import importlib
import sys
import types

for _mod_name, _attrs in (
    ("libcamera", {"Transform": lambda **kw: kw}),
    ("picamera2", {"MappedArray": object, "Picamera2": object}),
):
    if _mod_name not in sys.modules:
        try:
            importlib.import_module(_mod_name)
        except ImportError:
            _stub = types.ModuleType(_mod_name)
            for _k, _v in _attrs.items():
                setattr(_stub, _k, _v)
            sys.modules[_mod_name] = _stub
