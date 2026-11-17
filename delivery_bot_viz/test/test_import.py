import importlib


def test_package_imports():
    assert importlib.import_module('delivery_bot_viz').__doc__
