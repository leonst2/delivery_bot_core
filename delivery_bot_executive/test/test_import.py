import importlib


def test_node_module_has_an_entry_point():
    module = importlib.import_module('delivery_bot_executive.executive')
    assert callable(module.main)
