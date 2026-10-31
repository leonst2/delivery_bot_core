import os

import yaml

PARAMS_DIR = os.path.join(os.path.dirname(__file__), '..', 'params')
# Sim and robot share common.yaml. Add a key here only if it really is a simulation difference.
ALLOWED_SIM_KEYS = {'use_sim_time'}


def load(name):
    with open(os.path.join(PARAMS_DIR, name)) as params_file:
        return yaml.safe_load(params_file)


def test_sim_overlay_only_holds_simulation_differences():
    for node in load('sim.yaml').values():
        assert set(node['ros__parameters']) <= ALLOWED_SIM_KEYS


def test_sim_overlay_enables_sim_time():
    assert load('sim.yaml')['/**']['ros__parameters']['use_sim_time'] is True


def test_common_params_do_not_set_sim_time():
    for node in load('common.yaml').values():
        assert 'use_sim_time' not in node['ros__parameters']
