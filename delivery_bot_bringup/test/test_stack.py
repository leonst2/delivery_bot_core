from delivery_bot_bringup.stack import NODES, OPTIONAL_STAGES, select_stages, STAGES
import pytest


def names(stages):
    return [node.name for _, nodes in stages for node in nodes]


def test_node_names_are_unique():
    assert len({node.name for node in NODES}) == len(NODES)


def test_every_node_belongs_to_a_known_stage():
    assert {node.stage for node in NODES} == set(STAGES)


def test_stages_start_in_dependency_order():
    assert [stage for stage, _ in select_stages()] == list(STAGES)


def test_localization_mode_selects_one_node():
    assert 'localization' in names(select_stages('mcl'))
    assert 'ground_truth_localization' not in names(select_stages('mcl'))
    assert 'ground_truth_localization' in names(select_stages('ground_truth'))
    assert 'localization' not in names(select_stages('ground_truth'))


@pytest.mark.parametrize('stage', OPTIONAL_STAGES)
def test_optional_stages_can_be_disabled(stage):
    assert stage not in [name for name, _ in select_stages(disabled_stages=[stage])]


def test_core_stages_can_not_be_disabled():
    with pytest.raises(ValueError):
        select_stages(disabled_stages=['odometry'])


def test_unknown_localization_mode_is_rejected():
    with pytest.raises(ValueError):
        select_stages('slam')
