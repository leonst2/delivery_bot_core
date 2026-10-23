from delivery_bot_grid_step.grid_step_controller import STATES


def test_loop_states_present():
    for state in ('HALT', 'PERCEIVE', 'UPDATE_MAP', 'PLAN', 'MOVE_ONE_STEP'):
        assert state in STATES
