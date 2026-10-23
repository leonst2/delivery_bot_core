def test_import():
    import delivery_bot_perception.obstacle_tracker as tracker
    assert callable(tracker.main)
