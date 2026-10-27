from ament_copyright.main import main
import pytest

@pytest.mark.copyright_test
def test_copyright():
    rc = main(argv=['.', 'multi_goal_to_gp_goal'])
    assert rc == 0, 'Found errors'