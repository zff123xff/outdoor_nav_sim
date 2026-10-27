from ament_pep257.main import main
import pytest

@pytest.mark.linter_test
def test_pep257():
    rc = main(argv=['.', 'test_pep257'])
    assert rc == 0, 'Found errors'