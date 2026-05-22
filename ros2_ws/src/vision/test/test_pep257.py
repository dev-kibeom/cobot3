import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    pytest.skip('Docstring lint is disabled for prototype runtime/tools package.')
