"""Resolve this checkout and its optional catkin workspace after relocation."""
import os
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
SCRIPTS_ROOT = REPOSITORY_ROOT / 'opendrone/scripts'


def workspace_root():
    configured = os.environ.get('OPENDRONE_WORKSPACE')
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if not candidate.is_dir():
            raise ValueError(f'OPENDRONE_WORKSPACE does not exist: {candidate}')
        return candidate
    for candidate in REPOSITORY_ROOT.parents:
        if (candidate / 'devel/setup.bash').is_file() or (candidate / '.catkin_tools').is_dir():
            return candidate
    # Standalone checkout: CSV verification and planning outputs remain local.
    # Actual flight requires OPENDRONE_WORKSPACE and a built ROS environment.
    return REPOSITORY_ROOT


def artifacts_root(workspace=None):
    configured = os.environ.get('OPENDRONE_ARTIFACTS')
    root = Path(workspace).resolve() if workspace is not None else workspace_root()
    return Path(configured).expanduser().resolve() if configured else root / 'paper_artifacts'


def px4_source():
    configured = os.environ.get('PX4_SOURCE_DIR')
    return Path(configured).expanduser().resolve() if configured else Path.home() / 'PX4-Autopilot'


def data_root():
    """Resolve the optional full result bundle, separately from runnable code."""
    configured = os.environ.get('OPENDRONE_DATA_DIR')
    candidates = ([Path(configured).expanduser().resolve()] if configured else [
        PACKAGE_ROOT / 'data', workspace_root() / 'paper_artifacts/data_code_results_20261118/data'])
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError('Full results are external. Set OPENDRONE_DATA_DIR to the extracted data/ directory; '
                            'run_suite.py and verify_example.py do not require the full result bundle.')
