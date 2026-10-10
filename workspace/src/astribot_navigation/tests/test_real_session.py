"""Session teardown must expose child crashes and still stop other children."""
import subprocess
import sys
import pytest
from towergo_real.session import stop_all, stop_process


def test_nonzero_exit_is_not_accepted_as_clean_shutdown():
    process=subprocess.Popen([sys.executable,'-c','raise SystemExit(7)'],start_new_session=True)
    process.wait(timeout=5)
    with pytest.raises(RuntimeError,match='abnormally: 7'):
        stop_process(process)


def test_crashed_child_does_not_prevent_remaining_cleanup():
    failed=subprocess.Popen([sys.executable,'-c','raise SystemExit(7)'],start_new_session=True)
    failed.wait(timeout=5)
    other=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],
                           start_new_session=True,stderr=subprocess.DEVNULL)
    try:
        with pytest.raises(RuntimeError,match='abnormally: 7'):
            stop_all([failed,other])
        assert other.poll() is not None
    finally:
        if other.poll() is None:other.kill();other.wait(timeout=5)
