import io
import os
import sys
from typing import Optional


class IOUtil(io.TextIOBase):
    """
    A filtered, file-like stream that only forwards writes originating from
    desinated module to a preserved stdout/stderr stream.

    This class also exposes process-wide muting controls via `mute()` and
    `restore()` classmethods. When muted, file descriptors 1 and 2 are redirected
    to /dev/null (silencing Python prints and C-level writes), while
    `sys.stdout`/`sys.stderr` are replaced with instances of this class that
    forward only Logger output to the preserved original streams.
    """

    # Installation state
    _installed: bool = False
    _saved_stdout: Optional[io.TextIOBase] = None
    _saved_stderr: Optional[io.TextIOBase] = None

    # Filtering configuration (can be overridden on first mute call)
    _allow_suffix: str = "logger.py"
    _allow_func: str = None
    _max_depth: int = 12

    def __init__(self, target_stream: io.TextIOBase):
        """
        Create a filtered stream proxy.

        Args:
            target_stream: The preserved, real stream to forward allowed writes to.
        """
        self._target = target_stream

    # -------------------------
    # Public class API
    # -------------------------
    @classmethod
    def is_muted(cls) -> bool:
        return cls._installed

    @classmethod
    def mute(
        cls,
        allow_file_suffix: Optional[str] = None,
        allow_func: Optional[str] = None,
        max_depth: Optional[int] = None,
    ) -> None:
        """
        Enable global muting of stdout/stderr while allowing only the project's
        `Logger` to print.

        Notes:
        - Idempotent; calling multiple times is safe.
        - Should be called as early as possible, before importing noisy modules.
        - Affects the entire process (including threads used by executors).
        """

        if cls._installed:
            return

        # Apply configuration overrides once, on first mute.
        if allow_file_suffix is not None:
            cls._allow_suffix = allow_file_suffix
        if allow_func is not None:
            cls._allow_func = allow_func
        if max_depth is not None:
            cls._max_depth = max_depth

        # Flush current high-level streams
        try:
            sys.stdout.flush()
        except Exception:
            pass
        try:
            sys.stderr.flush()
        except Exception:
            pass

        # Preserve original stdout/stderr at the fd level, so we can still
        # write Logger output to the terminal/file even after muting fds 1/2.
        saved_out_fd = os.dup(1)
        saved_err_fd = os.dup(2)
        cls._saved_stdout = os.fdopen(saved_out_fd, "w", buffering=1)
        cls._saved_stderr = os.fdopen(saved_err_fd, "w", buffering=1)

        # Replace Python-level streams with our filtered proxies that forward
        # only Logger-originated writes.
        sys.stdout = cls(cls._saved_stdout)  # type: ignore[assignment]
        sys.stderr = cls(cls._saved_stderr)  # type: ignore[assignment]

        # Redirect OS fds to /dev/null to silence C-level prints
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        try:
            os.dup2(devnull_fd, 1)
            os.dup2(devnull_fd, 2)
        finally:
            os.close(devnull_fd)

        cls._installed = True

    @classmethod
    def restore(cls) -> None:
        """
        Restore stdout/stderr back to the preserved original streams and stop
        filtering. Safe to call if not muted.
        """
        if not cls._installed:
            return

        # Flush any pending writes
        try:
            sys.stdout.flush()
        except Exception:
            pass
        try:
            sys.stderr.flush()
        except Exception:
            pass

        # Reconnect fds 1/2 to preserved streams
        if cls._saved_stdout is not None:
            os.dup2(cls._saved_stdout.fileno(), 1)
        if cls._saved_stderr is not None:
            os.dup2(cls._saved_stderr.fileno(), 2)

        # Restore high-level streams
        if cls._saved_stdout is not None:
            sys.stdout = cls._saved_stdout  # type: ignore[assignment]
        if cls._saved_stderr is not None:
            sys.stderr = cls._saved_stderr  # type: ignore[assignment]

        cls._installed = False

    # -------------------------
    # TextIOBase overrides
    # -------------------------
    def _allowed(self) -> bool:
        """
        Determine whether the current write() call originates from the project's
        Logger implementation (`_print_msg` inside `src/shared/logger.py`).
        """
        # Skip two frames: this method and write(); then walk up the stack.
        try:
            frame = sys._getframe(2)
        except ValueError:
            frame = None

        steps_remaining = type(self)._max_depth
        allow_suffix = type(self)._allow_suffix
        allow_func = type(self)._allow_func

        while frame is not None and steps_remaining > 0:
            code_obj = frame.f_code
            filename = code_obj.co_filename.replace("\\", "/")
            if filename.endswith(allow_suffix):
                # If allow_func is None, allow any function within logger.py
                if allow_func is None or code_obj.co_name == allow_func:
                    return True
            frame = frame.f_back
            steps_remaining -= 1

        return False

    def write(self, s: str) -> int:  # type: ignore[override]
        if self._allowed():
            return self._target.write(s)
        # Drop the write but report that we consumed the bytes
        return len(s)

    def flush(self) -> None:  # type: ignore[override]
        try:
            self._target.flush()
        except Exception:
            pass

    def writable(self) -> bool:  # type: ignore[override]
        return True

    def isatty(self) -> bool:  # type: ignore[override]
        try:
            return bool(self._target.isatty())  # type: ignore[attr-defined]
        except Exception:
            return False

    def fileno(self) -> int:  # type: ignore[override]
        return self._target.fileno()

    @property
    def encoding(self) -> str:  # type: ignore[override]
        return getattr(self._target, "encoding", "utf-8")
