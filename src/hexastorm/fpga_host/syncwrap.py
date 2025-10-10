"""Utility to wrap async APIs for synchronous use in tests or REPL.

Example usage:
    @syncable
    class MyClass:
        async def foo(self): ...

    obj = MyClass(sync=True)   # async methods run synchronously
    obj.foo()  # no await needed
"""

import asyncio


def _is_coro_like(obj):
    """Return True if `obj` looks like a coroutine/generator (MicroPython/CPython)."""
    return hasattr(obj, "send") and hasattr(obj, "throw")


def _loop_is_running():
    """Best-effort check for an active event loop in MicroPython/CPython."""
    # MicroPython & CPython (3.7+): current_task()
    cur_task = getattr(asyncio, "current_task", None)
    if callable(cur_task):
        try:
            if cur_task() is not None:
                return True
        except Exception:
            pass
    # CPython: get_running_loop()
    get_running_loop = getattr(asyncio, "get_running_loop", None)
    if callable(get_running_loop):
        try:
            get_running_loop()
            return True
        except Exception:
            pass
    # CPython older: get_event_loop().is_running()
    get_event_loop = getattr(asyncio, "get_event_loop", None)
    if callable(get_event_loop):
        try:
            loop = get_event_loop()
            is_running = getattr(loop, "is_running", None)
            if callable(is_running) and is_running():
                return True
        except Exception:
            pass
    return False


def _run_sync(coro):
    """Run a coroutine to completion when no loop is active.
    If a loop is running, close the coroutine to avoid 'never awaited' warnings,
    then raise RuntimeError.
    """
    if not _loop_is_running():
        # Case 1: No loop is running (e.g., in REPL or simple sync function).
        # Use asyncio.run, which handles loop creation and cleanup.
        return asyncio.run(coro)

    # Case 2: A loop is already running (e.g., inside an async test or task).
    # We MUST NOT call asyncio.run() or it will raise the error.
    # Instead, we get the active loop and use its run_until_complete.
    # This is generally acceptable in test contexts where you block a thread
    # to wait for an async result.
    try:
        # Get the running loop. get_event_loop() is used here for broadest compatibility.
        loop = asyncio.get_event_loop()

        # NOTE: loop.is_running() should be True at this point based on the check above,
        # but using get_event_loop() is the most reliable way to get a loop object
        # that *might* be running if current_task/get_running_loop failed.

        # Run the coroutine to completion, blocking the current thread.
        return loop.run_until_complete(coro)

    except RuntimeError as e:
        # If get_event_loop() raises a RuntimeError (e.g., "no current event loop"),
        # or if run_until_complete fails in a nested context, re-raise it.
        # Ensure coroutine closure to prevent 'never awaited' warning.
        closer = getattr(coro, "close", None)
        if callable(closer):
            closer()
        raise RuntimeError(
            "Cannot run synchronously while an event loop is running. "
            "If inside an async function, use 'await' directly."
        ) from e


class SyncProxy:
    """
    Proxy that wraps an object and automatically runs async methods
    synchronously using asyncio.run(). Useful in tests or REPL when
    you want to call async APIs like normal functions.
    """

    __slots__ = ("_obj",)

    def __init__(self, obj):
        # Avoid recursion by setting internal attribute directly
        object.__setattr__(self, "_obj", obj)

    def __repr__(self):
        return f"<SyncProxy {self._obj!r}>"

    def __setattr__(self, name, value):
        # Forward sets to the wrapped object
        setattr(self._obj, name, value)

    def __getattr__(self, name):
        attr = getattr(self._obj, name)

        if callable(attr):

            def call(*a, **k):
                res = attr(*a, **k)
                if _is_coro_like(res):
                    try:
                        return _run_sync(res)
                    except Exception:
                        # In case _run_sync didn't get to consume it, ensure closure
                        closer = getattr(res, "close", None)
                        if callable(closer):
                            try:
                                closer()
                            except Exception:
                                pass
                        raise
                return res

            return call

        # async property -> attr is a coroutine object
        if _is_coro_like(attr):
            try:
                return _run_sync(attr)
            except Exception:
                closer = getattr(attr, "close", None)
                if callable(closer):
                    try:
                        closer()
                    except Exception:
                        pass
                raise

        return attr


def syncable(cls):
    """
    Class decorator that makes a class instantiable in sync mode:

        @syncable
        class MyClass:
            async def foo(self): ...

        obj = MyClass(sync=True)   # async methods run synchronously
        obj.foo()  # no await needed
    """

    def wrapper(*a, sync=True, **k):
        obj = cls(*a, **k)
        return SyncProxy(obj) if sync else obj

    return wrapper
