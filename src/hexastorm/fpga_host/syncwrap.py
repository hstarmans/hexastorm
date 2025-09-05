import asyncio


def _is_coro_like(obj):
    """Return True if `obj` looks like a coroutine/generator (MicroPython/CPython)."""
    return hasattr(obj, "send") and hasattr(obj, "throw")


class SyncProxy:
    """
    Proxy that wraps an object and automatically runs async methods
    synchronously using asyncio.run(). Useful in tests or REPL when
    you want to call async APIs like normal functions.
    """

    def __init__(self, obj):
        self._obj = obj

    def __getattr__(self, name):
        attr = getattr(self._obj, name)
        if callable(attr):

            def call(*a, **k):
                res = attr(*a, **k)
                if _is_coro_like(res):
                    return asyncio.run(res)  # only when no loop is active
                return res

            return call
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

    def wrapper(*a, sync=False, **k):
        obj = cls(*a, **k)
        return SyncProxy(obj) if sync else obj

    return wrapper
