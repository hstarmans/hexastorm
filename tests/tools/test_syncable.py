import unittest
import asyncio

from hexastorm.fpga_host.syncwrap import syncable


def _is_coro_like(obj):
    # Simple, MicroPython-friendly coroutine detector
    return hasattr(obj, "send") and hasattr(obj, "throw")


@syncable
class MyClass:
    def __init__(self):
        self._x = 0
        self.tag = "init"

    async def foo(self, inc=1):
        await asyncio.sleep(0)
        self._x += inc
        return self._x

    @property
    async def position(self):
        await asyncio.sleep(0)
        return 42

    @property
    def name(self):
        return "widget"


class TestSyncable(unittest.TestCase):
    def test_async_method_runs_sync(self):
        obj = MyClass(sync=True)
        v1 = obj.foo(2)  # no await
        v2 = obj.foo(3)  # cumulative
        self.assertEqual(v1, 2)
        self.assertEqual(v2, 5)

    def test_async_property_runs_sync(self):
        obj = MyClass(sync=True)
        self.assertEqual(obj.position, 42)  # no await

    def test_regular_property_and_attr_passthrough(self):
        obj = MyClass(sync=True)
        self.assertEqual(obj.name, "widget")
        obj.tag = "changed"
        self.assertEqual(obj.tag, "changed")

    def test_sync_false_exposes_coroutines(self):
        obj = MyClass(sync=False)
        # method returns coroutine
        m = obj.foo(1)
        self.assertTrue(_is_coro_like(m))
        # async property access yields coroutine
        p = obj.position
        self.assertTrue(_is_coro_like(p))

        # Sanity: actually await them to ensure they work
        async def run():
            r1 = await m
            r2 = await p
            return r1, r2

        r1, r2 = asyncio.run(run())
        self.assertEqual(r1, 1)
        self.assertEqual(r2, 42)

    def test_raises_inside_running_event_loop(self):
        # Inside a running loop, sync wrapper should refuse to block
        async def inner():
            obj = MyClass(sync=True)
            with self.assertRaises(RuntimeError):
                _ = obj.position  # would need to block -> should raise
            with self.assertRaises(RuntimeError):
                _ = obj.foo(1)  # same for methods

        asyncio.run(inner())
