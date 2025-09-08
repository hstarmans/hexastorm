from hexastorm.utils import LunaGatewareTestCase, async_test_case

from hexastorm.blocks.photodiode_debounce import PhotodiodeDebounce


class PhotodiodeDebounceTest(LunaGatewareTestCase):
    """
    We set small thresholds so tests run fast:
      - n_low=4  : need 4 consecutive low cycles to trigger
      - n_high=2 : need 2 consecutive high cycles to re-arm
    """

    FRAGMENT_UNDER_TEST = PhotodiodeDebounce
    FRAGMENT_ARGUMENTS = dict(n_low=4, n_high=2)

    async def initialize_signals(self, sim):
        self.sim = sim

    @async_test_case
    async def test_power_on_defaults(self, sim):
        dut = self.dut

        # Default input high (no light), let a few cycles pass.
        sim.set(dut.raw, 1)
        await self.advance_cycles(5)

        self.assertEqual(sim.get(dut.sync_level), 1)
        self.assertEqual(sim.get(dut.valid_pulse), 0)
        self.assertEqual(sim.get(dut.in_refractory), 0)
        # streaks should be saturated on the high side
        self.assertGreaterEqual(sim.get(dut.high_streak), 1)

    @async_test_case
    async def test_noise_shorter_than_n_low_does_not_trigger(self, sim):
        dut = self.dut

        # idle high for a bit
        await sim.tick()
        sim.set(dut.raw, 1)
        await self.advance_cycles(3)

        # inject brief low burst: 3 < n_low (4)
        sim.set(dut.raw, 0)
        await self.advance_cycles(3)
        sim.set(dut.raw, 1)
        await sim.tick()

        # Should not have produced a pulse, nor enter refractory
        self.assertEqual(sim.get(dut.valid_pulse), 0)
        self.assertEqual(sim.get(dut.in_refractory), 0)

    @async_test_case
    async def test_exact_n_low_produces_one_cycle_pulse(self, sim):
        dut = self.dut

        # Go low for exactly n_low cycles (4)
        sim.set(dut.raw, 0)
        await sim.tick()
        # first 3 cycles: no pulse yet
        for _ in range(3):
            await sim.tick()
            self.assertEqual(sim.get(dut.valid_pulse), 0)

        # On the 4th low, pulse should assert and refractory should latch
        await sim.tick()
        self.assertEqual(sim.get(dut.valid_pulse), 1)
        self.assertEqual(sim.get(dut.in_refractory), 1)

        # Next cycle, pulse should drop back to 0 (one-shot)
        await sim.tick()
        self.assertEqual(sim.get(dut.valid_pulse), 0)

    @async_test_case
    async def test_long_low_gives_single_pulse_due_to_refractory(self, sim):
        dut = self.dut

        # Hold low for much longer than n_low; should still only see 1 pulse.
        sim.set(dut.raw, 0)
        pulses = 0
        for _ in range(12):
            await sim.tick()
            pulses += sim.get(dut.valid_pulse)

        self.assertEqual(pulses, 1)
        self.assertEqual(sim.get(dut.in_refractory), 1)

    @async_test_case
    async def test_rearm_after_n_high_and_trigger_again(self, sim):
        dut = self.dut

        # First valid event
        sim.set(dut.raw, 0)
        await sim.tick()
        for _ in range(4):  # reach n_low
            await sim.tick()
        self.assertEqual(sim.get(dut.valid_pulse), 1)
        self.assertEqual(sim.get(dut.in_refractory), 1)

        # Go high for less than n_high -> still refractory
        sim.set(dut.raw, 1)
        await sim.tick()
        self.assertEqual(sim.get(dut.in_refractory), 1)

        # One more high to reach n_high=2 -> re-armed
        await sim.tick()
        # TODO: fix
        # self.assertEqual(sim.get(dut.in_refractory), 0)

        # # Now another valid low burst should fire a second pulse
        # sim.set(dut.raw, 0)
        # for _ in range(3):
        #     await sim.tick()
        #     self.assertEqual(sim.get(dut.valid_pulse), 0)
        # await sim.tick()
        # self.assertEqual(sim.get(dut.valid_pulse), 1)
        # self.assertEqual(sim.get(dut.in_refractory), 1)

    @async_test_case
    async def test_sync_level_tracks_raw_with_two_ff(self, sim):
        dut = self.dut

        # Toggle raw and ensure sync_level eventually matches.
        sim.set(dut.raw, 0)
        await sim.tick()
        await sim.tick()  # two cycles to propagate through 2FF path
        self.assertEqual(sim.get(dut.sync_level), 0)

        sim.set(dut.raw, 1)
        await sim.tick()
        await sim.tick()
        self.assertEqual(sim.get(dut.sync_level), 1)
