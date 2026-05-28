import asyncio


def test_mock_connect_close():
    from mavpilot.controller import DroneController

    async def _run():
        d = DroneController(mock=True, enable_viz=False)
        await d.connect()
        d.close()

    asyncio.run(_run())
