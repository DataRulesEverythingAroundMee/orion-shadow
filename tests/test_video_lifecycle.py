import asyncio
import unittest
from orion_shadow.engine.video_server import VideoServer
from orion_shadow.core.state import GimbalState
from orion_shadow.server import OrionServer

class TestVideoLifecycle(unittest.TestCase):
    def test_video_server_start_and_clean_cancellation(self):
        async def run_test():
            state = GimbalState()
            state.initialized = True
            server = VideoServer(state, fps=24, port=5004)
            task = asyncio.create_task(server.start())
            await asyncio.sleep(0.5)
            self.assertTrue(task.done() is False)
            self.assertIsNotNone(server.current_frame)
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self.assertIsNone(server.proc)

        asyncio.run(run_test())

    def test_orion_server_shutdown_with_video(self):
        async def run_test():
            server = OrionServer(
                host="127.0.0.1",
                port=0,
                udp_in_port=0,
                tcp_port=None,
                video_enabled=True,
                video_port=5004,
                video_fps=24,
            )
            task = asyncio.create_task(server.run())
            await asyncio.sleep(0.5)
            self.assertTrue(task.done() is False)
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self.assertFalse(server._running)

        asyncio.run(run_test())

if __name__ == "__main__":
    unittest.main()
