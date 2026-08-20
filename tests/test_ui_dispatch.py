import threading
import unittest

from motor_control.ui_dispatch import UiDispatcher, UiDispatchTimeout


class UiDispatcherTests(unittest.TestCase):
    def test_owner_call_runs_immediately(self):
        dispatcher = UiDispatcher()
        self.assertEqual(dispatcher.call(lambda: 42), 42)

    def test_worker_call_waits_for_owner_drain(self):
        dispatcher = UiDispatcher()
        result = []

        worker = threading.Thread(target=lambda: result.append(dispatcher.call(lambda: "ok")))
        worker.start()
        while not result and worker.is_alive():
            dispatcher.drain()
            worker.join(0.01)
        worker.join(1.0)
        self.assertEqual(result, ["ok"])

    def test_worker_receives_callback_exception(self):
        dispatcher = UiDispatcher()
        errors = []

        def fail():
            raise ValueError("bad config")

        def worker_call():
            try:
                dispatcher.call(fail)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=worker_call)
        worker.start()
        while worker.is_alive():
            dispatcher.drain()
            worker.join(0.01)
        self.assertIsInstance(errors[0], ValueError)
        self.assertEqual(str(errors[0]), "bad config")

    def test_async_error_is_reported_and_next_action_runs(self):
        errors = []
        dispatcher = UiDispatcher(errors.append)
        called = []
        dispatcher.post(lambda: 1 / 0)
        dispatcher.post(lambda: called.append(True))
        self.assertEqual(dispatcher.drain(), 2)
        self.assertIsInstance(errors[0], ZeroDivisionError)
        self.assertEqual(called, [True])

    def test_timeout_is_explicit(self):
        dispatcher = UiDispatcher()
        errors = []
        called = []

        def worker_call():
            try:
                dispatcher.call(lambda: called.append(True), timeout=0.01)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=worker_call)
        worker.start()
        worker.join(1.0)
        self.assertIsInstance(errors[0], UiDispatchTimeout)
        dispatcher.drain()
        self.assertEqual(called, [])

    def test_callback_started_at_timeout_returns_real_result(self):
        dispatcher = UiDispatcher()
        entered = threading.Event()
        release = threading.Event()
        result = []

        def callback():
            entered.set()
            release.wait(1.0)
            return "applied"

        worker = threading.Thread(
            target=lambda: result.append(dispatcher.call(callback, timeout=0.01))
        )
        worker.start()

        def release_callback():
            self.assertTrue(entered.wait(1.0))
            release.set()

        releaser = threading.Thread(target=release_callback)
        releaser.start()
        dispatcher.drain()
        worker.join(1.0)
        releaser.join(1.0)
        self.assertEqual(result, ["applied"])


if __name__ == "__main__":
    unittest.main()
