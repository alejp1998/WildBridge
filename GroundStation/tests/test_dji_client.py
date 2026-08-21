from wildbridge_groundstation.dji_client import DJIInterface


class RecordingDJIInterface(DJIInterface):
    def __init__(self):
        self.calls = []
        super().__init__("192.168.1.42")

    def requestSend(self, endPoint, data, verbose=False):  # noqa: N802, N803
        self.calls.append((endPoint, data, verbose))
        return "ok"


def test_client_uses_string_discovery_result():
    client = DJIInterface("", discover_callback=lambda: "192.168.1.42")

    assert client.IP_RC == "192.168.1.42"
    assert client.drone_name == "UNKNOWN"
    assert client.baseCommandUrl == "http://192.168.1.42:8080"


def test_client_uses_tuple_discovery_result_and_name():
    client = DJIInterface("", discover_callback=lambda: ("192.168.1.42", "mini1"))

    assert client.IP_RC == "192.168.1.42"
    assert client.drone_name == "mini1"


def test_client_can_query_config_name_for_known_ip():
    client = DJIInterface(
        "192.168.1.42",
        config_loader=lambda ip: {"droneName": f"name-for-{ip}"},
        query_config_name=True,
    )

    assert client.drone_name == "name-for-192.168.1.42"


def test_process_telemetry_data_stores_latest_complete_item():
    client = DJIInterface("192.168.1.42", timestamp_factory=lambda: "t1")

    buffer = client._process_telemetry_data(
        '{"heading": 1',
        b'}\nnot-json\n{"batteryLevel": 88}\n{"partial": true',
    )

    assert buffer == '{"partial": true'
    assert client.getTelemetry() == {"batteryLevel": 88, "timestamp": "t1"}


def test_request_send_posts_to_normalized_endpoint(monkeypatch):
    posts = []

    class Response:
        content = b"accepted"

    def fake_post(url, data, timeout):
        posts.append((url, data, timeout))
        return Response()

    monkeypatch.setattr("wildbridge_groundstation.dji_client.requests.post", fake_post)
    client = DJIInterface("192.168.1.42")

    assert client.requestSend("send/takeoff", "") == "accepted"
    assert posts == [("http://192.168.1.42:8080/send/takeoff", "", 5)]


def test_command_helpers_format_requests():
    client = RecordingDJIInterface()

    assert client.requestSendStick(2, -2, 0.5, -0.5) == "ok"
    assert client.requestSendGoToWPwithPID(1.0, 2.0, 3.0, 4.0, 5.5) == "ok"
    assert client.requestAbortAll() == "ok"

    assert client.calls == [
        ("/send/stick", "0.3000,-0.3000,0.3000,-0.3000", False),
        ("/send/gotoWPwithPID", "1.0,2.0,3.0,4.0,5.5", False),
        ("/send/abortAll", "", False),
    ]


def test_stop_telemetry_closes_socket_and_joins_thread():
    class Socket:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    class Thread:
        def __init__(self):
            self.join_timeout = None

        def join(self, timeout=None):
            self.join_timeout = timeout

    client = DJIInterface("192.168.1.42")
    client._running = True
    client._telemetry_socket = Socket()
    client._telemetry_thread = Thread()

    client.stopTelemetryStream()

    assert client._running is False
    assert client._telemetry_socket.closed is True
    assert client._telemetry_thread.join_timeout == 2
