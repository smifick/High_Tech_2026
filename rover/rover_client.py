from shared.ws_client import WsClient


class RoverClient(WsClient):
    def __init__(self, ws, robot_id, navigate, stop):
        super().__init__(ws, robot_id)
        self.commands = {
            "navigate": navigate,
            "stop": stop,
        }