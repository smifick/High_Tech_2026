from shared.ws_client import WsClient


class DroneClient(WsClient):
    def __init__(self, ws, robot_id, navigate, land):
        super().__init__(ws, robot_id)
        self.commands = {
            "navigate": navigate,
            "land": land,
        }