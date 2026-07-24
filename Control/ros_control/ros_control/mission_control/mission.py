from model.vehical_model import Vehicle_State
from nav_msgs.msg import Path

from abc import ABC, abstractmethod
from typing import Optional


class Mission(ABC):

    progress_flag = 0
    complete = False

    def __init__(self, logger, trigger_ebs):
        super().__init__()

        self.logger = logger#ros2 logger passed through from cmd node
        self.trigger_ebs = trigger_ebs# trigger ebs function passed through from cmd node

    def reset(self):
        self.progress_flag = 0
        self.complete = False

    def is_complete(self) -> bool:
        return self.complete

    @abstractmethod
    def get_commands(
        self, 
        current_state:Vehicle_State,
        desired_path: Optional[Path] = None
    ) -> tuple[float, float]:
        pass
