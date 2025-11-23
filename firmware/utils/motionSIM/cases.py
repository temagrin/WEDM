from dataclasses import dataclass, field


def ml(*items)->field:
    return field(default_factory=lambda: [item for item in items])

@dataclass
class TestCaseBase:
    title: str = "Базовый кейс"
    test_paths: list= ml((1700,600),)
    pause_times: list = ml()
    resume_times: list = ml()
    delta_t: int = 1
    delta_t_planer: int = 100
    max_test_time: int = 100000000
    max_speed: int = 10000

@dataclass
class Square(TestCaseBase):
    title: str = "Квадрат"
    test_paths: list= ml((1200,0),(1200,1200),(0,1200),(0,0),)

ALL_CASES = [TestCaseBase(), Square()]