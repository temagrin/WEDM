from dataclasses import dataclass, field


def ml(*pairs) -> field:
    return field(default_factory=lambda: [item for item in pairs])


@dataclass
class TestCaseBase:
    title: str = "Базовый кейс"
    test_paths: list = ml((17000, 6000), )
    pause_times: list = ml()
    resume_times: list = ml()
    delta_t: int = 1
    delta_t_planer: int = 100
    max_test_time: int = 100000000
    max_speed: int = 50000


@dataclass
class Square(TestCaseBase):
    title: str = "Квадрат"
    test_paths: list = ml((120, 0), (120, 120), (0, 120), (0, 0), )
    max_speed: int = 50


@dataclass
class FastRomb(TestCaseBase):
    title: str = "Ромб"
    test_paths: list = ml((90000, 30000), (120000, 120000), (30000, 90000), (0, 0))
    max_speed: int = 50000


@dataclass
class FastStar(TestCaseBase):
    title: str = "Звезда"
    test_paths: list = ml(
        (50000, 50000),
        (100000, 0),
        (-50000, 30000),
        (150000, 30000),
        (0,0)
    )
    max_speed: int = 50000


ALL_CASES = [FastStar,]
