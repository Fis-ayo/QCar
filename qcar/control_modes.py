"""Explicit actuator-authority policies for controlled QCar experiments."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlMode:
    name: str
    safe_speed_control: bool
    regularized_stanley_speed: bool
    localization_stop: bool
    tracking_loss_stop: bool
    camera_steering: bool

    @property
    def boundary_speed_control(self):
        # Boundary classifications remain measurement-only in every cleaned
        # mode. They must not obscure waypoint-versus-hybrid comparisons.
        return self.name == "preventive_stop"

    @property
    def non_camera_signature(self):
        """Fields that must match for a valid waypoint-versus-hybrid A/B."""
        return (
            self.safe_speed_control,
            self.regularized_stanley_speed,
            self.localization_stop,
            self.tracking_loss_stop,
            self.boundary_speed_control,
        )


MODES = {
    "baseline": ControlMode(
        name="baseline",
        safe_speed_control=False,
        regularized_stanley_speed=False,
        localization_stop=False,
        tracking_loss_stop=False,
        camera_steering=False,
    ),
    "baseline_safety": ControlMode(
        name="baseline_safety",
        safe_speed_control=True,
        regularized_stanley_speed=True,
        localization_stop=True,
        tracking_loss_stop=True,
        camera_steering=False,
    ),
    "preventive_stop": ControlMode(
        name="preventive_stop",
        safe_speed_control=True,
        regularized_stanley_speed=True,
        localization_stop=True,
        tracking_loss_stop=True,
        camera_steering=False,
    ),
    "hybrid": ControlMode(
        name="hybrid",
        safe_speed_control=True,
        regularized_stanley_speed=True,
        localization_stop=True,
        tracking_loss_stop=True,
        camera_steering=True,
    ),
}


def get_control_mode(name):
    try:
        return MODES[str(name)]
    except KeyError as exc:
        raise ValueError(
            f"Unknown control mode {name!r}; choose from {sorted(MODES)}"
        ) from exc
