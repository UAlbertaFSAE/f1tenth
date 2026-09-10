from __future__ import annotations

from enum import IntEnum
from typing import Callable


class IntegratorType(IntEnum):
    """Available integration schemes for environment dynamics."""

    EULER = 1
    RK4 = 2

    def integration_fn(self) -> Callable:
        """Return the concrete integrator function associated with this enum."""
        if self is IntegratorType.EULER:
            return euler_integration
        if self is IntegratorType.RK4:
            return rk4_integration
        raise ValueError(f"Unsupported integrator type: {self}")

    @classmethod
    def from_type(cls, integrator_type: "IntegratorType") -> Callable:
        """Preserve the historic API that exposed a class-level lookup."""
        if not isinstance(integrator_type, IntegratorType):
            raise TypeError(
                f"integrator_type must be an IntegratorType, got {integrator_type!r}"
            )
        return integrator_type.integration_fn()


def integrator_from_type(integrator_type: IntegratorType) -> Callable:
    """Public helper mirroring the historic functional API."""
    return IntegratorType.from_type(integrator_type)


def rk4_integration(f, x, u, dt, *args):
    k1 = f(x, u, *args)

    k2_state = x + dt * (k1 / 2)
    k2 = f(k2_state, u, *args)

    k3_state = x + dt * (k2 / 2)
    k3 = f(k3_state, u, *args)

    k4_state = x + dt * k3
    k4 = f(k4_state, u, *args)

    # dynamics integration
    x = x + dt * (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def euler_integration(f, x, u, dt, *args):
    x = x + dt * f(x, u, *args)
    return x