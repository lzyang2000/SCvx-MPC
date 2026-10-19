"""Rigid axle geometry for a flat approach joining an inside circular ramp."""
import math
import numpy as np


class RampContactGeometry:
    def __init__(self, radius, end_angle, wheelbase, tire_radius, hub_drop):
        self.radius = radius
        self.half_base = wheelbase / 2
        self.entry_s = -self.half_base
        hub_radius = radius - tire_radius
        self.entry_angle = math.asin(self.half_base / hub_radius)
        self.orbit_radius = math.sqrt(hub_radius**2 - self.half_base**2) - hub_drop
        # Entry: front axle is on the circle; rear axle is still on flat ground.
        front_angle = np.linspace(0., 2*self.entry_angle, 257)
        front_x = hub_radius*np.sin(front_angle)
        front_z = radius - hub_radius*np.cos(front_angle)
        dz = front_z - tire_radius
        dx = np.sqrt(wheelbase**2 - dz**2)
        pitch = np.arctan2(dz, dx)
        x = front_x - dx/2 - hub_drop*np.sin(pitch)
        z = (front_z + tire_radius)/2 + hub_drop*np.cos(pitch)
        distance = np.r_[0., np.cumsum(np.hypot(np.diff(x), np.diff(z)))]
        self.entry_length = distance[-1]
        tx, tz = np.gradient(x, distance), np.gradient(z, distance)
        norm = np.hypot(tx, tz)
        tx, tz = tx/norm, tz/norm
        curvature = np.gradient(np.unwrap(np.arctan2(tz, tx)), distance)
        self.entry = (distance, x, z, pitch, tx, tz, np.gradient(pitch, distance), curvature)
        self.end_angle = end_angle
        self.release_s = self.entry_s + self.entry_length + self.orbit_radius*(end_angle-self.entry_angle)
        self.flat_height = hub_drop + tire_radius

    def at(self, path_s):
        """Return COM, path tangent, body pitch, pitch per meter and curvature."""
        if path_s < self.entry_s:
            return (np.array([path_s, 0., self.flat_height]), np.array([1., 0., 0.]), 0., 0., 0.)
        distance = path_s - self.entry_s
        if distance < self.entry_length:
            station, *fields = self.entry
            x, z, pitch, tx, tz, pitch_rate, curvature = [np.interp(distance, station, f) for f in fields]
            tangent = np.array([tx, 0., tz])
            tangent /= np.linalg.norm(tangent)
            return np.array([x, 0., z]), tangent, pitch, pitch_rate, curvature
        pitch = min(self.entry_angle + (distance-self.entry_length)/self.orbit_radius, self.end_angle)
        position = np.array([self.orbit_radius*math.sin(pitch), 0.,
                             self.radius-self.orbit_radius*math.cos(pitch)])
        return position, np.array([math.cos(pitch), 0., math.sin(pitch)]), pitch, 1/self.orbit_radius, 1/self.orbit_radius
