#!/usr/bin/env python3
"""Task outcome models and algorithm-independent completion evaluators."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, Optional, Sequence, Tuple


MISSION_PATH_TOPIC = '/waypoint_generator/waypoints'
TRAJECTORY_TRIGGER_TOPIC = '/waypoint_generator/traj_start_trigger'
MISSION_GOAL_TOLERANCE = 0.4
MISSION_GOAL_DWELL_TIME = 0.5


class TaskOutcomeStatus(str, Enum):
    """Coarse task result; independent from runner/infrastructure status."""

    SUCCEEDED = 'succeeded'
    NOT_SUCCEEDED = 'not_succeeded'
    UNKNOWN = 'unknown'
    NOT_APPLICABLE = 'not_applicable'


@dataclass(frozen=True)
class TaskOutcome:
    status: TaskOutcomeStatus
    reason: str
    completion_time: Optional[float] = None
    evidence: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {
            'status': self.status.value,
            'reason': self.reason,
            'completion_time': self.completion_time,
            'evidence': dict(self.evidence),
        }

    @classmethod
    def from_dict(cls, value: Dict[str, object]) -> 'TaskOutcome':
        return cls(
            status=TaskOutcomeStatus(str(value.get('status', 'unknown'))),
            reason=str(value.get('reason', 'insufficient_evidence')),
            completion_time=(
                float(value['completion_time'])
                if value.get('completion_time') is not None
                else None
            ),
            evidence=dict(value.get('evidence') or {}),
        )


class NoOutcomeEvaluator:
    """Evaluator for continuous tasks without a semantic terminal goal."""

    @property
    def terminal(self) -> bool:
        return False

    @property
    def outcome(self) -> TaskOutcome:
        return TaskOutcome(
            TaskOutcomeStatus.NOT_APPLICABLE,
            'continuous_task',
        )

    def start(self, now: float) -> None:
        del now

    def update_path(
        self,
        points: Iterable[Sequence[float]],
        now: float,
        frame_id: str = '',
    ) -> None:
        del points, now, frame_id

    def update_position(
        self,
        position: Sequence[float],
        now: float,
        frame_id: str = '',
    ) -> None:
        del position, now, frame_id

    def timeout(self, now: float) -> TaskOutcome:
        del now
        return self.outcome

    def emergency(self, now: float) -> TaskOutcome:
        del now
        return self.outcome

    def collision_limit(
        self,
        now: float,
        count: int,
        limit: int,
    ) -> TaskOutcome:
        del now, count, limit
        return self.outcome

    def mark_unknown(self, reason: str) -> TaskOutcome:
        del reason
        return self.outcome


class PathGoalEvaluator:
    """根据原始 Path 和机体位姿判定整条路径任务。"""

    def __init__(
        self,
        goal_tolerance: float = MISSION_GOAL_TOLERANCE,
        progress_tolerance: float = 0.5,
        dwell_time: float = MISSION_GOAL_DWELL_TIME,
    ):
        if goal_tolerance <= 0.0:
            raise ValueError('goal_tolerance 必须大于 0')
        if progress_tolerance < 0.0:
            raise ValueError('progress_tolerance 不能小于 0')
        if dwell_time < 0.0:
            raise ValueError('dwell_time 不能小于 0')
        self.goal_tolerance = float(goal_tolerance)
        self.progress_tolerance = float(progress_tolerance)
        self.dwell_time = float(dwell_time)
        self._points: Tuple[Tuple[float, float, float], ...] = ()
        self._cumulative_lengths: Tuple[float, ...] = ()
        self._start_time: Optional[float] = None
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._path_frame = ''
        self._position_frame = ''
        self._max_progress = 0.0
        self._inside_since: Optional[float] = None
        self._outcome = TaskOutcome(
            TaskOutcomeStatus.UNKNOWN,
            'awaiting_evidence',
        )

    @property
    def terminal(self) -> bool:
        return self._outcome.status in {
            TaskOutcomeStatus.SUCCEEDED,
            TaskOutcomeStatus.NOT_SUCCEEDED,
        }

    @property
    def outcome(self) -> TaskOutcome:
        return self._outcome

    @staticmethod
    def _point(value: Sequence[float]) -> Tuple[float, float, float]:
        if len(value) < 3:
            raise ValueError('三维位置必须至少包含 x/y/z')
        point = (float(value[0]), float(value[1]), float(value[2]))
        if not all(math.isfinite(item) for item in point):
            raise ValueError('路径和位置必须是有限数值')
        return point

    @staticmethod
    def _distance(
        left: Sequence[float],
        right: Sequence[float],
    ) -> float:
        return math.sqrt(sum(
            (float(left[index]) - float(right[index])) ** 2
            for index in range(3)
        ))

    def start(self, now: float) -> None:
        if self._start_time is None:
            self._start_time = float(now)
        if self._last_position is not None:
            self._evaluate(self._last_position, float(now))

    def update_path(
        self,
        points: Iterable[Sequence[float]],
        now: float,
        frame_id: str = '',
    ) -> None:
        del now
        normalized = tuple(self._point(point) for point in points)
        normalized_frame = str(frame_id or '')
        if normalized == self._points and normalized_frame == self._path_frame:
            return

        self._points = normalized
        self._path_frame = normalized_frame
        cumulative = [0.0]
        for index in range(1, len(normalized)):
            cumulative.append(
                cumulative[-1] + self._distance(normalized[index - 1], normalized[index])
            )
        self._cumulative_lengths = tuple(cumulative)
        self._max_progress = 0.0
        self._inside_since = None
        if not self.terminal:
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                'awaiting_position' if normalized else 'missing_mission_path',
                evidence={'waypoint_count': len(normalized)},
            )

    def update_position(
        self,
        position: Sequence[float],
        now: float,
        frame_id: str = '',
    ) -> None:
        self._last_position = self._point(position)
        self._position_frame = str(frame_id or '')
        if not self.terminal:
            self._evaluate(self._last_position, float(now))

    def _frames_match(self) -> bool:
        return (
            not self._path_frame
            or not self._position_frame
            or self._path_frame == self._position_frame
        )

    def _closest_progress(self, position: Sequence[float]) -> float:
        if len(self._points) <= 1:
            return 0.0

        best_distance_sq = float('inf')
        best_progress = 0.0
        for index in range(len(self._points) - 1):
            left = self._points[index]
            right = self._points[index + 1]
            segment = tuple(right[axis] - left[axis] for axis in range(3))
            segment_norm_sq = sum(value * value for value in segment)
            ratio = 0.0
            if segment_norm_sq > 1e-12:
                offset = tuple(position[axis] - left[axis] for axis in range(3))
                ratio = sum(
                    offset[axis] * segment[axis] for axis in range(3)
                ) / segment_norm_sq
                ratio = max(0.0, min(1.0, ratio))
            projection = tuple(
                left[axis] + ratio * segment[axis] for axis in range(3)
            )
            distance_sq = sum(
                (position[axis] - projection[axis]) ** 2 for axis in range(3)
            )
            progress = (
                self._cumulative_lengths[index]
                + math.sqrt(segment_norm_sq) * ratio
            )
            if distance_sq < best_distance_sq - 1e-9:
                best_distance_sq = distance_sq
                best_progress = progress
            elif (
                abs(distance_sq - best_distance_sq) <= 1e-9
                and (
                    abs(progress - self._max_progress),
                    -progress if self._max_progress > 0.0 else progress,
                )
                < (
                    abs(best_progress - self._max_progress),
                    -best_progress if self._max_progress > 0.0 else best_progress,
                )
            ):
                best_progress = progress
        return best_progress

    def _evidence(self, position: Sequence[float]) -> Dict[str, object]:
        total_length = (
            self._cumulative_lengths[-1] if self._cumulative_lengths else 0.0
        )
        final_distance = (
            self._distance(position, self._points[-1])
            if self._points
            else None
        )
        return {
            'waypoint_count': len(self._points),
            'goal_tolerance': self.goal_tolerance,
            'progress_tolerance': self.progress_tolerance,
            'dwell_time': self.dwell_time,
            'path_length': total_length,
            'max_path_progress': self._max_progress,
            'remaining_path': max(0.0, total_length - self._max_progress),
            'final_goal_distance': final_distance,
            'path_frame': self._path_frame,
            'position_frame': self._position_frame,
        }

    def _evaluate(self, position: Sequence[float], now: float) -> None:
        if not self._points:
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                'missing_mission_path',
            )
            return
        if self._start_time is None:
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                'not_started',
                evidence=self._evidence(position),
            )
            return
        if not self._frames_match():
            self._inside_since = None
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                'frame_mismatch',
                evidence=self._evidence(position),
            )
            return

        self._max_progress = max(
            self._max_progress,
            self._closest_progress(position),
        )
        evidence = self._evidence(position)
        at_goal = evidence['final_goal_distance'] <= self.goal_tolerance
        near_path_end = evidence['remaining_path'] <= self.progress_tolerance
        if at_goal and near_path_end:
            if self._inside_since is None:
                self._inside_since = now
            if now - self._inside_since >= self.dwell_time:
                self._outcome = TaskOutcome(
                    TaskOutcomeStatus.SUCCEEDED,
                    'goal_reached',
                    completion_time=max(0.0, now - self._start_time),
                    evidence=evidence,
                )
                return
        else:
            self._inside_since = None

        self._outcome = TaskOutcome(
            TaskOutcomeStatus.UNKNOWN,
            'in_progress',
            evidence=evidence,
        )

    def timeout(self, now: float) -> TaskOutcome:
        del now
        if self.terminal:
            return self._outcome
        if not self._points or self._last_position is None or not self._frames_match():
            reason = (
                'missing_mission_path'
                if not self._points
                else (
                    'missing_position'
                    if self._last_position is None
                    else 'frame_mismatch'
                )
            )
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                reason,
                evidence=(
                    self._evidence(self._last_position)
                    if self._last_position is not None
                    else {'waypoint_count': len(self._points)}
                ),
            )
        else:
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.NOT_SUCCEEDED,
                'timeout',
                evidence=self._evidence(self._last_position),
            )
        return self._outcome

    def emergency(self, now: float) -> TaskOutcome:
        del now
        if not self.terminal:
            evidence = (
                self._evidence(self._last_position)
                if self._last_position is not None
                else {'waypoint_count': len(self._points)}
            )
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.NOT_SUCCEEDED,
                'emergency',
                evidence=evidence,
            )
        return self._outcome

    def collision_limit(
        self,
        now: float,
        count: int,
        limit: int,
    ) -> TaskOutcome:
        del now
        if not self.terminal:
            evidence = (
                self._evidence(self._last_position)
                if self._last_position is not None
                else {'waypoint_count': len(self._points)}
            )
            evidence.update({
                'collision_episode_count': int(count),
                'collision_episode_limit': int(limit),
            })
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.NOT_SUCCEEDED,
                'collision_limit',
                evidence=evidence,
            )
        return self._outcome

    def mark_unknown(self, reason: str) -> TaskOutcome:
        if not self.terminal:
            evidence = (
                self._evidence(self._last_position)
                if self._last_position is not None
                else {'waypoint_count': len(self._points)}
            )
            self._outcome = TaskOutcome(
                TaskOutcomeStatus.UNKNOWN,
                reason,
                evidence=evidence,
            )
        return self._outcome
