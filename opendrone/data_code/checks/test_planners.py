import pytest

from flight_eval.planners import PLANNER_REGISTRY, get_planner_launch


# Integrated-mission planners -- each declares a mission_evaluation contract
# consumed by SequentialGoalMissionTask / ReferencePathMissionTask.
_INTEGRATED_MISSION_PLANNERS = (
    'ego_planner',
    'ego_planner_mid360',
    'ego_planner_v2',
    'fast_planner_kino',
    'fast_planner_topo',
    'super_planner_od',
    'airfar_planner',
    'airfar_planner_mid360',
)


def test_airfar_uses_upstream_pipeline_contract():
    default = get_planner_launch('airfar_planner')
    assert default['args']['config_file'] == 'default'
    assert (
        default['runtime_parameter_expectations'][
            '/airfar_local_planner/pathScale'
        ]
        == 0.5
    )
    assert '/airfar/plan_status' not in default['record_topics']
    assert '/airfar/geometric_path' not in default['record_topics']

    mid360 = get_planner_launch('airfar_planner_mid360')
    assert mid360['args']['config_file'] == 'default'
    assert (
        mid360['runtime_parameter_expectations'][
            '/airfar_local_planner/pathScale'
        ]
        == 2.0
    )
    assert (
        mid360['runtime_parameter_expectations'][
            '/terrainAnalysisExt/decayTime'
        ]
        == 10.0
    )
    assert (
        mid360['runtime_parameter_expectations'][
            '/airfar_local_planner/minPathScale'
        ]
        == 0.5
    )
    assert (
        mid360['runtime_parameter_expectations'][
            '/airfar_path_follower/cruiseSpeed'
        ]
        == 1.0
    )
    assert (
        mid360['runtime_parameter_expectations'][
            '/airfar_path_follower/goalTolerance'
        ]
        == 0.4
    )
    assert (
        mid360['runtime_parameter_expectations'][
            '/airfar_planner/GPlanner/converge_distance'
        ]
        == 0.4
    )

    overridden = get_planner_launch(
        'airfar_planner_mid360',
        {'cruise_speed': '0.8'},
    )
    assert overridden['args']['cruise_speed'] == '0.8'
    assert (
        overridden['runtime_parameter_expectations'][
            '/airfar_path_follower/cruiseSpeed'
        ]
        == 0.8
    )

    with pytest.raises(ValueError, match='与任务到达容差'):
        get_planner_launch(
            'airfar_planner_mid360',
            {'goal_tolerance': '0.35'},
        )


def test_integrated_mission_planners_advertise_mission_evaluation_block():
    for name in _INTEGRATED_MISSION_PLANNERS:
        info = get_planner_launch(name)
        assert 'mission_evaluation' in info, (
            f'{name} must declare mission_evaluation for integrated missions'
        )
        block = info['mission_evaluation']
        assert 'completion_source' in block
        # 'auto' was deprecated by the new contract; only planner/position.
        assert block['completion_source'] in {'planner', 'position'}
        # Required by every mission_evaluation block.
        assert 'fall_back_tolerance' in block
        assert block['fall_back_tolerance'] > 0.0
        # Planner-source contracts additionally declare the low-speed
        # terminal-state bound used by WaypointSequenceEvaluator to gate
        # the planner-event accept. Position-source blocks use the
        # evaluator default (0.3 m/s) implicitly.
        if block['completion_source'] == 'planner':
            assert 'max_goal_speed' in block
            assert block['max_goal_speed'] >= 0.0


def test_planner_registry_default_mission_evaluation_is_safe():
    # Planners that don't publish /planner/output at all (analytic_reference,
    # none) get an empty mission_evaluation block; the evaluator falls back
    # to position-only semantics for backward compatibility.
    for name in ('none', 'analytic_reference'):
        info = PLANNER_REGISTRY[name]
        assert info.get('mission_evaluation', {}) == {}


def test_topo_only_runs_reference_path_mission():
    # Fast-Planner Topo has no per-waypoint progression: it consumes the
    # Path as a single global reference. It must therefore be excluded
    # from sequential_goal_mission (which requires per-stop evidence) and
    # only show up under reference_path_mission.
    topo_tasks = PLANNER_REGISTRY['fast_planner_topo']['tasks']
    assert 'reference_path_mission' in topo_tasks
    assert 'sequential_goal_mission' not in topo_tasks


def test_kino_runs_sequential_goal_mission_but_not_reference_path():
    kino_tasks = PLANNER_REGISTRY['fast_planner_kino']['tasks']
    assert 'sequential_goal_mission' in kino_tasks
    assert 'reference_path_mission' not in kino_tasks


def test_ego_and_airfar_use_position_completion_source():
    # EGO family and Air-FAR physically pause at each waypoint, so the
    # position path is authoritative; planner events are diagnostic only.
    for name in ('ego_planner', 'ego_planner_mid360', 'ego_planner_v2',
                 'airfar_planner', 'airfar_planner_mid360'):
        info = get_planner_launch(name)
        assert info['mission_evaluation']['completion_source'] == 'position', (
            f'{name} expected to use position source for completion'
        )


def test_kino_topo_and_super_use_planner_completion_source():
    # Kino, Topo and SUPER may not physically reach every waypoint (Kino
    # is time-only, SUPER can re-route around obstacles). They must rely
    # on the planner's own WAYPOINT/REACHED events for completion.
    for name in ('fast_planner_kino', 'fast_planner_topo', 'super_planner_od'):
        info = get_planner_launch(name)
        assert info['mission_evaluation']['completion_source'] == 'planner', (
            f'{name} expected to use planner source for completion'
        )
