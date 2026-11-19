"""Apply archived task parameters after each planner's ordinary launch tree."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from .runtime import RevisionFlightRunner


class SuiteFlightRunner(RevisionFlightRunner):
    def __init__(self, *args, **kwargs):
        import yaml
        planner_args = dict(kwargs.get('planner_args') or {})
        task_file = planner_args.pop('_paper_task_config', None)
        kwargs['planner_args'] = planner_args
        super().__init__(*args, **kwargs)
        self.paper_waypoints = yaml.safe_load(Path(task_file).read_text()) if task_file else {}
        if not task_file or not self.planner_info.get('launch_file'):
            return
        self.planner_info = copy.deepcopy(self.planner_info)
        info = self.planner_info
        original = self._resolve_launch_path(info['launch_pkg'], info['launch_file'])
        if original is None:
            raise FileNotFoundError(info['launch_file'])
        launch = ET.Element('launch')
        arguments = info.get('args', {})
        for key, value in arguments.items():
            ET.SubElement(launch, 'arg', name=key, default=str(value))
        include = ET.SubElement(launch, 'include', file=original)
        for key in arguments:
            ET.SubElement(include, 'arg', name=key, value='$(arg ' + key + ')')
        # Loading after the include overrides private node params in ROS1.
        ET.SubElement(launch, 'rosparam', command='load', file=str(task_file), ns='waypoint_generator')
        payload = ET.tostring(launch, encoding='utf-8', xml_declaration=True)
        wrapper = Path(task_file).parent.parent / 'launches' / (hashlib.sha256(payload).hexdigest()[:16] + '.launch')
        wrapper.parent.mkdir(parents=True, exist_ok=True)
        wrapper.write_bytes(payload)
        info['launch_file'] = str(wrapper)
