'''Build and optionally reproduce the curated, math-only research handoff ZIP.

No application startup, serial access, configuration writes or motor commands.
Run: python -B scripts/package_gait_research.py --verify
'''
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ASSET = Path('docs/assets/gait_avoidance_2026-09-24')
REPORT = Path('docs/RESEARCH_2026-09-24_GAIT_SPIN_ORBIT_AVOIDANCE.md')
BRIEF = Path('docs/EXTERNAL_BRIEF_2026-09-24_GAIT_CLEARANCE_OPTIMIZATION.md')
HANDOFF = Path('docs/HANDOFF_2026-09-24_GAIT_RESEARCH_BRANCH.md')
BUNDLE = ROOT / 'docs/handoff_gait_avoidance_research_2026-09-24.zip'
FILES = [REPORT, BRIEF, HANDOFF, Path('scripts/research_gait_clearance.py'), Path(__file__).relative_to(ROOT)]
REFERENCES = [
    'motor_control/gait_avoidance.py', 'motor_control/gait_planner.py',
    'motor_control/gait_executor.py', 'motor_control/desktop_app.py',
    'motor_control/ui/gait_tab.py', 'esp32_stepper/src/sync_math.h',
    'esp32_stepper/src/stepper.cpp', 'tests/test_gait_avoidance.py',
    'docs/handoff_control_method_utf8.txt', 'docs/handoff_motor_control_check.txt',
    'docs/HANDOFF_2026-09-23_TWO_MODE_LEG_AVOIDANCE.md',
]
EXPECTED_ROWS = {
    'low_trace.csv': 6001, 'high_trace.csv': 6001,
    'four_actions_trace.csv': 24004, 'high_candidate_trace.csv': 6001,
    'execution_knots.csv': 124, 'critical_pose_pairs.csv': 12,
    'configuration_grid.csv': 58322, 'pointwise_phase_grid_envelope.csv': 242,
    'high_parameter_scan_coarse.csv': 315, 'high_parameter_scan_refined.csv': 616,
    'sampling_convergence.csv': 12,
}
README = '''# 三足腿—高杆避让：独立研究资料包

2026-09-24；生产源码基线 57dc02bd1a0c40d41c7845e8940a3d7d9f5b4de3。

主线负责人先阅读 docs/HANDOFF_2026-09-24_GAIT_RESEARCH_BRANCH.md。
本分支交付控制逻辑分析及研究证据，未修改生产控制代码，未部署候选参数。

先阅读 docs/EXTERNAL_BRIEF_2026-09-24_GAIT_CLEARANCE_OPTIMIZATION.md，
再阅读 docs/RESEARCH_2026-09-24_GAIT_SPIN_ORBIT_AVOIDANCE.md。
CSV、JSON及五组SVG/PNG图均在 docs/assets/gait_avoidance_2026-09-24/。

Python 3.10+，在解压后的根目录运行：

    python -B scripts/research_gait_clearance.py

仅需标准库。重生成CSV/JSON/SVG，不自动刷新PNG；PNG是随包提供的原始快照。
没有电机连接、串口、GUI、配置写入或固件刷写。请不要将此包当作完整应用。
几何模块 gait_avoidance.py、gait_planner.py 原样复制。
仅本ZIP的 motor_control/__init__.py 使用数学研究专用的空初始化文件，
避免加载完整应用依赖；工作区生产初始化文件没有修改。
其他控制/GUI/固件源码和测试为审阅参考，不保证可在裁剪包中独立运行。

重点问题：分别优化LOW/HIGH全程最小间隙；逐姿态最大值不等于安全路径。
当前n=18未改动；n=19.6只是有限族离线候选，未证明全局最优，未部署。
图中距离为腿中心线到高杆轴线的距离/R，尚未扣除实体厚度和误差。
现场标定参数缺失，不可凭此批准实机运行。

MANIFEST.sha256列出所有其他包内文件的SHA-256。
文档中的资料包下载链接指向本ZIP自身，解压目录内不会重复嵌套该ZIP。
不包含.git、虚拟环境、机器配置、账户凭据或整套运行环境。
'''


def validate_source():
    metadata = json.loads((ROOT/ASSET/'model_and_results.json').read_text(encoding='utf-8'))
    for name, expected in metadata['sources_sha256'].items():
        actual = hashlib.sha256((ROOT/Path(name.replace('\\', '/'))).read_bytes()).hexdigest()
        assert actual == expected, f'Stale generated data: {name}; rerun research script'
    for name, expected in EXPECTED_ROWS.items():
        with (ROOT/ASSET/name).open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.reader(stream))
        assert len(rows)-1 == expected, (name, len(rows)-1, expected)
    # Check local links in the authored documents; the ZIP self-link is
    # supplied by the output package, not recursively embedded in itself.
    for document in [REPORT, BRIEF, HANDOFF]:
        content = (ROOT/document).read_text(encoding='utf-8')
        for target in re.findall(r'\]\(([^)]+)\)', content):
            if '://' in target or target.startswith('#'):
                continue
            resolved = (ROOT/document.parent/target.split('#')[0]).resolve()
            assert resolved == BUNDLE or resolved.exists(), (document, target)


def build():
    validate_source()
    paths = FILES+[Path(p) for p in REFERENCES]
    paths += [ASSET/name for name in EXPECTED_ROWS]
    paths += [ASSET/'model_and_results.json']
    for stem in ['01_angle_curves', '02_clearance_curves', '03_configuration_space',
                 '04_extremum_poses', '05_parameter_scan']:
        paths.extend(ASSET/(stem+suffix) for suffix in ['.svg', '.png'])
    entries = {p.as_posix(): (ROOT/p).read_bytes() for p in paths}
    entries['README_FIRST.md'] = README.encode('utf-8')
    entries['motor_control/__init__.py'] = b'# Math-only archive initializer; NOT the production package initializer.\n'
    manifest = ''.join(f'{hashlib.sha256(data).hexdigest()}  {name}\n'
                       for name, data in sorted(entries.items()))
    entries['MANIFEST.sha256'] = manifest.encode('utf-8')
    with zipfile.ZipFile(BUNDLE, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name, data in sorted(entries.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 9, 24, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, data, compresslevel=9)
    with zipfile.ZipFile(BUNDLE) as archive:
        assert archive.testzip() is None
        for line in archive.read('MANIFEST.sha256').decode('utf-8').splitlines():
            digest, name = line.split('  ', 1)
            assert hashlib.sha256(archive.read(name)).hexdigest() == digest, name
    print(f'Built {BUNDLE}: {len(entries)} files, {BUNDLE.stat().st_size} bytes', flush=True)
    print('SHA256:', hashlib.sha256(BUNDLE.read_bytes()).hexdigest(), flush=True)


def verify_reproduction():
    # Fresh, task-owned temporary directory, removed automatically on exit.
    # Verify only our curated ZIP; no user-provided archive is accepted.
    with tempfile.TemporaryDirectory(prefix='gait-research-verify-') as folder:
        destination = Path(folder)
        with zipfile.ZipFile(BUNDLE) as archive:
            for name in archive.namelist():
                relative = PurePosixPath(name)
                assert not relative.is_absolute() and '..' not in relative.parts and ':' not in name
            archive.extractall(destination)
        result = subprocess.run([sys.executable, '-B', 'scripts/research_gait_clearance.py'],
                                cwd=destination, text=True, capture_output=True, timeout=180)
        if result.returncode:
            raise RuntimeError(result.stdout+'\n'+result.stderr)
        baseline = json.loads((ROOT/ASSET/'model_and_results.json').read_text(encoding='utf-8'))
        reproduced = json.loads((destination/ASSET/'model_and_results.json').read_text(encoding='utf-8'))
        for key in ['baselines', 'high_candidate', 'checks', 'rods_xy_over_R', 'search']:
            assert baseline[key] == reproduced[key], key
        # With the same interpreter/platform, generated numerical files should
        # reproduce byte-for-byte. JSON's Git availability field may differ.
        for original in (ROOT/ASSET).iterdir():
            if original.suffix in {'.csv', '.svg'}:
                assert original.read_bytes() == (destination/ASSET/original.name).read_bytes(), original.name
        print('Fresh extracted ZIP reproduced all CSV/SVG and numerical summaries exactly.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify', action='store_true', help='Reproduce in a fresh temporary extraction')
    args = parser.parse_args()
    build()
    if args.verify:
        verify_reproduction()


if __name__ == '__main__':
    main()
