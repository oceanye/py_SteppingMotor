"""Offline export: python scripts/export_diagnostics.py [--output path.zip]."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motor_control.diagnostic_export import export_bundle


def main():
    parser = argparse.ArgumentParser(description="导出脱敏 GUI/电机诊断包；不连接串口、不自动上传")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--label", default="field-diagnostics")
    args = parser.parse_args()
    print(export_bundle(ROOT, args.output, label=args.label))


if __name__ == "__main__":
    main()
