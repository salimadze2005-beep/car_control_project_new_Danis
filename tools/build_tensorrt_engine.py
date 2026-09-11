#!/usr/bin/env python3
"""Build a target-specific TensorRT engine with NVIDIA's trtexec."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys


DEFAULT_TRTEXEC = Path('/usr/src/tensorrt/bin/trtexec')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def locate_trtexec(explicit=None):
    if explicit:
        candidate = Path(explicit).expanduser()
        if candidate.is_file():
            return candidate.resolve()
        raise FileNotFoundError('trtexec not found: %s' % candidate)
    found = shutil.which('trtexec')
    if found:
        return Path(found).resolve()
    if DEFAULT_TRTEXEC.is_file():
        return DEFAULT_TRTEXEC
    raise FileNotFoundError(
        'trtexec not found. Install TensorRT from the JetPack matching this Jetson.')


def workspace_argument(help_text, workspace_mib):
    if '--memPoolSize' in help_text:
        return '--memPoolSize=workspace:%d' % workspace_mib
    return '--workspace=%d' % workspace_mib


def build_command(trtexec, onnx, engine, fp16, workspace_mib, help_text):
    command = [
        str(trtexec),
        '--onnx=%s' % onnx,
        '--saveEngine=%s' % engine,
        workspace_argument(help_text, workspace_mib),
    ]
    if fp16:
        command.append('--fp16')
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Build best.onnx on the target Jetson; engines are not portable between TensorRT/GPU versions.')
    parser.add_argument('onnx', type=Path)
    parser.add_argument('engine', type=Path)
    parser.add_argument('--trtexec')
    parser.add_argument('--fp32', action='store_true', help='Disable the default FP16 build')
    parser.add_argument('--workspace-mib', type=int, default=1024)
    parser.add_argument('--expected-sha256')
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args(argv)

    onnx = args.onnx.expanduser().resolve()
    engine = args.engine.expanduser().resolve()
    if not onnx.is_file():
        parser.error('ONNX file does not exist: %s' % onnx)
    if onnx.suffix.lower() != '.onnx' or engine.suffix.lower() != '.engine':
        parser.error('Use an .onnx input and an .engine output')
    if onnx == engine:
        parser.error('Input and output must be different files')
    if args.workspace_mib <= 0:
        parser.error('--workspace-mib must be positive')
    if engine.exists() and not args.force:
        parser.error('Engine already exists; choose another path or pass --force')

    actual_hash = sha256(onnx)
    if args.expected_sha256 and actual_hash.lower() != args.expected_sha256.lower():
        parser.error('ONNX SHA-256 mismatch: %s' % actual_hash)

    try:
        trtexec = locate_trtexec(args.trtexec)
        help_result = subprocess.run(
            [str(trtexec), '--help'], check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        command = build_command(
            trtexec, onnx, engine, not args.fp32,
            args.workspace_mib, help_result.stdout)
        print('ONNX SHA-256:', actual_hash)
        print('Building on this Jetson:', ' '.join(command))
        subprocess.run(command, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        print('ERROR:', error, file=sys.stderr)
        return 1

    if not engine.is_file() or engine.stat().st_size == 0:
        print('ERROR: trtexec did not produce a non-empty engine', file=sys.stderr)
        return 1
    print('Engine:', engine)
    print('Engine SHA-256:', sha256(engine))
    print('trtexec completed its default inference benchmark successfully.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
