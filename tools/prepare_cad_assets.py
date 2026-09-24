#!/usr/bin/env python3
"""Preserve supplied CAD sources and hashes; this is NOT a geometry converter."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    destination = Path(__file__).resolve().parents[1]/'simulation/cad_sources'
    files = sorted(p for p in args.source.iterdir() if p.suffix.lower() in ('.sldprt','.sldasm'))
    if not files:
        parser.error('No SolidWorks sources found')
    destination.mkdir(parents=True, exist_ok=True)
    entries = []
    for source in files:
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        target = destination/source.name
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise FileExistsError('Different CAD source already exists: '+str(target))
        if not target.exists():
            shutil.copy2(source, target)
        entries.append({'file': source.name, 'sha256': digest, 'bytes': source.stat().st_size,
                        'conversion_status': 'pending_compatible_CAD_exporter'})
    manifest = {'schema_version': 1, 'integrated_into_fsds': False,
                'required': ['compatible SolidWorks exporter', 'FSDS Unreal 4.27 editor project with assets',
                             'mesh/material import and cook; vehicle rig and physics verification'],
                'files': entries}
    (destination/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Preserved %d CAD sources; no geometry conversion claimed: %s' % (len(files),destination))


if __name__ == '__main__':
    main()
