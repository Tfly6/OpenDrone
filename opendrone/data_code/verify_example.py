"""Verify the small synthetic tracking example using only the standard library."""
import csv
import json
import math
from pathlib import Path


def main():
    directory = Path(__file__).resolve().parent / 'samples'
    with (directory / 'tracking_reference.csv').open() as stream:
        reference = {float(r['time']): [float(r[k]) for k in ('x','y','z')] for r in csv.DictReader(stream)}
    errors = []
    with (directory / 'tracking_state.csv').open() as stream:
        for row in csv.DictReader(stream):
            target = reference[float(row['time'])]
            errors.append(math.sqrt(sum((float(row[k])-target[i])**2 for i,k in enumerate(('x','y','z')))))
    expected = json.loads((directory / 'expected.json').read_text())
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    ordered = sorted(errors)
    q = .95*(len(ordered)-1)
    i = int(q)
    p95 = ordered[i] + (ordered[min(i+1,len(ordered)-1)]-ordered[i])*(q-i)
    assert len(errors) == expected['sample_count']
    assert abs(rmse-expected['rmse_m']) < 1e-12
    assert abs(p95-expected['p95_m']) < 1e-12
    print(f'Synthetic example verified: {len(errors)} samples, RMSE={rmse:.3f} m, P95={p95:.3f} m')


if __name__ == '__main__':
    main()
