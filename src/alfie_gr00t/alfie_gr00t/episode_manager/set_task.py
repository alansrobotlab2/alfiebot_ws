#!/usr/bin/env python3
"""Rewrite the task_index column of one episode parquet.

    python set_task.py <episode.parquet> <task_index> [<task_id> <fps> [<tasks.yaml>]]

Run by the episode manager under the GR00T interpreter, because the node itself
runs on system python, which has no pyarrow. With a task ID it also prints the
episode's grasp and done frames for that task as a line `FRAMES {json}`; that
part needs alfie_gr00t importable and is skipped (with a note) when it is not.
"""

import json
import os
import sys

import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq


def main() -> None:
    path, task_index = sys.argv[1], int(sys.argv[2])
    table = pq.read_table(path)
    column = table.schema.get_field_index('task_index')
    if column < 0:
        sys.exit(f'{path} has no task_index column')
    field = table.schema.field(column)
    table = table.set_column(column, field,
                             pa.array([task_index] * table.num_rows, type=field.type))

    tmp = path + '.tmp'
    pq.write_table(table, tmp)
    os.replace(tmp, path)

    values = pc.unique(pq.read_table(path, columns=['task_index'])['task_index']).to_pylist()
    if values not in ([task_index], []):
        sys.exit(f'{os.path.basename(path)} verify failed: task_index={values}')
    print(f'{os.path.basename(path)}: task_index={task_index} ({table.num_rows} rows, verified)')

    if len(sys.argv) >= 5:
        task_id, fps = sys.argv[3], float(sys.argv[4])
        try:
            from alfie_gr00t import task_registry
            registry = task_registry.load(sys.argv[5] if len(sys.argv) > 5 else None)
            states = pq.read_table(path, columns=['observation.state'])[
                'observation.state'].to_pylist()
            print('FRAMES ' + json.dumps(registry.episode_frames(task_id, states, 1.0 / fps)))
        except Exception as exc:   # noqa: BLE001 - the relabel itself is done
            print(f'grasp/done frames not recomputed: {exc}')


if __name__ == '__main__':
    main()
