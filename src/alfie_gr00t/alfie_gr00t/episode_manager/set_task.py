#!/usr/bin/env python3
"""Rewrite the task_index column of one episode parquet.

    python set_task.py <episode.parquet> <task_index>

Run by the episode manager under the GR00T interpreter, because the node itself
runs on system python, which has no pyarrow. Kept free of alfie_gr00t imports
so it works however that interpreter's sys.path is set up.
"""

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


if __name__ == '__main__':
    main()
