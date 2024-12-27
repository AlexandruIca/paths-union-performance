import time
import pathops
from enum import Enum
from typing import List, Any
from helpers import Transform, IDENTITY, read_paths_from_svg, Dimensions, write_output, skia_to_svg_path, overlap_order


def perform_union_naive(paths: List[Any]) -> Any:
    if len(paths) == 0:
        return None
    if len(paths) == 1:
        return paths[0]

    result = paths[0]

    for path in paths[1:]:
        new_path = pathops.op(result, path, pathops.PathOp.UNION)
        result = new_path

    return result


def perform_union_intervals(paths: List[Any], interval_length=100) -> Any:
    if len(paths) == 0:
        return None
    if len(paths) == 1:
        return paths[0]

    result = paths[0]

    if len(paths) > interval_length:
        results = []
        iterations = int(len(paths) / interval_length)
        for i in range(iterations + 1):
            start_index = i * interval_length
            end_index = (i + 1) * interval_length
            results.append(perform_union_naive(paths[start_index:end_index]))
        result = perform_union_intervals(results)
    else:
        result = perform_union_naive(paths)

    return result


class Version(Enum):
    SORTED_OVERLAP = -2
    DESCENDING_LENGTH = -1
    NAIVE = 0
    DIVIDE_AND_CONQUER = 1


if __name__ == '__main__':
    svg_file_path = 'Flag.svg'
    (dimensions, paths, colors) = read_paths_from_svg(svg_file_path)
    result = perform_union_intervals(paths)
    stroke_width = min(float(dimensions.width),
                       float(dimensions.height)) / 200.0
    write_output(result, dimensions, stroke_width)

    timings = []
    for length in [2, 12, 36, 50, 100, 200]:
        start = time.time()
        result = perform_union_intervals(paths, interval_length=length)
        end = time.time()

        duration = end - start
        timings.append((duration, length))

    start = time.time()
    perform_union_naive(paths)
    end = time.time()
    timings.append((end - start, Version.NAIVE))

    sorted_paths = sorted(paths, key=lambda p: len(p), reverse=True)
    start = time.time()
    perform_union_intervals(sorted_paths)
    end = time.time()
    timings.append((end - start, Version.DESCENDING_LENGTH))

    new_paths = []
    for idx in overlap_order:
        new_paths.append(paths[idx])

    start = time.time()
    perform_union_intervals(new_paths, interval_length=16)
    end = time.time()
    timings.append((end - start, Version.SORTED_OVERLAP))

    print('Results (Version: Duration):\n')

    sorted_timings = sorted(timings)
    for (t, l) in sorted_timings:
        version = ''
        match l:
            case Version.SORTED_OVERLAP:
                version = 'overlap'
            case Version.DESCENDING_LENGTH:
                version = 'descl'
            case Version.NAIVE:
                version = 'naive'
            case _:
                version = l

        print(f'\t- {version:8}: {t:.4f}s')
    print()
