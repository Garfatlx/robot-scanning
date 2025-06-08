import math
import zlib
import pytest
from scansegmentapi import compact_stream_extractor as se


def make_compact_telegram(lines_in_modules, sizes_of_modules):
    # 4 byte stx + 4 byte command id + 20 byte + 4 byte size of first module = 32 byte header
    buffer = se.DELIMITER + bytes(20) + sizes_of_modules[0].to_bytes(4, 'little')

    sizes_of_modules.append(0)
    for i in range(len(lines_in_modules)):
        position_module_start = len(buffer)
        buffer += bytes(sizes_of_modules[i])

        # Write number of lines in module header
        buffer = buffer[:position_module_start+20] + \
            lines_in_modules[i].to_bytes(4, 'little') + \
            buffer[position_module_start + 24:]

        # 36 bytes + 28 bytes per line
        next_module_size_position = position_module_start + 36 + 28 * lines_in_modules[i]
        # Write size of next module in module header
        buffer = buffer[:next_module_size_position] + \
            sizes_of_modules[i+1].to_bytes(4, 'little') + \
            buffer[next_module_size_position+4:]

    return buffer + zlib.crc32(buffer).to_bytes(4, 'little')


@pytest.fixture(name="stream_extractor")
def fixture_stream_extractor():
    return se.CompactStreamExtractor()


def test_extract_one_segment_from_one_contiguous_block(stream_extractor):
    telegram = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])

    assert stream_extractor.extract_data_packages(telegram) == [telegram]


def test_extract_two_segments_from_one_contiguous_block(stream_extractor):
    telegram1 = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])
    telegram2 = make_compact_telegram([4, 5, 6], [420*4, 420*5, 420*6])

    assert stream_extractor.extract_data_packages(telegram1 + telegram2) == [telegram1, telegram2]


def test_extract_two_segments_with_nonsense_infix(stream_extractor):
    telegram1 = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])
    telegram2 = make_compact_telegram([4, 5, 6], [420*4, 420*5, 420*6])

    assert stream_extractor.extract_data_packages(
        telegram1 + "Nonsense".encode() + telegram2
        ) == [telegram1, telegram2]


def test_extract_segment_with_nonsense_prefix(stream_extractor):
    telegram = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])

    assert stream_extractor.extract_data_packages("Nonsense".encode() + telegram) == [telegram]


def test_extract_segment_with_nonsense_postfix(stream_extractor):
    telegram = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])

    assert stream_extractor.extract_data_packages(telegram + "Nonsense".encode()) == [telegram]


def test_extract_segment_from_three_byte_chunks(stream_extractor):
    telegram = make_compact_telegram([1, 2, 3], [420*1, 420*2, 420*3])

    chunk_size = 3
    chunks = [
        telegram[i*chunk_size:(i+1)*chunk_size] for i in range(math.ceil(len(telegram)/chunk_size))
        ]
    for chunk in chunks[:-1]:
        assert stream_extractor.extract_data_packages(chunk) == []

    assert stream_extractor.extract_data_packages(chunks[-1]) == [telegram]
