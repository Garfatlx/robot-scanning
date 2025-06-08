import math
import zlib
import pytest
from scansegmentapi import msgpack_stream_extractor as se


def make_msgpack_telegram(msgpack_buffer):
    buffer = se.STX + len(msgpack_buffer).to_bytes(4, 'little') + msgpack_buffer
    return buffer + zlib.crc32(msgpack_buffer).to_bytes(4, 'little', signed=False)


@pytest.fixture(name="stream_extractor")
def fixture_stream_extractor():
    return se.MsgpackStreamExtractor()


def test_extract_one_segment_from_one_contiguous_block(stream_extractor):
    telegram = make_msgpack_telegram("This is some scan data.".encode())

    assert stream_extractor.extract_data_packages(telegram) == [telegram]


def test_extract_two_segments_from_one_contiguous_block(stream_extractor):
    telegram1 = make_msgpack_telegram("This is some scan data.".encode())
    telegram2 = make_msgpack_telegram("This is other scan data.".encode())

    assert stream_extractor.extract_data_packages(telegram1 + telegram2) == [telegram1, telegram2]


def test_extract_two_segments_with_nonsense_infix(stream_extractor):
    telegram1 = make_msgpack_telegram("This is some scan data.".encode())
    telegram2 = make_msgpack_telegram("This is other scan data.".encode())

    assert stream_extractor.extract_data_packages(
        telegram1 + "Nonsense".encode() + telegram2
        ) == [telegram1, telegram2]


def test_extract_segment_with_nonsense_prefix(stream_extractor):
    telegram = make_msgpack_telegram("This is some scan data.".encode())

    assert stream_extractor.extract_data_packages("Nonsense".encode() + telegram) == [telegram]


def test_extract_segment_with_nonsense_postfix(stream_extractor):
    telegram = make_msgpack_telegram("This is some scan data.".encode())

    assert stream_extractor.extract_data_packages(telegram + "Nonsense".encode()) == [telegram]


def test_extract_segment_from_three_byte_chunks(stream_extractor):
    telegram = make_msgpack_telegram("This is some scan data.".encode())

    chunk_size = 3
    chunks = [
        telegram[i*chunk_size:(i+1)*chunk_size] for i in range(math.ceil(len(telegram)/chunk_size))
        ]
    for chunk in chunks[:-1]:
        assert stream_extractor.extract_data_packages(chunk) == []

    assert stream_extractor.extract_data_packages(chunks[-1]) == [telegram]
