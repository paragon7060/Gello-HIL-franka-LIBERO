"""Bounded, read-only HTTP file for one HDF5 episode, without a full dataset download."""

from __future__ import annotations

import io
import re
from collections import OrderedDict


class RangeReader(io.RawIOBase):
    def __init__(self, url, *, session=None, byte_limit=80 * 1024**2, request_limit=128):
        import requests

        self.session = session or requests.Session()
        self.owns_session = session is None
        self.url, self.offset, self.size = url, 0, None
        self.byte_limit, self.request_limit = byte_limit, request_limit
        self.bytes_read, self.requests = 0, 0
        self.cache = OrderedDict()
        self.block_size = 256 * 1024
        self._fetch(0, self.block_size)

    def _fetch(self, start, length):
        if length < 1 or self.bytes_read + length > self.byte_limit:
            raise ValueError("Reference download byte budget exceeded")
        if self.requests >= self.request_limit:
            raise ValueError("Reference download request budget exceeded")
        end = start + length - 1
        self.requests += 1
        with self.session.get(
            self.url, headers={"Range": f"bytes={start}-{end}", "Accept-Encoding": "identity"},
            timeout=(15, 45), stream=True,
        ) as response:
            # A server ignoring Range must never download the entire multi-GB file.
            match = re.fullmatch(r"bytes (\d+)-(\d+)/(\d+)", response.headers.get("Content-Range", ""))
            if response.status_code != 206 or not match:
                raise ValueError("Source did not honor the bounded HTTP Range request")
            first, last, size = map(int, match.groups())
            if first != start or last != min(end, size - 1) or self.size not in (None, size):
                raise ValueError("Source returned an incorrect or changing HTTP range")
            self.size = size
            data = response.raw.read(length + 1)
            self.bytes_read += len(data)
            if len(data) != last - first + 1:
                raise ValueError("Incomplete HTTP range response")
        self.cache[(start, len(data))] = data
        while len(self.cache) > 24:
            self.cache.popitem(last=False)
        return data

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.offset

    def seek(self, offset, whence=0):
        position = offset if whence == 0 else self.offset + offset if whence == 1 else self.size + offset
        if whence not in (0, 1, 2) or position < 0:
            raise ValueError("Invalid reference file seek")
        self.offset = position
        return position

    def readinto(self, buffer):
        data = self.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)

    def read(self, length=-1):
        if length < 0:
            length = self.size - self.offset
        length = min(length, max(0, self.size - self.offset))
        if length == 0:
            return b""
        for (start, size), data in list(self.cache.items()):
            if start <= self.offset and self.offset + length <= start + size:
                result = data[self.offset - start:self.offset - start + length]
                break
        else:
            start = self.offset // self.block_size * self.block_size
            amount = min(self.size - start, max(self.block_size, self.offset - start + length))
            data = self._fetch(start, amount)
            result = data[self.offset - start:self.offset - start + length]
        self.offset += length
        return result

    def close(self):
        if self.owns_session:
            self.session.close()
        super().close()
