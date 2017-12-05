import base58
import binascii

__author__ = 'Xomak'


class MultihashConverter:

    CONSTANT_HEX = "1220"

    @staticmethod
    def to_bytes(hash_string):
        bytes_array = base58.b58decode(hash_string)
        return bytes_array[2:]

    @staticmethod
    def to_hash(bytes_string):
        bytes_string = bytes_string.encode('latin-1')
        return base58.b58encode(binascii.unhexlify(MultihashConverter.CONSTANT_HEX) + bytes(bytes_string))


