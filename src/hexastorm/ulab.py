def packbits(bitlst, bitorder="big"):
    """Packs the elements of a binary-valued array
    into bits in a uint8 array."""

    tmp_str = ""
    byte_lst = []

    for idx, bit in enumerate(bitlst):
        tmp_str += str(bit)
        if len(tmp_str) == 8:
            if bitorder == "little":
                tmp_str = tmp_str[::-1]
            byte_lst.append(int(tmp_str, 2))
            tmp_str = ""

    if len(tmp_str) > 0:
        tmp_str += "0" * (8 - len(tmp_str))
        if bitorder == "little":
            tmp_str = tmp_str[::-1]
        byte_lst.append(int(tmp_str, 2))
    return byte_lst
