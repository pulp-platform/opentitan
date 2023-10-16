# Copyright 2022 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#

import random
import os

def generate_vmem_file(filename, num_words, word_length):
    with open(filename, 'w') as vmem_file:
        word_value = 0
        for i in range(num_words):
            word_hex = ''.join(random.choice('0123456789ABCDEF') for _ in range(word_length))
            vmem_file.write(f'@{word_value:08X} {word_hex}\n')
            word_value += 1

if __name__ == "__main__":
    # Get the current working directory
    current_directory = os.getcwd()

    filename_2048 = os.path.join(current_directory, "rand_preload_2048.vmem")
    num_words_2048 = 2048
    word_length_2048 = 19  # 76 bits (19 hex digits)
    generate_vmem_file(filename_2048, num_words_2048, word_length_2048)

    filename_512 = os.path.join(current_directory, "rand_preload_512.vmem")
    num_words_512 = 512
    word_length_512 = 19  # 76 bits (19 hex digits)
    generate_vmem_file(filename_512, num_words_512, word_length_512)

    filename_4096 = os.path.join(current_directory, "rand_preload_4096.vmem")
    num_words_4096 = 4096
    word_length_4096 = 19  # 76 bits (19 hex digits)
    generate_vmem_file(filename_4096, num_words_4096, word_length_4096)
