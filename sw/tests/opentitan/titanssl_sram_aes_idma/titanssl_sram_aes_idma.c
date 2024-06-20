// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"

#include "sw/device/lib/arch/device.h"
#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/base/bitfield.h"
#include "sw/device/lib/base/memory.h"
#include "sw/device/silicon_creator/rom/uart.h"
#include "sw/device/silicon_creator/rom/string_lib.h"

#include "aes_regs.h"  // Generated.


#define TARGET_SYNTHESIS


#define IDMA_BASE 		 0xfef00000
#define TCDM_BASE      0xfff01000
#define L2_BASE        0x1C001000
#define L3_BASE        0x80000000

#define IDMA_SRC_ADDR_OFFSET         0x0d8
#define IDMA_DST_ADDR_OFFSET         0x0d0
#define IDMA_LENGTH_OFFSET           0x0e0
#define IDMA_NEXT_ID_OFFSET          0x044
#define IDMA_REPS_2                  0x0f8
#define IDMA_CONF                    0x000

typedef struct
{
    uint8_t *data;
    size_t  n;
} titanssl_buffer_t;


static titanssl_buffer_t buffer_plain_idma;
static titanssl_buffer_t buffer_cipher_idma;
static titanssl_buffer_t buffer_plain;
static titanssl_buffer_t buffer_cipher;
static titanssl_buffer_t buffer_key;
static titanssl_buffer_t buffer_iv;


/* ============================================================================
 * Benchmark setup
 * ========================================================================= */

#define TITANSSL_CFG_DEBUG   0
#define TITANSSL_CFG_MEM_L3  1
#define TITANSSL_CFG_MEM_L1  0
#ifndef TITANSSL_CFG_PAYLOAD
  #define TITANSSL_CFG_PAYLOAD 4096
#endif

/* ============================================================================
 * Benchmark automatic configuration
 * ========================================================================= */

#if TITANSSL_CFG_MEM_L3
#define TITANSSL_ADDR_PLAIN_IDMA  0xfff00000
#define TITANSSL_ADDR_CIPHER_IDMA 0xfff04000
#define TITANSSL_ADDR_PLAIN  0x80720000
#define TITANSSL_ADDR_CIPHER 0x80740000
#define TITANSSL_ADDR_KEY    0xe0006000
#define TITANSSL_ADDR_IV     0xe0006100
#elif TITANSSL_CFG_MEM_L1
#define TITANSSL_ADDR_PLAIN  0xe0002000
#define TITANSSL_ADDR_CIPHER 0xe0004000
#define TITANSSL_ADDR_KEY    0xe0006000
#define TITANSSL_ADDR_IV     0xe0006100
#else
#error "Wrong benchmark memory configuration"
#endif

#define TITANSSL_SIZE_PLAIN  ((TITANSSL_CFG_PAYLOAD+0xF) & ~0xF)
#define TITANSSL_SIZE_CIPHER TITANSSL_SIZE_PLAIN
#define TITANSSL_SIZE_KEY    32
#define TITANSSL_SIZE_IV     16

/* ============================================================================
 * Benchmark implementation
 * ========================================================================= */

int next_id, new_next_id;

void wait_for_idma_eot(int next_id){
    volatile uint32_t *ptr;
    ptr = (uint32_t *) IDMA_BASE + IDMA_NEXT_ID_OFFSET ;
    while(*ptr!=next_id)  //IDMA_WAIT
      asm volatile("nop");  //IDMA_WAIT
}

int issue_idma_transaction(uint32_t src_addr, uint32_t dst_addr, uint32_t num_bytes){
    volatile uint32_t * ptr32;
    uint8_t buff = 0;
    ptr32 = (int *) (IDMA_BASE + IDMA_SRC_ADDR_OFFSET);
    *ptr32 = src_addr;
    ptr32 = (int *) (IDMA_BASE + IDMA_DST_ADDR_OFFSET);
    *ptr32 = dst_addr;
    ptr32 = (int *) (IDMA_BASE + IDMA_LENGTH_OFFSET);
    *ptr32 = num_bytes;
    ptr32 = (int *) (IDMA_BASE + IDMA_CONF);
    *ptr32 = 0x3<<10;
    ptr32 = (int *) (IDMA_BASE + IDMA_REPS_2);
    *ptr32 = 0x00000001;
    ptr32 = (int *) (IDMA_BASE + IDMA_NEXT_ID_OFFSET);
    buff = *ptr32;
    return *ptr32; // IDMA
}
void initialize_edn()
{
    uint32_t *p;
    mmio_region_t aes; //CONF

    p = (uint32_t*)0xc1160024;
    *p = 0x00909099;
    p = (uint32_t*)0xc1160020;
    *p = 0x00000006;
    p = (uint32_t*)0xc1150014;
    *p = 0x00000666;
    p = (uint32_t*)0xc1170014;
    *p = 0x00009966;
    aes = mmio_region_from_addr(TOP_EARLGREY_AES_BASE_ADDR); //CONF
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
}

void initialize_memory()
{

#if TITANSSL_CFG_MEM_L3
    buffer_plain_idma.data = (uint8_t*)TITANSSL_ADDR_PLAIN_IDMA;
    buffer_plain_idma.n = TITANSSL_SIZE_PLAIN;

    buffer_cipher_idma.data = (uint8_t*)TITANSSL_ADDR_CIPHER_IDMA;
    buffer_cipher_idma.n = TITANSSL_SIZE_PLAIN;
#endif
    buffer_plain.data = (uint8_t*)TITANSSL_ADDR_PLAIN;
    buffer_plain.n = TITANSSL_SIZE_PLAIN;
    //for (size_t i=0; i<TITANSSL_SIZE_PLAIN; i++) buffer_plain.data[i] = 0x0;

    buffer_cipher.data = (uint8_t*)TITANSSL_ADDR_CIPHER;
    buffer_cipher.n = TITANSSL_SIZE_CIPHER;
    //for (size_t i=0; i<TITANSSL_SIZE_CIPHER; i++) buffer_cipher.data[i] = 0x0;

    buffer_key.data = (uint8_t*)TITANSSL_ADDR_KEY;
    buffer_key.n = TITANSSL_SIZE_KEY;
    for (size_t i=0; i<TITANSSL_SIZE_KEY; i++) buffer_key.data[i] = 0x0;

    buffer_iv.data = (uint8_t*)TITANSSL_ADDR_IV;
    buffer_iv.n = TITANSSL_SIZE_IV;
    for (size_t i=0; i<TITANSSL_SIZE_IV; i++) buffer_iv.data[i] = 0x0;
}

void titanssl_benchmark_aes(
        titanssl_buffer_t *const plain_idma,
        titanssl_buffer_t *const cipher_idma,
        titanssl_buffer_t *const plain,
        titanssl_buffer_t *const cipher,
        titanssl_buffer_t *const key,
        titanssl_buffer_t *const iv)
{ //CONF
  mmio_region_t aes; //CONF
    uint32_t reg; //CONF
    uint8_t *dp_src; //CONF
    uint8_t *dp_dst; //CONF
    // Get the AES IP base address
    aes = mmio_region_from_addr(TOP_EARLGREY_AES_BASE_ADDR); //CONF

#if TITANSSL_CFG_MEM_L3
    uint32_t src_addr_int = (uint32_t)(uintptr_t)plain->data;//CONF
    uint32_t dst_addr_int = (uint32_t)(uintptr_t)plain_idma->data;//CONF
    next_id = issue_idma_transaction(src_addr_int, dst_addr_int, plain->n);//CONF
#endif

    // Reset the IP
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
    reg = bitfield_bit32_write(0, AES_CTRL_SHADOWED_MANUAL_OPERATION_BIT, true); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF
    reg = bitfield_bit32_write(0, AES_TRIGGER_KEY_IV_DATA_IN_CLEAR_BIT, true); //CONF
    reg = bitfield_bit32_write(reg, AES_TRIGGER_DATA_OUT_CLEAR_BIT, true); //CONF
    mmio_region_write32(aes, AES_TRIGGER_REG_OFFSET, reg); //CONF
    while (!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
    reg = bitfield_field32_write(0, AES_CTRL_SHADOWED_OPERATION_FIELD, AES_CTRL_SHADOWED_OPERATION_MASK); //CONF
    reg = bitfield_field32_write(reg, AES_CTRL_SHADOWED_MODE_FIELD, AES_CTRL_SHADOWED_MODE_VALUE_AES_NONE); //CONF
    reg = bitfield_field32_write(reg, AES_CTRL_SHADOWED_KEY_LEN_FIELD, AES_CTRL_SHADOWED_KEY_LEN_MASK); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF

    // Initialize AES IP configurations
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
    reg = bitfield_field32_write(0, AES_CTRL_SHADOWED_OPERATION_FIELD, AES_CTRL_SHADOWED_OPERATION_VALUE_AES_ENC); //CONF
    reg = bitfield_field32_write(reg, AES_CTRL_SHADOWED_MODE_FIELD, AES_CTRL_SHADOWED_MODE_VALUE_AES_CBC); //CONF
    reg = bitfield_field32_write(reg, AES_CTRL_SHADOWED_KEY_LEN_FIELD, AES_CTRL_SHADOWED_KEY_LEN_VALUE_AES_256); //CONF
    reg = bitfield_field32_write(reg, AES_CTRL_SHADOWED_PRNG_RESEED_RATE_FIELD, AES_CTRL_SHADOWED_PRNG_RESEED_RATE_VALUE_PER_64); //CONF
    reg = bitfield_bit32_write(reg, AES_CTRL_SHADOWED_MANUAL_OPERATION_BIT, false); //CONF
    reg = bitfield_bit32_write(reg, AES_CTRL_SHADOWED_SIDELOAD_BIT, false); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg); //CONF

    // Initialize AES IP auxiliary configurations
    reg = bitfield_bit32_write(0, AES_CTRL_AUX_SHADOWED_KEY_TOUCH_FORCES_RESEED_BIT, false); //CONF
    reg = bitfield_bit32_write(reg, AES_CTRL_AUX_SHADOWED_FORCE_MASKS_BIT, false); //CONF
    mmio_region_write32(aes, AES_CTRL_AUX_SHADOWED_REG_OFFSET, reg); //CONF
    mmio_region_write32(aes, AES_CTRL_AUX_SHADOWED_REG_OFFSET, reg); //CONF
    mmio_region_write32(aes, AES_CTRL_AUX_REGWEN_REG_OFFSET, true); //CONF

    // Initialize key shares
    mmio_region_write32(aes, AES_KEY_SHARE0_0_REG_OFFSET, ((uint32_t*)(key->data))[0]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_1_REG_OFFSET, ((uint32_t*)(key->data))[1]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_2_REG_OFFSET, ((uint32_t*)(key->data))[2]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_3_REG_OFFSET, ((uint32_t*)(key->data))[3]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_4_REG_OFFSET, ((uint32_t*)(key->data))[4]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_5_REG_OFFSET, ((uint32_t*)(key->data))[5]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_6_REG_OFFSET, ((uint32_t*)(key->data))[6]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE0_7_REG_OFFSET, ((uint32_t*)(key->data))[7]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_0_REG_OFFSET, ((uint32_t*)(key->data))[0]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_1_REG_OFFSET, ((uint32_t*)(key->data))[1]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_2_REG_OFFSET, ((uint32_t*)(key->data))[2]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_3_REG_OFFSET, ((uint32_t*)(key->data))[3]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_4_REG_OFFSET, ((uint32_t*)(key->data))[4]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_5_REG_OFFSET, ((uint32_t*)(key->data))[5]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_6_REG_OFFSET, ((uint32_t*)(key->data))[6]); //CONF
    mmio_region_write32(aes, AES_KEY_SHARE1_7_REG_OFFSET, ((uint32_t*)(key->data))[7]); //CONF

    // Initialize IV
    reg = mmio_region_read32(aes, AES_CTRL_SHADOWED_REG_OFFSET); //CONF
    reg = bitfield_field32_read(reg, AES_CTRL_SHADOWED_MODE_FIELD); //CONF
    if (reg != AES_CTRL_SHADOWED_MODE_VALUE_AES_ECB) //CONF
    {
        while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
        mmio_region_write32(aes, AES_IV_0_REG_OFFSET, ((uint32_t*)(iv->data))[0]); //CONF
        mmio_region_write32(aes, AES_IV_1_REG_OFFSET, ((uint32_t*)(iv->data))[1]); //CONF
        mmio_region_write32(aes, AES_IV_2_REG_OFFSET, ((uint32_t*)(iv->data))[2]); //CONF
        mmio_region_write32(aes, AES_IV_3_REG_OFFSET, ((uint32_t*)(iv->data))[3]); //CONF
    } //CONF
#if TITANSSL_CFG_MEM_L3
    wait_for_idma_eot(next_id); //CONF
    // Compute AES
    dp_src = plain_idma->data; //CIPHER
    dp_dst = cipher_idma->data; //CIPHER
#elif TITANSSL_CFG_MEM_L1
    // Compute AES
    dp_src = plain->data; //CIPHER
    dp_dst = cipher->data; //CIPHER
#endif
    mmio_region_write32(aes, AES_DATA_IN_0_REG_OFFSET, ((uint32_t*)dp_src)[0]); //CIPHER
    mmio_region_write32(aes, AES_DATA_IN_1_REG_OFFSET, ((uint32_t*)dp_src)[1]); //CIPHER
    mmio_region_write32(aes, AES_DATA_IN_2_REG_OFFSET, ((uint32_t*)dp_src)[2]); //CIPHER
    mmio_region_write32(aes, AES_DATA_IN_3_REG_OFFSET, ((uint32_t*)dp_src)[3]); //CIPHER
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_INPUT_READY_BIT)); //CIPHER
    dp_src += 16; //CIPHER
    #if TITANSSL_CFG_MEM_L3
    while (dp_src - plain_idma->data < plain->n) { //CIPHER
    #elif TITANSSL_CFG_MEM_L1
    while (dp_src - plain->data < plain->n) { //CIPHER
    #endif
        mmio_region_write32(aes, AES_DATA_IN_0_REG_OFFSET, ((uint32_t*)dp_src)[0]); //CIPHER
        mmio_region_write32(aes, AES_DATA_IN_1_REG_OFFSET, ((uint32_t*)dp_src)[1]); //CIPHER
        mmio_region_write32(aes, AES_DATA_IN_2_REG_OFFSET, ((uint32_t*)dp_src)[2]); //CIPHER
        mmio_region_write32(aes, AES_DATA_IN_3_REG_OFFSET, ((uint32_t*)dp_src)[3]); //CIPHER

        while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_OUTPUT_VALID_BIT)); //CIPHER
        ((uint32_t*)(dp_dst))[0] = mmio_region_read32(aes, AES_DATA_OUT_0_REG_OFFSET); //CIPHER
        ((uint32_t*)(dp_dst))[1] = mmio_region_read32(aes, AES_DATA_OUT_1_REG_OFFSET); //CIPHER
        ((uint32_t*)(dp_dst))[2] = mmio_region_read32(aes, AES_DATA_OUT_2_REG_OFFSET); //CIPHER
        ((uint32_t*)(dp_dst))[3] = mmio_region_read32(aes, AES_DATA_OUT_3_REG_OFFSET); //CIPHER
        dp_dst += 16; //CIPHER
        dp_src += 16; //CIPHER
    }
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_OUTPUT_VALID_BIT)); //CIPHER
    ((uint32_t*)(dp_dst))[0] = mmio_region_read32(aes, AES_DATA_OUT_0_REG_OFFSET); //CIPHER
    ((uint32_t*)(dp_dst))[1] = mmio_region_read32(aes, AES_DATA_OUT_1_REG_OFFSET); //CIPHER
    ((uint32_t*)(dp_dst))[2] = mmio_region_read32(aes, AES_DATA_OUT_2_REG_OFFSET); //CIPHER
    ((uint32_t*)(dp_dst))[3] = mmio_region_read32(aes, AES_DATA_OUT_3_REG_OFFSET); //CIPHER
#if TITANSSL_CFG_MEM_L3
    uint32_t src_addr_int_cipher = (uint32_t)(uintptr_t)cipher_idma->data; //CIPHER
    uint32_t dst_addr_int_cipher = (uint32_t)(uintptr_t)cipher->data; //CIPHER
    new_next_id = issue_idma_transaction(src_addr_int_cipher, dst_addr_int_cipher, plain->n);
#endif
    // Reset operation mode, key, iv, and data registers
    while(!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT)); //CONF
    reg = bitfield_bit32_write(0, AES_CTRL_SHADOWED_MANUAL_OPERATION_BIT, true);//CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg);//CONF
    mmio_region_write32(aes, AES_CTRL_SHADOWED_REG_OFFSET, reg);//CONF
    reg = bitfield_bit32_write(0, AES_TRIGGER_KEY_IV_DATA_IN_CLEAR_BIT, true);//CONF
    reg = bitfield_bit32_write(reg, AES_TRIGGER_DATA_OUT_CLEAR_BIT, true);//CONF
    mmio_region_write32(aes, AES_TRIGGER_REG_OFFSET, reg);//CONF
    while (!mmio_region_get_bit32(aes, AES_STATUS_REG_OFFSET, AES_STATUS_IDLE_BIT));//CONF
#if TITANSSL_CFG_MEM_L3
    wait_for_idma_eot(new_next_id); //IDMA_WAIT
#endif
}//CONF

int main(
        int argc, 
        char **argv)
{
#ifdef TARGET_SYNTHESIS
#define baud_rate 115200
#define test_freq 50000000
#else
#define baud_rate 115200
#define test_freq 100000000
#endif
	uart_set_cfg(
        0,
        (test_freq/baud_rate)>>4
    );

    initialize_edn();
    initialize_memory();
    titanssl_benchmark_aes(
        &buffer_plain_idma,
        &buffer_cipher_idma,
        &buffer_plain,
        &buffer_cipher,
        &buffer_key,
        &buffer_iv
    );

#if TITANSSL_CFG_DEBUG

  printf("payload size: %d Bytes\r\n",buffer_plain.n);
  uart_wait_tx_done();
  printf("input: 0x");
  uart_wait_tx_done();
	for (int i=0; i<buffer_plain.n; i++)
	{
		printf("%02x", buffer_plain.data[i]);
        uart_wait_tx_done();
	}
  printf("\r\n");
  uart_wait_tx_done();

  printf("Output: 0x");
	for (int i=0; i<buffer_cipher.n; i++)
	{
		printf("%02x", buffer_cipher.data[i]);
        uart_wait_tx_done();
	}
  printf("\r\n");
  uart_wait_tx_done();

  printf("Key: 0x");
	for (int i=0; i<buffer_key.n; i++)
	{
		printf("%02x", buffer_key.data[i]);
        uart_wait_tx_done();
	}
  printf("\r\n");
  uart_wait_tx_done();
#endif

	return 0;
}

      /* ptr2 = (uint32_t *) 0xfef00024;
        if(*ptr2==next_id){
          uint32_t src_addr_int_cipher = (uint32_t)(uintptr_t)cipher_idma->data;
          uint32_t dst_addr_int_cipher = (uint32_t)(uintptr_t)cipher->data;
          new_next_id = issue_idma_transaction(src_addr_int_cipher, dst_addr_int_cipher, plain->n);
          }*/
