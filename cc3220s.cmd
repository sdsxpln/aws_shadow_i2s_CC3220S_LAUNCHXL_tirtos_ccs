//*****************************************************************************
// cc3220s.cmd
//
// CCS linker configuration file for CC3220S production device, where 
// it has 256kB of RAM (where bootloader uses 16kB of RAM)
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


--retain=g_pfnVectors

//*****************************************************************************
// The following command line options are set as part of the CCS project.  
// If you are building using the command line, or for some reason want to    
// define them here, you can uncomment and modify these lines as needed.     
// If you are using CCS for building, it is probably better to make any such 
// modifications in your CCS project and leave this file alone.              
//*****************************************************************************


//*****************************************************************************
// The starting address of the application.  Normally the interrupt vectors  
// must be located at the beginning of the application.                      
//*****************************************************************************
#define SRAM_BASE 0x20004000

//*****************************************************************************
// NOTE:
//    While loading the application the intials 16KB (SRAM_DATA_2) of SRAM is 
//    used by the bootloader. This section of the memory is available to the 
//    application once it is launched by the bootloader and this linker script 
//    uses the 16KB block for placing the '.stack' section.
// 
//    The rest 240KB starting from 0x20004000 can be divided between code 
//    section(SRAM_CODE) and data section (SRAM_DATA)as per the 
//    application requirement.
//
//
//           _____________________________ _0x20000000
//          |      ROM Bootloader         |
//          |       SRAM_DATA_2           |
//          |_____________________________|_0x20004000
//          |                             |
//          |                             |
//          |                             |
//          |                             |
//          |        SRAM_CODE            |
//          |                             |
//          |                             |
//          |                             |
//          |                             |
//          |                             |
//          |-----------------------------|-0x2000XXXX
//          |                             |
//          |                             |
//          |                             |
//          |         SRAM_DATA           |
//          |                             |
//          |                             |
//          |_____________________________|_0x2003FFFF
//
//
//*****************************************************************************

/* System memory map */

MEMORY
{
    /* Application uses internal RAM for program and data */
    SRAM_CODE   (RWX) : origin = 0x20004000, length = 0x19000 /* 100KB */
    SRAM_DATA   (RWX) : origin = 0x2001D000, length = 0x23000 /* 140KB */
    SRAM_DATA_2 (RWX) : origin = 0x20000000, length = 0x4000  /* 16KB */
}

/* Section allocation in memory */

SECTIONS
{
    .resetVecs            : > SRAM_BASE
    .init_array           : > SRAM_CODE
    .text                 : > SRAM_CODE
    .const                : > SRAM_CODE
    .cinit                : > SRAM_CODE
    .pinit                : > SRAM_CODE
    .data                 : > SRAM_DATA
    .bss                  : > SRAM_DATA
    .sysmem               : > SRAM_DATA
    .ramvecs              : > SRAM_DATA_2 ALIGN(1024)
    .stack                : > SRAM_DATA_2(HIGH)
}

