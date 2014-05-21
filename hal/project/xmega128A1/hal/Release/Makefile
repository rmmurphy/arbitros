################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../../../../source/xmega128A1/hal_adc.c \
../../../../source/xmega128A1/hal_clocks.c \
../../../../source/xmega128A1/hal_dma.c \
../../../../source/xmega128A1/hal_eeprom.c \
../../../../source/xmega128A1/hal_gpio.c \
../../../../source/xmega128A1/hal_pmic.c \
../../../../source/xmega128A1/hal_spi.c \
../../../../source/xmega128A1/hal_timer.c \
../../../../source/xmega128A1/hal_twi.c \
../../../../source/xmega128A1/hal_uart.c \
../../../../source/xmega128A1/hal_watchDog.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
hal_adc.o \
hal_clocks.o \
hal_dma.o \
hal_eeprom.o \
hal_gpio.o \
hal_pmic.o \
hal_spi.o \
hal_timer.o \
hal_twi.o \
hal_uart.o \
hal_watchDog.o


OBJS_AS_ARGS +=  \
hal_adc.o \
hal_clocks.o \
hal_dma.o \
hal_eeprom.o \
hal_gpio.o \
hal_pmic.o \
hal_spi.o \
hal_timer.o \
hal_twi.o \
hal_uart.o \
hal_watchDog.o


C_DEPS +=  \
hal_adc.d \
hal_clocks.d \
hal_dma.d \
hal_eeprom.d \
hal_gpio.d \
hal_pmic.d \
hal_spi.d \
hal_timer.d \
hal_twi.d \
hal_uart.d \
hal_watchDog.d


C_DEPS_AS_ARGS +=  \
hal_adc.d \
hal_clocks.d \
hal_dma.d \
hal_eeprom.d \
hal_gpio.d \
hal_pmic.d \
hal_spi.d \
hal_timer.d \
hal_twi.d \
hal_uart.d \
hal_watchDog.d


OUTPUT_FILE_PATH +=libhal.a

OUTPUT_FILE_PATH_AS_ARGS +=libhal.a

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler























./%.o: ../../../../source/xmega128A1/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../headers/xmega128A1" -I"../../../../../rtos/headers" -I"../../../../../utilities/headers"  -Os -fpack-struct -fshort-enums -Wno-unused-but-set-variable -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)


$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Archiver : GNU ar (AVR_8_bit_GNU_Toolchain_3.4.1_830) 2.22
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-ar.exe$(QUOTE) -r  -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS)
	@echo Finished building target: $@
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "libhal.elf" "libhal.a" "libhal.hex" "libhal.lss" "libhal.eep" "libhal.map" "libhal.srec"
	