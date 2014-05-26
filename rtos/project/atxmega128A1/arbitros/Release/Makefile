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
../../../../../hal/source/xmega128A1/hal_initThreadStack.c \
../../../../source/arb_console.c \
../../../../source/arb_device.c \
../../../../source/arb_error.c \
../../../../source/arb_idle.c \
../../../../source/arb_mailbox.c \
../../../../source/arb_main.c \
../../../../source/arb_memory.c \
../../../../source/arb_printf.c \
../../../../source/arb_scheduler.c \
../../../../source/arb_semaphore.c \
../../../../source/arb_sysTimer.c \
../../../../source/arb_thread.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
hal_initThreadStack.o \
arb_console.o \
arb_device.o \
arb_error.o \
arb_idle.o \
arb_mailbox.o \
arb_main.o \
arb_memory.o \
arb_printf.o \
arb_scheduler.o \
arb_semaphore.o \
arb_sysTimer.o \
arb_thread.o


OBJS_AS_ARGS +=  \
hal_initThreadStack.o \
arb_console.o \
arb_device.o \
arb_error.o \
arb_idle.o \
arb_mailbox.o \
arb_main.o \
arb_memory.o \
arb_printf.o \
arb_scheduler.o \
arb_semaphore.o \
arb_sysTimer.o \
arb_thread.o


C_DEPS +=  \
hal_initThreadStack.d \
arb_console.d \
arb_device.d \
arb_error.d \
arb_idle.d \
arb_mailbox.d \
arb_main.d \
arb_memory.d \
arb_printf.d \
arb_scheduler.d \
arb_semaphore.d \
arb_sysTimer.d \
arb_thread.d


C_DEPS_AS_ARGS +=  \
hal_initThreadStack.d \
arb_console.d \
arb_device.d \
arb_error.d \
arb_idle.d \
arb_mailbox.d \
arb_main.d \
arb_memory.d \
arb_printf.d \
arb_scheduler.d \
arb_semaphore.d \
arb_sysTimer.d \
arb_thread.d


OUTPUT_FILE_PATH +=libarbitros.a

OUTPUT_FILE_PATH_AS_ARGS +=libarbitros.a

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler



























./%.o: ../../../../../hal/source/xmega128A1/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../headers" -I"../../../../../utilities/headers" -I"../../../../../drivers/headers" -I"../../../../../hal/headers/xmega128A1"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: ../../../../source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../headers" -I"../../../../../utilities/headers" -I"../../../../../drivers/headers" -I"../../../../../hal/headers/xmega128A1"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
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
	rm -rf "libarbitros.elf" "libarbitros.a" "libarbitros.hex" "libarbitros.lss" "libarbitros.eep" "libarbitros.map" "libarbitros.srec"
	