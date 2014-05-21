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
../../../../source/utl_buffer.c \
../../../../source/utl_linkedlist.c \
../../../../source/utl_math.c \
../../../../source/utl_stateMachine.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
utl_buffer.o \
utl_linkedlist.o \
utl_math.o \
utl_stateMachine.o


OBJS_AS_ARGS +=  \
utl_buffer.o \
utl_linkedlist.o \
utl_math.o \
utl_stateMachine.o


C_DEPS +=  \
utl_buffer.d \
utl_linkedlist.d \
utl_math.d \
utl_stateMachine.d


C_DEPS_AS_ARGS +=  \
utl_buffer.d \
utl_linkedlist.d \
utl_math.d \
utl_stateMachine.d


OUTPUT_FILE_PATH +=libutilities.a

OUTPUT_FILE_PATH_AS_ARGS +=libutilities.a

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler









./%.o: ../../../../source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../../hal/headers/xmega128A1" -I"../../../../../rtos/headers" -I"../../../../headers"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
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
	@echo Invoking: AVR/GNU Archiver : GNU ar (WinAVR 20100110) 2.19
	$(QUOTE)C:\WinAVR-20100110\bin\avr-ar.exe$(QUOTE) -r  -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS)
	@echo Finished building target: $@
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "libutilities.elf" "libutilities.a" "libutilities.hex" "libutilities.lss" "libutilities.eep" "libutilities.map" "libutilities.srec"
	