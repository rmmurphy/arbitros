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
../../../../../../drivers/source/drv_console.c \
../../../../../../drivers/source/drv_ins.c \
../../../../../../drivers/source/drv_signal.c \
../../source/usr_appInit.c \
../../source/usr_console.c \
../../source/usr_navigation.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
drv_console.o \
drv_ins.o \
drv_signal.o \
usr_appInit.o \
usr_console.o \
usr_navigation.o


OBJS_AS_ARGS +=  \
drv_console.o \
drv_ins.o \
drv_signal.o \
usr_appInit.o \
usr_console.o \
usr_navigation.o


C_DEPS +=  \
drv_console.d \
drv_ins.d \
drv_signal.d \
usr_appInit.d \
usr_console.d \
usr_navigation.d


C_DEPS_AS_ARGS +=  \
drv_console.d \
drv_ins.d \
drv_signal.d \
usr_appInit.d \
usr_console.d \
usr_navigation.d


OUTPUT_FILE_PATH +=ins.elf

OUTPUT_FILE_PATH_AS_ARGS +=ins.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler


./drv_ins.o: ../../../../../../drivers/source/drv_ins.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DF_CPU=32000000UL  -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -g2 -Wall -c -gdwarf-2 -std=gnu99 -fomit-frame-pointer -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1  -O0 -o"$@" "$<" 
	@echo Finished building: $<










./%.o: ../../../../../../drivers/source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DF_CPU=32000000UL  -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -g2 -Wall -c -gdwarf-2 -std=gnu99 -fomit-frame-pointer -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: ../../source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DF_CPU=32000000UL  -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -g2 -Wall -c -gdwarf-2 -std=gnu99 -fomit-frame-pointer -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
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
	@echo Invoking: AVR/GNU Linker : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="ins.map" -Wl,--start-group -Wl,-lhal -Wl,-larbitros -Wl,-lutilities -Wl,-lsd -Wl,-lprintf_flt -Wl,-lm -Wl,-lc  -Wl,--end-group -Wl,-L"../../../../../../hal/project/xmega128A1/hal/Release" -Wl,-L"../../../../../../rtos/project/atxmega128A1/arbitros/Release" -Wl,-L"../../../../../../utilities/project/atxmega128A1/utilities/Release" -Wl,-L"../../../../../../rtos/packages/sd/project/atxmega128A1/sd/Release" -Wl,-L"../../../../../../rtos/hal/project/xmega128A1/hal/Release" -Wl,-L"../../../../../../packages/sd/project/atxmega128A1/sd/Release"  -Wl,--gc-sections -Wl,-u,vfprintf -lprintf_flt -lm  -mmcu=atxmega128a1  
	@echo Finished building target: $@
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "ins.elf" "ins.hex"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "ins.elf" "ins.eep" || exit 0
	"C:\WinAVR-20100110\bin\avr-objdump.exe" -h -S "ins.elf" > "ins.lss"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "ins.elf" "ins.srec"
	"C:\WinAVR-20100110\bin\avr-size.exe" "ins.elf"
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "ins.elf" "ins.a" "ins.hex" "ins.lss" "ins.eep" "ins.map" "ins.srec"
	