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
../../../../../../drivers/source/drv_sd.c \
../../../../../../drivers/source/drv_signal.c \
../../source/usr_appInit.c \
../../source/usr_console.c \
../../source/usr_mathTest.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
drv_console.o \
drv_sd.o \
drv_signal.o \
usr_appInit.o \
usr_console.o \
usr_mathTest.o


OBJS_AS_ARGS +=  \
drv_console.o \
drv_sd.o \
drv_signal.o \
usr_appInit.o \
usr_console.o \
usr_mathTest.o


C_DEPS +=  \
drv_console.d \
drv_sd.d \
drv_signal.d \
usr_appInit.d \
usr_console.d \
usr_mathTest.d


C_DEPS_AS_ARGS +=  \
drv_console.d \
drv_sd.d \
drv_signal.d \
usr_appInit.d \
usr_console.d \
usr_mathTest.d


OUTPUT_FILE_PATH +=mathEval.elf

OUTPUT_FILE_PATH_AS_ARGS +=mathEval.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler










./usr_mathTest.o: ../../source/usr_mathTest.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -O3 -fpack-struct -fshort-enums -g2 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1  -O0 -o"$@" "$<" 
	@echo Finished building: $<


./%.o: ../../../../../../drivers/source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -O3 -fpack-struct -fshort-enums -g2 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: ../../source/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../headers" -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../../../packages/sd/headers" -I"../../../../../../packages/sd/SdFat" -I"../../../../../../packages/sd/SdFat/arduino" -I"../../../../../../packages/sd/SdFat/utility"  -O3 -fpack-struct -fshort-enums -g2 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
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
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="mathEval.map" -Wl,--start-group -Wl,-lhal -Wl,-larbitros -Wl,-lutilities -Wl,-lsd -Wl,-lprintf_flt -Wl,-lm -Wl,-lc  -Wl,--end-group -Wl,-L"../../../../../../rtos/project/atxmega128A1/arbitros/Release" -Wl,-L"../../../../../../utilities/project/atxmega128A1/utilities/Release" -Wl,-L"../../../../../../packages/sd/project/atxmega128A1/sd/Release" -Wl,-L"../../../../../../hal/project/xmega128A1/hal/Release"  -Wl,--gc-sections -Wl,-u,vfprintf -lprintf_flt -lm  -mmcu=atxmega128a1  
	@echo Finished building target: $@
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "mathEval.elf" "mathEval.hex"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "mathEval.elf" "mathEval.eep" || exit 0
	"C:\WinAVR-20100110\bin\avr-objdump.exe" -h -S "mathEval.elf" > "mathEval.lss"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "mathEval.elf" "mathEval.srec"
	"C:\WinAVR-20100110\bin\avr-size.exe" "mathEval.elf"
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "mathEval.elf" "mathEval.a" "mathEval.hex" "mathEval.lss" "mathEval.eep" "mathEval.map" "mathEval.srec"
	