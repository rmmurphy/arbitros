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
../../../../SdFat/arduino/new.cpp \
../../../../SdFat/arduino/Print.cpp \
../../../../SdFat/arduino/WString.cpp \
../../../../SdFat/MinimumSerial.cpp \
../../../../SdFat/Sd2Card.cpp \
../../../../SdFat/SdBaseFile.cpp \
../../../../SdFat/SdFat.cpp \
../../../../SdFat/SdFile.cpp \
../../../../SdFat/SdVolume.cpp \
../../../../source/sdInterface.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
new.o \
Print.o \
WString.o \
MinimumSerial.o \
Sd2Card.o \
SdBaseFile.o \
SdFat.o \
SdFile.o \
SdVolume.o \
sdInterface.o


OBJS_AS_ARGS +=  \
new.o \
Print.o \
WString.o \
MinimumSerial.o \
Sd2Card.o \
SdBaseFile.o \
SdFat.o \
SdFile.o \
SdVolume.o \
sdInterface.o


C_DEPS +=  \
new.d \
Print.d \
WString.d \
MinimumSerial.d \
Sd2Card.d \
SdBaseFile.d \
SdFat.d \
SdFile.d \
SdVolume.d \
sdInterface.d


C_DEPS_AS_ARGS +=  \
new.d \
Print.d \
WString.d \
MinimumSerial.d \
Sd2Card.d \
SdBaseFile.d \
SdFat.d \
SdFile.d \
SdVolume.d \
sdInterface.d


OUTPUT_FILE_PATH +=libsd.a

OUTPUT_FILE_PATH_AS_ARGS +=libsd.a

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler





















./%.o: ../../../../SdFat/arduino/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C++ Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../headers" -I"../../../../SdFat" -I"../../../../SdFat/arduino" -I"../../../../SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: ../../../../SdFat/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C++ Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../headers" -I"../../../../SdFat" -I"../../../../SdFat/arduino" -I"../../../../SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: ../../../../source/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C++ Compiler : (AVR_8_bit_GNU_Toolchain_3.4.1_830) 4.6.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"../../../../../../hal/headers/xmega128A1" -I"../../../../../../drivers/headers" -I"../../../../../../rtos/headers" -I"../../../../../../utilities/headers" -I"../../../../headers" -I"../../../../SdFat" -I"../../../../SdFat/arduino" -I"../../../../SdFat/utility"  -Os -ffunction-sections -fpack-struct -fshort-enums -Wall -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atxmega128a1   -o"$@" "$<" 
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
	@echo Invoking: AVR8/GNU Archiver : GNU ar (AVR_8_bit_GNU_Toolchain_3.4.1_830) 2.22
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-ar.exe$(QUOTE) -r  -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS)
	@echo Finished building target: $@
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "libsd.elf" "libsd.a" "libsd.hex" "libsd.lss" "libsd.eep" "libsd.map" "libsd.srec"
	