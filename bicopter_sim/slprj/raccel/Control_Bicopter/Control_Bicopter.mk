# Copyright 1994-2019 The MathWorks, Inc.
#
# File    : raccel_vcx64.tmf   
#
# Abstract:
#       Template makefile for building a PC-based "rapid acceleration" 
#       executable from the generated C code for a Simulink model using
#       generated C code and the
#               Microsoft Visual C/C++ compiler version 8.0 for x64
#
#       Note that this template is automatically customized by the build 
#       procedure to create "<model>.mk"
#
#       The following defines can be used to modify the behavior of the
#       build:
#
#         OPT_OPTS       - Optimization option. See DEFAULT_OPT_OPTS in
#                          vctools.mak for default.
#         OPTS           - User specific options.
#         CPP_OPTS       - C++ compiler options.
#         USER_SRCS      - Additional user sources, such as files needed by
#                          S-functions.
#         USER_INCLUDES  - Additional include paths
#                          (i.e. USER_INCLUDES="-Iwhere-ever -Iwhere-ever2")
#
#       To enable debugging:
#         set DEBUG_BUILD = 1, which will trigger OPTS=-Zi (may vary with
#                               compiler version, see compiler doc) 
#
#       This template makefile is designed to be used with a system target
#       file that contains 'rtwgensettings.BuildDirSuffix' see raccel.tlc

#------------------------ Macros read by make_rtw -----------------------------
#
# The following macros are read by the build procedure:
#
#  MAKECMD         - This is the command used to invoke the make utility
#  HOST            - What platform this template makefile is targeted for
#                    (i.e. PC or UNIX)
#  BUILD           - Invoke make from the build procedure (yes/no)?
#  SYS_TARGET_FILE - Name of system target file.

MAKECMD         = nmake
HOST            = PC
BUILD           = yes
SYS_TARGET_FILE = raccel.tlc
BUILD_SUCCESS	= *** Created

# Opt in to simplified format by specifying compatible Toolchain
TOOLCHAIN_NAME = [\
    "Microsoft Visual C++ 2019 v16.0 | nmake (64-bit Windows)", \
    "Microsoft Visual C++ 2017 v15.0 | nmake (64-bit Windows)", \
    "Microsoft Visual C++ 2015 v14.0 | nmake (64-bit Windows)"]

#---------------------- Tokens expanded by make_rtw ---------------------------
#
# The following tokens, when wrapped with "|>" and "<|" are expanded by the
# build procedure.
#
#  MODEL_NAME          - Name of the Simulink block diagram
#  MODEL_MODULES       - Any additional generated source modules
#  MAKEFILE_NAME       - Name of makefile created from template makefile <model>.mk
#  MATLAB_ROOT         - Path to where MATLAB is installed.
#  MATLAB_BIN          - Path to MATLAB executable.
#  S_FUNCTIONS_LIB     - List of S-functions libraries to link.
#  BUILDARGS           - Options passed in at the command line.

MODEL                  = Control_Bicopter
MODULES                = Control_Bicopter_ae14a523_1.c Control_Bicopter_ae14a523_1_create.c Control_Bicopter_ae14a523_1_setParameters.c Control_Bicopter_ae14a523_1_asserts.c Control_Bicopter_ae14a523_1_deriv.c Control_Bicopter_ae14a523_1_checkDynamics.c Control_Bicopter_ae14a523_1_output.c Control_Bicopter_ae14a523_1_assembly.c Control_Bicopter_ae14a523_1_computeConstraintError.c Control_Bicopter_ae14a523_1_mode_zero_crossings.c Control_Bicopter_ae14a523_1_geometries.c Control_Bicopter_ae14a523_1_gateway.c pm_printf.c rt_logging.c rt_backsubrr_dbl.c rt_forwardsubrr_dbl.c rt_lu_real.c rt_matrixlib_dbl.c Control_Bicopter.c Control_Bicopter_capi.c Control_Bicopter_data.c Control_Bicopter_tgtconn.c rtGetInf.c rtGetNaN.c rt_nonfinite.c rt_logging_mmi.c rtw_modelmap_utils.c raccel_main_new.c raccel_sup.c raccel_mat.c simulink_solver_api.c raccel_utils.c common_utils.c ext_svr.c updown.c ext_work.c rtiostream_interface.c rtiostream_tcpip.c rtiostream_utils.c
MAKEFILE               = Control_Bicopter.mk
MATLAB_ROOT            = C:\Program Files\MATLAB\R2019b
ALT_MATLAB_ROOT        = C:\PROGRA~1\MATLAB\R2019b
MATLAB_BIN             = C:\Program Files\MATLAB\R2019b\bin
ALT_MATLAB_BIN         = C:\PROGRA~1\MATLAB\R2019b\bin
MASTER_ANCHOR_DIR      = 
START_DIR              = H:\4-project\matlab\bicopter_sim
S_FUNCTIONS_LIB        = $(MATLAB_ROOT)\extern\lib\win64\microsoft\libmwcoder_target_services.lib $(MATLAB_ROOT)\extern\lib\win64\microsoft\libmwcoder_ParamTuningTgtAppSvc.lib
BUILDARGS              =  RSIM_SOLVER_SELECTION=2 EXTMODE_STATIC_ALLOC=0 EXTMODE_STATIC_ALLOC_SIZE=1000000 EXTMODE_TRANSPORT=0 TMW_EXTMODE_TESTING=0 COMBINE_OUTPUT_UPDATE_FCNS=0 INCLUDE_MDL_TERMINATE_FCN=1 MULTI_INSTANCE_CODE=0 MODELREF_TARGET_TYPE=NONE ISPROTECTINGMODEL=NOTPROTECTING OPTS="-DTGTCONN -DON_TARGET_WAIT_FOR_START=0"
RSIM_PARAMETER_LOADING = 1
ENABLE_SLEXEC_SSBRIDGE = 1
VISUAL_VER             = 16.0

SOLVER              = 
PCMATLABROOT        = C:\\Program Files\\MATLAB\\R2019b

MODELREFS            = 

OPTIMIZATION_FLAGS   = /DNDEBUG /Od /Oy-
ADDITIONAL_LDFLAGS   = 
DEFINES_CUSTOM       = -DEXT_MODE -DIS_RAPID_ACCEL 
SYSTEM_LIBS          = /LIBPATH:"C:\Program Files\MATLAB\R2019b\extern\lib\win64\microsoft" libmwipp.lib libut.lib libmx.lib libmex.lib libmat.lib libmwmathutil.lib libmwslexec_simbridge.lib libmwsl_fileio.lib libmwsigstream.lib libmwsl_AsyncioQueue.lib libmwsl_services.lib libmwsdi_raccel.lib libmwi18n.lib libmwsl_simtarget_instrumentation.lib libfixedpoint.lib libmwslexec_simlog.lib

MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
# To enable debugging:
# set DEBUG_BUILD = 1
DEBUG_BUILD             = 0

#--------------------------- Model and reference models -----------------------
MODELREF_LINK_LIBS        = 
MODELREF_LINK_RSPFILE     = Control_Bicopter_ref.rsp
COMPILER_COMMAND_FILE     = Control_Bicopter_comp.rsp
RELATIVE_PATH_TO_ANCHOR   = ..\..\..
GLOBAL_TIMING_ENGINE      = 0

!if "$(MATLAB_ROOT)" != "$(ALT_MATLAB_ROOT)"
MATLAB_ROOT = $(ALT_MATLAB_ROOT)
!endif
!if "$(MATLAB_BIN)" != "$(ALT_MATLAB_BIN)"
MATLAB_BIN = $(ALT_MATLAB_BIN)
!endif
MATLAB_ARCH_BIN = $(MATLAB_BIN)\win64

#---------------------------Solver---------------------------------------------

RSIM_WITH_SL_SOLVER = 1

#--------------------------- Tool Specifications ------------------------------
!include $(MATLAB_ROOT)\rtw\c\tools\vctools.mak

CMD_FILE             = $(MODEL).rsp

#------------------------------ Include Path ----------------------------------

# Additional includes


INCLUDE = .;$(RELATIVE_PATH_TO_ANCHOR);$(BUILDINFO_INCLUDES);$(INCLUDE)

#----------------------------- Parameter Tuning -------------------------------
PARAM_CC_OPTS = -DRSIM_PARAMETER_LOADING

#----------------- Compiler and Linker Options --------------------------------

# Optimization Options
OPT_OPTS = $(DEFAULT_OPT_OPTS)

# General User Options
!if "$(DEBUG_BUILD)" == "0"
DBG_FLAG =
!else
#   Set OPT_OPTS=-Zi and any additional flags for debugging
DBG_FLAG = -Zi
!endif

!if "$(OPTIMIZATION_FLAGS)" != ""
CC_OPTS = $(OPTS)  $(OPTIMIZATION_FLAGS)
!else
CC_OPTS = $(OPT_OPTS) $(OPTS) 
!endif


CPP_REQ_DEFINES = -DMODEL=$(MODEL) -DHAVESTDIO

CPP_REQ_DEFINES = $(CPP_REQ_DEFINES) -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=$(MODEL_HAS_DYNAMICALLY_LOADED_SFCNS)

CPP_REQ_DEFINES = $(CPP_REQ_DEFINES) \
                  -DNRT -DRSIM_WITH_SL_SOLVER

!if "$(ENABLE_SLEXEC_SSBRIDGE)" != "0"
CPP_REQ_DEFINES = $(CPP_REQ_DEFINES) -DENABLE_SLEXEC_SSBRIDGE=$(ENABLE_SLEXEC_SSBRIDGE)
!endif

# Uncomment this line to move warning level to W4
# cflags = $(cflags:W3=W4)
CFLAGS   = $(cflags) @$(COMPILER_COMMAND_FILE) $(cvarsdll) $(PARAM_CC_OPTS) /wd4996 \
	   $(DBG_FLAG) $(CC_OPTS) $(DEFINES_CUSTOM) $(CPP_REQ_DEFINES) \
	   $(USER_INCLUDES)
CPPFLAGS = $(cflags) @$(COMPILER_COMMAND_FILE) $(cvarsdll) $(PARAM_CC_OPTS) \
	   /wd4996 /EHsc- $(DBG_FLAG) $(CPP_OPTS) $(CC_OPTS) $(DEFINES_CUSTOM) \
	   $(CPP_REQ_DEFINES) $(USER_INCLUDES)
LDFLAGS  = $(ldebug) $(conflags) $(conlibs) $(ADDITIONAL_LDFLAGS)

#----------------------------- Source Files -----------------------------------

#Standalone executable
PRODUCT   = $(MODEL).exe	
BUILD_PRODUCT_TYPE = executable
REQ_SRCS  = $(MODULES)

USER_SRCS =

SRCS = $(REQ_SRCS) $(USER_SRCS)

OBJS_CPP_UPPER = $(SRCS:.CPP=.obj)
OBJS_CPP_LOWER = $(OBJS_CPP_UPPER:.cpp=.obj)
OBJS_C_UPPER = $(OBJS_CPP_LOWER:.C=.obj)
OBJS = $(OBJS_C_UPPER:.c=.obj)

# ------------------------- Additional Libraries ------------------------------

LIBS =

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\sm\ssci\lib\win64\sm_ssci_vcx64.lib
!else
LIBS = $(LIBS) sm_ssci.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\sm\core\lib\win64\sm_vcx64.lib
!else
LIBS = $(LIBS) sm.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\pm_math\lib\win64\pm_math_vcx64.lib
!else
LIBS = $(LIBS) pm_math.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\simscape\engine\sli\lib\win64\ssc_sli_vcx64.lib
!else
LIBS = $(LIBS) ssc_sli.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\simscape\engine\core\lib\win64\ssc_core_vcx64.lib
!else
LIBS = $(LIBS) ssc_core.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\network_engine\lib\win64\ne_vcx64.lib
!else
LIBS = $(LIBS) ne.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\common\math\core\lib\win64\mc_vcx64.lib
!else
LIBS = $(LIBS) mc.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\common\external\library\lib\win64\ex_vcx64.lib
!else
LIBS = $(LIBS) ex.lib
!endif

!if "$(OPT_OPTS)" == "$(DEFAULT_OPT_OPTS)"
LIBS = $(LIBS) $(MATLAB_ROOT)\toolbox\physmod\common\foundation\core\lib\win64\pm_vcx64.lib
!else
LIBS = $(LIBS) pm.lib
!endif


LIBMWIPP = $(MATLAB_ROOT)\lib\win64\libmwipp.lib
LIBS = $(LIBS) $(LIBMWIPP)

#--------------------------------- Rules --------------------------------------
all: set_environment_variables $(PRODUCT)

#--- Stand-alone model ---
$(PRODUCT) : $(OBJS) $(LIBS) $(MODELREF_LINK_LIBS)
	$(LD) $(LDFLAGS) $(S_FUNCTIONS_LIB) $(LIBS) $(SYSTEM_LIBS) \
		@$(CMD_FILE) @$(MODELREF_LINK_RSPFILE) -out:$@
	@cmd /C "echo $(BUILD_SUCCESS) $(BUILD_PRODUCT_TYPE): $@"

{$(MATLAB_ROOT)\toolbox\physmod\sm\ssci\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\sm\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\pm_math\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\engine\sli\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\engine\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\compiler\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\network_engine\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\math\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\lang\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\external\library\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\foundation\core\c\win64}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\rtw\c\src}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\simulink\src}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.c.obj :
	$(CC) $(CFLAGS) $<

{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.c.obj :
	$(CC) $(CFLAGS) $<

rt_logging.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\rt_logging.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\rt_logging.c"

raccel_main_new.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_main_new.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_main_new.c"

raccel_sup.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_sup.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_sup.c"

raccel_mat.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_mat.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\raccel\raccel_mat.c"

simulink_solver_api.obj : "C:\Program Files\MATLAB\R2019b\simulink\include\simulink_solver_api.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\simulink\include\simulink_solver_api.c"

raccel_utils.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\rapid\raccel_utils.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\rapid\raccel_utils.c"

common_utils.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\rapid\common_utils.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\rapid\common_utils.c"

ext_svr.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\ext_svr.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\ext_svr.c"

updown.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\updown.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\updown.c"

ext_work.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\ext_work.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\ext_work.c"

rtiostream_interface.obj : "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\rtiostream_interface.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\rtw\c\src\ext_mode\common\rtiostream_interface.c"

rtiostream_tcpip.obj : "C:\Program Files\MATLAB\R2019b\toolbox\coder\rtiostream\src\rtiostreamtcpip\rtiostream_tcpip.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\toolbox\coder\rtiostream\src\rtiostreamtcpip\rtiostream_tcpip.c"

rtiostream_utils.obj : "C:\Program Files\MATLAB\R2019b\toolbox\coder\rtiostream\src\utils\rtiostream_utils.c"
	$(CC) $(CFLAGS) "C:\Program Files\MATLAB\R2019b\toolbox\coder\rtiostream\src\utils\rtiostream_utils.c"



{$(MATLAB_ROOT)\toolbox\physmod\sm\ssci\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\sm\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\pm_math\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\engine\sli\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\engine\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\simscape\compiler\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\network_engine\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\math\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\lang\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\external\library\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\physmod\common\foundation\core\c\win64}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\rtw\c\src}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\simulink\src}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.cpp.obj :
	$(CC) $(CPPFLAGS) $<



# Put these rule last, otherwise nmake will check toolboxes first

{$(RELATIVE_PATH_TO_ANCHOR)}.cpp.obj :
	$(CC) $(CPPFLAGS) $<

.cpp.obj :
	$(CC) $(CPPFLAGS) $<

{$(RELATIVE_PATH_TO_ANCHOR)}.c.obj :
	$(CC) $(CFLAGS) $<

.c.obj :
	$(CC) $(CFLAGS) $<

set_environment_variables:
	@set INCLUDE=$(INCLUDE)
	@set LIB=$(LIB)


# Libraries:



MODULES_sm_ssci = \
    sm_ssci_3dd14f0a.obj \
    sm_ssci_646478c5.obj \
    sm_ssci_916e6db1.obj \
    sm_ssci_b2b6b422.obj \
    sm_ssci_c16a187b.obj \


sm_ssci.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_sm_ssci)
	$(LIBCMD) /nologo /out:$@ $(MODULES_sm_ssci)

MODULES_sm = \
    sm_1ade6722.obj \
    sm_1dba7276.obj \
    sm_1e022f29.obj \
    sm_2567b5ea.obj \
    sm_26df822b.obj \
    sm_346a6262.obj \
    sm_3c02344f.obj \
    sm_440126a7.obj \
    sm_45dc522c.obj \
    sm_5763bdb7.obj \
    sm_6166f1eb.obj \
    sm_62d41fb5.obj \
    sm_67d72683.obj \
    sm_6fbd150d.obj \
    sm_71b23225.obj \
    sm_73d210b9.obj \
    sm_7a6fe4d7.obj \
    sm_8bba0509.obj \
    sm_916806b3.obj \
    sm_9abcb56e.obj \
    sm_a0028316.obj \
    sm_a264a430.obj \
    sm_aa059a3a.obj \
    sm_acba2496.obj \
    sm_badd8656.obj \
    sm_bc63e36c.obj \
    sm_bfda6e42.obj \
    sm_c0ba649d.obj \
    sm_c160a102.obj \
    sm_c60cd1fd.obj \
    sm_c7d3720c.obj \
    sm_c7d6c0d7.obj \
    sm_cabca465.obj \
    sm_d3d79f0f.obj \
    sm_d3d946fd.obj \
    sm_e2bea5a2.obj \
    sm_e8bab6d7.obj \
    sm_efdfa66e.obj \


sm.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_sm)
	$(LIBCMD) /nologo /out:$@ $(MODULES_sm)

MODULES_pm_math = \
    pm_math_1966ea7d.obj \
    pm_math_1ad202b7.obj \
    pm_math_1c69d5b2.obj \
    pm_math_2cdd2951.obj \
    pm_math_3463da5d.obj \
    pm_math_360e4b46.obj \
    pm_math_48bd51fb.obj \
    pm_math_500a14cd.obj \
    pm_math_5a01dda4.obj \
    pm_math_646fa971.obj \
    pm_math_8d05b7c0.obj \
    pm_math_a001e9ec.obj \
    pm_math_b7b980b1.obj \
    pm_math_bad43c87.obj \
    pm_math_d1be0f30.obj \
    pm_math_da630bd2.obj \
    pm_math_f760e8f6.obj \


pm_math.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_pm_math)
	$(LIBCMD) /nologo /out:$@ $(MODULES_pm_math)

MODULES_ssc_sli = \
    ssc_sli_0763c151.obj \
    ssc_sli_0bd269e6.obj \
    ssc_sli_0d6d0780.obj \
    ssc_sli_136b443c.obj \
    ssc_sli_140b3534.obj \
    ssc_sli_15d5f6ce.obj \
    ssc_sli_1db813e8.obj \
    ssc_sli_2bbd58a4.obj \
    ssc_sli_2f6ea1cd.obj \
    ssc_sli_360cfd63.obj \
    ssc_sli_43618287.obj \
    ssc_sli_466b08dd.obj \
    ssc_sli_496799bd.obj \
    ssc_sli_4e028390.obj \
    ssc_sli_51dbd3b5.obj \
    ssc_sli_550a4805.obj \
    ssc_sli_5a0cb974.obj \
    ssc_sli_5d63aeeb.obj \
    ssc_sli_62d81790.obj \
    ssc_sli_77063d8b.obj \
    ssc_sli_7a618260.obj \
    ssc_sli_880e593a.obj \
    ssc_sli_89d0f30a.obj \
    ssc_sli_8a64c4e2.obj \
    ssc_sli_93019ea6.obj \
    ssc_sli_9abcdb7f.obj \
    ssc_sli_9b67747c.obj \
    ssc_sli_9c030181.obj \
    ssc_sli_c7dda239.obj \
    ssc_sli_d064c978.obj \
    ssc_sli_d80c44d2.obj \
    ssc_sli_dcd66f69.obj \
    ssc_sli_e66fe6d5.obj \
    ssc_sli_e7b327bb.obj \
    ssc_sli_eb0a5702.obj \
    ssc_sli_f6bd9cc8.obj \
    ssc_sli_f9b5dbc5.obj \
    ssc_sli_fa0ce53e.obj \
    ssc_sli_fbdf29da.obj \


ssc_sli.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_ssc_sli)
	$(LIBCMD) /nologo /out:$@ $(MODULES_ssc_sli)

MODULES_ssc_core = \
    ssc_core_01dcc633.obj \
    ssc_core_04da2c69.obj \
    ssc_core_05058dd9.obj \
    ssc_core_06ba68a6.obj \
    ssc_core_09b5fa6e.obj \
    ssc_core_0bd666aa.obj \
    ssc_core_0f019bd9.obj \
    ssc_core_0f0420a6.obj \
    ssc_core_18bf4d77.obj \
    ssc_core_1b0cafd5.obj \
    ssc_core_1c6b0332.obj \
    ssc_core_1fd25120.obj \
    ssc_core_24b4cdee.obj \
    ssc_core_2568b075.obj \
    ssc_core_280c0222.obj \
    ssc_core_2cd54448.obj \
    ssc_core_3169e4b7.obj \
    ssc_core_360a4baf.obj \
    ssc_core_37d4ea84.obj \
    ssc_core_40dfdbdc.obj \
    ssc_core_41017299.obj \
    ssc_core_4666b45b.obj \
    ssc_core_48b08af1.obj \
    ssc_core_48b1386a.obj \
    ssc_core_4965213d.obj \
    ssc_core_4ad9135b.obj \
    ssc_core_4db6bd68.obj \
    ssc_core_4db86fcc.obj \
    ssc_core_4e03e39d.obj \
    ssc_core_4e04eecd.obj \
    ssc_core_54d55ae9.obj \
    ssc_core_5505224d.obj \
    ssc_core_56b1a2bf.obj \
    ssc_core_576cd129.obj \
    ssc_core_59b034b8.obj \
    ssc_core_5d6ba758.obj \
    ssc_core_67d1f118.obj \
    ssc_core_68da074b.obj \
    ssc_core_6b6b89d2.obj \
    ssc_core_6dd833f3.obj \
    ssc_core_73d9c2b7.obj \
    ssc_core_76d825be.obj \
    ssc_core_79dd08ab.obj \
    ssc_core_7a613edb.obj \
    ssc_core_83db8762.obj \
    ssc_core_856738f2.obj \
    ssc_core_8569edc5.obj \
    ssc_core_89d7fa79.obj \
    ssc_core_8a6471dc.obj \
    ssc_core_8d0064b8.obj \
    ssc_core_96061071.obj \
    ssc_core_97d767fe.obj \
    ssc_core_990fe1a4.obj \
    ssc_core_9b607b15.obj \
    ssc_core_9c016445.obj \
    ssc_core_9c01d168.obj \
    ssc_core_9dd110ad.obj \
    ssc_core_9fb0e229.obj \
    ssc_core_9fb25b4f.obj \
    ssc_core_a1d393be.obj \
    ssc_core_a4d4c45e.obj \
    ssc_core_a4da1d0a.obj \
    ssc_core_a6b78ccc.obj \
    ssc_core_a76299bc.obj \
    ssc_core_a867d880.obj \
    ssc_core_a9bf1ff2.obj \
    ssc_core_abd05c18.obj \
    ssc_core_abd5e7b4.obj \
    ssc_core_acb64294.obj \
    ssc_core_acb6462e.obj \
    ssc_core_b1038cbb.obj \
    ssc_core_b10e34f4.obj \
    ssc_core_b2b3b239.obj \
    ssc_core_b402b40d.obj \
    ssc_core_b7b88213.obj \
    ssc_core_b96ebc21.obj \
    ssc_core_bc648043.obj \
    ssc_core_c3003040.obj \
    ssc_core_c5b050d7.obj \
    ssc_core_c5b63cb2.obj \
    ssc_core_c607b660.obj \
    ssc_core_c8d83e88.obj \
    ssc_core_cab615c8.obj \
    ssc_core_cabdc251.obj \
    ssc_core_cc067f58.obj \
    ssc_core_ce6a84bb.obj \
    ssc_core_d06d763c.obj \
    ssc_core_d3d34d7c.obj \
    ssc_core_d70a6a09.obj \
    ssc_core_d807fa59.obj \
    ssc_core_dcda6edd.obj \
    ssc_core_deb7fd8d.obj \
    ssc_core_e0d0866d.obj \
    ssc_core_e2b61d72.obj \
    ssc_core_e400c1c2.obj \
    ssc_core_ead8f455.obj \
    ssc_core_ee000fbe.obj \
    ssc_core_ee01086d.obj \
    ssc_core_ee0f0141.obj \
    ssc_core_f9b6dbed.obj \
    ssc_core_fa09e9e6.obj \
    ssc_core_fbd34e62.obj \
    ssc_core_ff06d9a4.obj \


ssc_core.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_ssc_core)
	$(LIBCMD) /nologo /out:$@ $(MODULES_ssc_core)

MODULES_ne = \
    ne_59b4e14a.obj \


ne.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_ne)
	$(LIBCMD) /nologo /out:$@ $(MODULES_ne)

MODULES_mc = \
    mc_026e4f4b.obj \
    mc_03b98f6f.obj \
    mc_0764157d.obj \
    mc_0bd30dee.obj \
    mc_0d6fd085.obj \
    mc_0ed73c49.obj \
    mc_10d4ab75.obj \
    mc_11086079.obj \
    mc_110e6c6c.obj \
    mc_140e3c4c.obj \
    mc_15d12d95.obj \
    mc_15d828ca.obj \
    mc_220ba961.obj \
    mc_2565d6e0.obj \
    mc_2a642f54.obj \
    mc_2a6a9b24.obj \
    mc_2bbf87e3.obj \
    mc_2cdc96b4.obj \
    mc_32d501e3.obj \
    mc_32dc008a.obj \
    mc_3b6a945d.obj \
    mc_3e66abdf.obj \
    mc_4105189f.obj \
    mc_47b8cebe.obj \
    mc_47b91db1.obj \
    mc_4b0301c6.obj \
    mc_4c6117e3.obj \
    mc_51d4094e.obj \
    mc_52623861.obj \
    mc_52688a58.obj \
    mc_53b1fc84.obj \
    mc_550847c3.obj \
    mc_5766048f.obj \
    mc_59b6e413.obj \
    mc_5d65cd86.obj \
    mc_5edd42ef.obj \
    mc_630208f8.obj \
    mc_630dda0e.obj \
    mc_67da200d.obj \
    mc_67da4f41.obj \
    mc_6b6d311a.obj \
    mc_6e61d16c.obj \
    mc_6fb1c336.obj \
    mc_7809a65c.obj \
    mc_7a613aec.obj \
    mc_7bbf41f0.obj \
    mc_7cd58f0b.obj \
    mc_7cdbe436.obj \
    mc_7d0547c8.obj \
    mc_7d099de7.obj \
    mc_7eb21b39.obj \
    mc_81b0ada5.obj \
    mc_81b5717e.obj \
    mc_870ec75e.obj \
    mc_89d597cf.obj \
    mc_90b6aa0a.obj \
    mc_95b62b73.obj \
    mc_9ab7d9b0.obj \
    mc_9b6376d1.obj \
    mc_9b6c1529.obj \
    mc_a2647600.obj \
    mc_a26bab1a.obj \
    mc_a3b90582.obj \
    mc_a865d1dd.obj \
    mc_acb3fad7.obj \
    mc_af0cc4c9.obj \
    mc_b0de9cbc.obj \
    mc_b362c5eb.obj \
    mc_b7b03d44.obj \
    mc_b96a0bad.obj \
    mc_bb0520ee.obj \
    mc_bdbb9b78.obj \
    mc_c2dbf4b2.obj \
    mc_c8d25d23.obj \
    mc_cab8a1f9.obj \
    mc_ce6656ce.obj \
    mc_d20085b7.obj \
    mc_d9d38185.obj \
    mc_dbbb14d2.obj \
    mc_dcdddfae.obj \
    mc_debb448f.obj \
    mc_e7bc2f1a.obj \
    mc_e969ae87.obj \
    mc_edbf543c.obj \
    mc_ee000fbe.obj \
    mc_efdea3a7.obj \
    mc_f3be157c.obj \
    mc_fcb15a9b.obj \
    mc_fd619d14.obj \
    mc_fd6341bb.obj \


mc.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_mc)
	$(LIBCMD) /nologo /out:$@ $(MODULES_mc)

MODULES_ex = \
    ex_04d5441d.obj \
    ex_136645f8.obj \
    ex_17bc61ac.obj \
    ex_18b4440a.obj \
    ex_2ebcd5b2.obj \
    ex_316a81de.obj \
    ex_40d5be33.obj \
    ex_47b11894.obj \
    ex_690b7cd0.obj \
    ex_79d100f1.obj \
    ex_8a6fc761.obj \
    ex_b2b40ad5.obj \
    ex_bb0efd4b.obj \
    ex_debffef2.obj \
    ex_e40d74b8.obj \
    ex_f866102d.obj \


ex.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_ex)
	$(LIBCMD) /nologo /out:$@ $(MODULES_ex)

MODULES_pm = \
    pm_09bc42e2.obj \
    pm_14098e54.obj \
    pm_26dc3230.obj \
    pm_4fd5f5b9.obj \
    pm_fed8c2c9.obj \


pm.lib : rtw_proj.tmw $(MAKEFILE) $(MODULES_pm)
	$(LIBCMD) /nologo /out:$@ $(MODULES_pm)



#----------------------------- Dependencies -----------------------------------

$(OBJS) : $(MAKEFILE) rtw_proj.tmw
