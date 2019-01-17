###############################################################################
#
# 
#
###############################################################################
THIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
x86_64_accton_miniomp_INCLUDES := -I $(THIS_DIR)inc
x86_64_accton_miniomp_INTERNAL_INCLUDES := -I $(THIS_DIR)src
x86_64_accton_miniomp_DEPENDMODULE_ENTRIES := init:x86_64_accton_miniomp ucli:x86_64_accton_miniomp

