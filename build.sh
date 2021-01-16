#!/bin/bash

export KERNELNAME=Lithium

export LOCALVERSION=

export KBUILD_BUILD_USER=zRyu

export KBUILD_BUILD_HOST=ndeooo.xyz

export TOOLCHAIN=gcc

export DEVICES=X00T/TD

source helper

gen_toolchain

send_msg "⏳ Start building ${KERNELNAME} ${LOCALVERSION} for ${DEVICES}..."

START=$(date +"%s")

for i in ${DEVICES//,/ }
do 

	build ${i} -X00T

        build ${i} -X00T

done

END=$(date +"%s")
 
DIFF=$(( END - START ))
