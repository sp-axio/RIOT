#!/bin/sh

if [ `uname` = "Linux" ]; then
    echo "### Flashing axio-builder-ms500 ###"
	BINFILE=${ELFFILE%.elf}.bin
	SIGFILE=${ELFFILE%.elf}.bin.sig
#SIGFILE=`basename ${ELFFILE%.elf}.bin.sig`
	"${OBJCOPY}" --gap-fill=0xff -O binary "${ELFFILE}" "${BINFILE}"
	echo "${SIGNFW}" "${BINFILE}" "${PKEY}"
	"${SIGNFW}" "${BINFILE}" "${PKEY}"

    echo "echo:${AXTOOL}" -f "${SIGFILE}" "${PORT}"
    "${AXTOOL}" -f "${SIGFILE}" "${PORT}"
else
    echo "CAUTION: No flash tool for your host system found!"
fi
