#!/bin/bash
txt_start=
env_start=
env_length=
oct_env_start=
oct_txt_start=
oct_seek_pos=
seek_pos=
header_name=
show=1

check_null()
{
    if [ -z $2 ]; then
        echo "${1} null"
        show=0
        exit
    fi    
}

# don't change grep pattern '.text     ' to '.text'
txt_start=`${CROSS_COMPILE}objdump -h u-boot | grep '.text     ' | awk '{ print $4}'`
check_null txt_start $txt_start

env_start=`${CROSS_COMPILE}objdump -x u-boot | grep default_environment$ | awk '{ print $1 }' `
check_null env_start $env_start

env_length=`${CROSS_COMPILE}objdump -x u-boot | grep default_environment$ | awk '{ print $5 }'`
check_null env_length $env_length

oct_env_start=`printf "%d" 0x$env_start`
check_null oct_env_start $oct_env_start

oct_txt_start=`printf "%d" 0x$txt_start`
check_null oct_txt_start $oct_txt_start

oct_seek_pos=$(expr ${oct_env_start} - ${oct_txt_start} )
oct_seek_pos=$(expr ${oct_seek_pos} - 64 )
check_null oct_seek_pos $oct_seek_pos

seek_pos=`printf "%x" ${oct_seek_pos}`
check_null seek_pos $seek_pos

header_name="${seek_pos}-${env_length}"
if [ "x$show" = "x1" ]; then
    echo $header_name
fi
