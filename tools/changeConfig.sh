#!/bin/bash
if [ "$#" != "1" ]; then
  echo "usage: changeConfig.sh <config_path>"
  exit -1
fi
configFile=$1
if [ "$OBSOLETE_KEEP_ADB_SECURE" != "1" ] || \
   [ "$EUCLID_BUILD_INSECURE_KERNEL" == "1" ] || \
   [ "$TARGET_BUILD_VARIANT" != "user" ] ; then \
  sed -i '/CONFIG_SECURITY_SELINUX_DEVELOP.*/d' $configFile
  echo -e "CONFIG_SECURITY_SELINUX_DEVELOP=y\n" >> $configFile
else
  sed -i '/CONFIG_SECURITY_SELINUX_DEVELOP.*/d' $configFile
fi
echo  "===OBSOLETE_KEEP_ADB_SECURE = $OBSOLETE_KEEP_ADB_SECURE"
echo  "===EUCLID_BUILD_INSECURE_KERNEL = $EUCLID_BUILD_INSECURE_KERNEL"
echo  "===TARGET_BUILD_VARIANT = $TARGET_BUILD_VARIANT"

function setting_kernel_config()
{
    config=$1
    value=$2
    string=$3

    sed -i "/${config}.*/d" $configFile

    if [ $string -eq 0 ]
    then
        echo -e "${config}=${value}" >> $configFile
    else
        echo -e "${config}=\"${value}\"" >> $configFile
    fi
}

if [[ "$TARGET_MEMLEAK_DETECT_TEST" == 1 ]]
then
    setting_kernel_config CONFIG_OPLUS_MEMLEAK_DETECT true 1
    setting_kernel_config CONFIG_MEMLEAK_DETECT_THREAD y 0
fi
